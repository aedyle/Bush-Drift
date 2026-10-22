/* Bush Drift's road: see route.h. */
#include "route.h"
#include "tuning.h"
#include "vec.h"
#ifdef __EMSCRIPTEN__
/* the browser build (session 3b): no worker thread; route_stream_take builds the nearest wanted chunk itself */
#define pthread_mutex_lock(m) ((void)0)
#define pthread_mutex_unlock(m) ((void)0)
#define pthread_cond_signal(c) ((void)0)
typedef int pthread_t, pthread_mutex_t, pthread_cond_t;
#define PTHREAD_MUTEX_INITIALIZER 0
#define PTHREAD_COND_INITIALIZER 0
#else
#include <pthread.h>
#endif
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ------------------------------------------------------------- loading */

static int read_route_csv(Route *R, const char *path)
{
    FILE *f = fopen(path, "r");
    if (!f) { fprintf(stderr, "route: no %s (run scripts/build_all.sh)\n", path); return 0; }
    char line[512];
    int cap = 0;
    if (!fgets(line, sizeof(line), f)) { fclose(f); return 0; } /* header */
    while (fgets(line, sizeof(line), f)) {
        float s, x, y, e, ed, hd, k;
        if (sscanf(line, "%f,%f,%f,%f,%f,%f,%f", &s, &x, &y, &e, &ed, &hd, &k) != 7) continue;
        if (R->n == cap) {
            cap = cap ? cap * 2 : 4096;
            R->x = realloc(R->x, sizeof(float) * cap); R->z = realloc(R->z, sizeof(float) * cap);
            R->h = realloc(R->h, sizeof(float) * cap); R->hdem = realloc(R->hdem, sizeof(float) * cap);
            R->head = realloc(R->head, sizeof(float) * cap); R->curv = realloc(R->curv, sizeof(float) * cap);
        }
        R->x[R->n] = x; R->z[R->n] = y; R->h[R->n] = e; R->hdem[R->n] = ed; R->head[R->n] = hd; R->curv[R->n] = k;
        R->n++;
    }
    fclose(f);
    if (R->n < 10) { fprintf(stderr, "route: %s has %d rows\n", path, R->n); return 0; }
    R->length = (float)(R->n - 1) * RT_DS;
    return 1;
}

/* meta.txt: one "key value [value]" a line (scripts/build_route.py). Headings there are map headings
 * (0 = east, anticlockwise); the car's yaw is 0 = north, positive towards east. */
static int pad_h_given;   /* meta gave the car park's level (session 3b) */
static int read_meta(Route *R, const char *path, float *pad_elev)
{
    pad_h_given = 0;
    FILE *f = fopen(path, "r");
    if (!f) { fprintf(stderr, "route: no %s\n", path); return 0; }
    char line[256], key[64];
    float a, b;
    int got = 0;
    *pad_elev = NAN;
    R->clearing_r = 40.0f; R->lane_w = 3.5f; R->lane_lo = R->lane_hi = 3.5f;
    while (fgets(line, sizeof(line), f)) {
        int n = sscanf(line, "%63s %f %f", key, &a, &b);
        if (n < 2) continue;
        if (!strcmp(key, "lane_width_m")) R->lane_w = a;
        else if (!strcmp(key, "lane_width_range") && n >= 3) { R->lane_lo = a; R->lane_hi = b; }
        else if (!strcmp(key, "start_x")) { R->start_x = a; got |= 1; }
        else if (!strcmp(key, "start_y")) { R->start_z = a; got |= 2; }
        else if (!strcmp(key, "start_heading")) { R->start_yaw = (float)(M_PI / 2) - a; got |= 4; }
        else if (!strcmp(key, "pad_x")) R->pad_x = a;
        else if (!strcmp(key, "pad_y")) R->pad_z = a;
        else if (!strcmp(key, "pad_r")) R->pad_r = a;
        else if (!strcmp(key, "pad_h")) { R->pad_h = a; pad_h_given = 1; }   /* session 3b: the car park's own level */
        else if (!strcmp(key, "pad_elev")) *pad_elev = a;
        else if (!strcmp(key, "ford_s")) R->ford_s = a;
        else if (!strcmp(key, "bridge_s0")) R->bridge_s0 = a;
        else if (!strcmp(key, "bridge_s1")) R->bridge_s1 = a;
        else if (!strcmp(key, "hut_x")) R->hut_x = a;
        else if (!strcmp(key, "hut_y")) R->hut_z = a;
        else if (!strcmp(key, "clearing_r")) R->clearing_r = a;
    }
    fclose(f);
    if (got != 7) fprintf(stderr, "route: meta.txt has no start pose; starting on the road\n");
    return got == 7 ? 2 : 1;
}

static int read_grid(Route *R, const char *tpath, const char *cpath)
{
    FILE *f = fopen(tpath, "rb");
    if (!f) { fprintf(stderr, "route: no %s\n", tpath); return 0; }
    char magic[4]; int32_t nx, ny; float x0, y0, cell;
    if (fread(magic, 1, 4, f) != 4 || memcmp(magic, "BDT1", 4) || fread(&nx, 4, 1, f) != 1 || fread(&ny, 4, 1, f) != 1 ||
        fread(&x0, 4, 1, f) != 1 || fread(&y0, 4, 1, f) != 1 || fread(&cell, 4, 1, f) != 1 || nx <= 1 || ny <= 1) {
        fprintf(stderr, "route: %s is not a BDT1 grid\n", tpath); fclose(f); return 0;
    }
    R->gx = nx; R->gy = ny; R->gx0 = x0; R->gz0 = y0; R->cell = cell;
    size_t N = (size_t)nx * ny;
    R->dem = malloc(N * sizeof(float));
    if (fread(R->dem, sizeof(float), N, f) != N) { fprintf(stderr, "route: %s is short\n", tpath); fclose(f); return 0; }
    fclose(f);
    f = fopen(cpath, "rb");
    if (f) {
        if (fread(magic, 1, 4, f) == 4 && !memcmp(magic, "BDC1", 4) && fread(&nx, 4, 1, f) == 1 && fread(&ny, 4, 1, f) == 1 &&
            nx == R->gx && ny == R->gy && fseek(f, 12, SEEK_CUR) == 0) {
            R->canopy = malloc(N);
            if (fread(R->canopy, 1, N, f) != N) { free(R->canopy); R->canopy = NULL; }
        }
        fclose(f);
    }
    if (!R->canopy) fprintf(stderr, "route: no canopy map (%s): even forest along the track\n", cpath);
    return 1;
}

/* ------------------------------------------------- nearest point on the line */

static void build_buckets(Route *R)
{
    float minx = 1e30f, maxx = -1e30f, minz = 1e30f, maxz = -1e30f;
    for (int i = 0; i < R->n; i++) {
        minx = fminf(minx, R->x[i]); maxx = fmaxf(maxx, R->x[i]);
        minz = fminf(minz, R->z[i]); maxz = fmaxf(maxz, R->z[i]);
    }
    R->bcell = 20.0f;
    R->bx0 = minx - 1; R->bz0 = minz - 1;
    R->bnx = (int)((maxx - R->bx0) / R->bcell) + 2;
    R->bnz = (int)((maxz - R->bz0) / R->bcell) + 2;
    int nb = R->bnx * R->bnz;
    R->bfirst = calloc((size_t)nb + 1, sizeof(int));
    R->blist = malloc(sizeof(int) * (size_t)R->n);
    int *cnt = calloc((size_t)nb, sizeof(int));
    for (int i = 0; i < R->n; i++) {
        int bx = (int)((R->x[i] - R->bx0) / R->bcell), bz = (int)((R->z[i] - R->bz0) / R->bcell);
        cnt[bz * R->bnx + bx]++;
    }
    for (int b = 0; b < nb; b++) R->bfirst[b + 1] = R->bfirst[b] + cnt[b];
    memset(cnt, 0, sizeof(int) * (size_t)nb);
    for (int i = 0; i < R->n; i++) {
        int bx = (int)((R->x[i] - R->bx0) / R->bcell), bz = (int)((R->z[i] - R->bz0) / R->bcell), b = bz * R->bnx + bx;
        R->blist[R->bfirst[b] + cnt[b]++] = i;
    }
    free(cnt);
}

RtNear route_near(const Route *R, float x, float z)
{
    int cx = (int)floorf((x - R->bx0) / R->bcell), cz = (int)floorf((z - R->bz0) / R->bcell);
    float best = 1e30f; int bi = 0;
    for (int k = 0; k < R->bnx + R->bnz + 2; k++) {
        for (int j = cz - k; j <= cz + k; j++) {
            if (j < 0 || j >= R->bnz) continue;
            int step = (j == cz - k || j == cz + k) ? 1 : 2 * k; /* the ring only */
            for (int i = cx - k; i <= cx + k; i += step > 0 ? step : 1) {
                if (i < 0 || i >= R->bnx) continue;
                int b = j * R->bnx + i;
                for (int q = R->bfirst[b]; q < R->bfirst[b + 1]; q++) {
                    int s = R->blist[q];
                    float dx = R->x[s] - x, dz = R->z[s] - z, d = dx * dx + dz * dz;
                    if (d < best) { best = d; bi = s; }
                }
                if (step == 0) break;
            }
        }
        float ring = (float)k * R->bcell;
        if (best < 1e29f && ring * ring > best) break;
    }
    /* project onto the two segments either side of the nearest sample */
    RtNear r = { (float)bi * RT_DS, 0, sqrtf(best), bi };
    float bestd = 1e30f;
    for (int e = -1; e <= 0; e++) {
        int a = bi + e, b = a + 1;
        if (a < 0 || b >= R->n) continue;
        float tx = R->x[b] - R->x[a], tz = R->z[b] - R->z[a], L2 = tx * tx + tz * tz;
        float px = x - R->x[a], pz = z - R->z[a];
        float t = L2 > 0 ? (px * tx + pz * tz) / L2 : 0;
        if ((a > 0 || t > 0) && (b < R->n - 1 || t < 1)) t = clampf(t, 0, 1); /* the two ends extend straight on */
        float fx = R->x[a] + tx * t, fz = R->z[a] + tz * t, dx = x - fx, dz = z - fz, d = dx * dx + dz * dz;
        if (d < bestd) {
            float L = sqrtf(L2);
            bestd = d;
            r.s = ((float)a + t) * RT_DS;
            r.lat = L > 0 ? (tx * (z - R->z[a]) - tz * (x - R->x[a])) / L : 0;
            r.dist = sqrtf(d);
            r.i = t < 0.5f ? a : b;
        }
    }
    return r;
}

/* --------------------------------------------------------------- heights */

static float smooth01(float t) { t = clamp01(t); return t * t * (3.0f - 2.0f * t); }

/* The ford: a shallow dip pressed into the road surface (the smoothed profile rounds the creek off). */
static float ford_dip(const Route *R, float s)
{
    if (R->ford_s <= 0) return 0;
    float d = fabsf(s - R->ford_s) / 9.0f;
    return d >= 1 ? 0 : 0.45f * (0.5f + 0.5f * cosf((float)M_PI * d));
}

float route_road_h(const Route *R, float s)
{
    float u = clampf(s / RT_DS, 0, (float)(R->n - 1));
    int i = (int)u; if (i >= R->n - 1) i = R->n - 2;
    float t = u - (float)i;
    return R->h[i] + (R->h[i + 1] - R->h[i]) * t - ford_dip(R, s);
}

void route_point(const Route *R, float s, float *x, float *z)
{
    float u = clampf(s / RT_DS, 0, (float)(R->n - 1));
    int i = (int)u; if (i >= R->n - 1) i = R->n - 2;
    float t = u - (float)i;
    *x = R->x[i] + (R->x[i + 1] - R->x[i]) * t;
    *z = R->z[i] + (R->z[i + 1] - R->z[i]) * t;
}

float route_heading_at(const Route *R, float s)
{
    int i = (int)clampf(s / RT_DS, 0, (float)(R->n - 2));
    return atan2f(R->z[i + 1] - R->z[i], R->x[i + 1] - R->x[i]);
}

float route_dem(const Route *R, float x, float z)
{
    float u = clampf((x - R->gx0) / R->cell, 0, (float)(R->gx - 1) - 0.001f);
    float v = clampf((z - R->gz0) / R->cell, 0, (float)(R->gy - 1) - 0.001f);
    int i = (int)u, j = (int)v;
    float tu = u - (float)i, tv = v - (float)j;
    const float *d = R->dem + (size_t)j * R->gx + i;
    float a = d[0], b = d[1], c = d[R->gx], e = d[R->gx + 1];
    return (a * (1 - tu) + b * tu) * (1 - tv) + (c * (1 - tu) + e * tu) * tv;
}

float route_half_width(void) { return 0.5f * T.road_width; }

static int in_bridge(const Route *R, float s) { return R->bridge_s1 > R->bridge_s0 && s > R->bridge_s0 - 0.5f && s < R->bridge_s1 + 0.5f; }

/* ---------------------------------------------------------- the map's edge (build B) */
/* The hut's open frost hollow: grid points joined to the hut through open ground (the imagery's canopy under 45%,
 * DEM known, within 230 m of the hut; the lawn round the hut and the car park always), small gaps closed. hut_d is
 * then each grid point's distance to it (a two-pass chamfer, metres). */
/* far.bin (20 m) sampled onto the 5 m grid: heights (kept in hdem_far_tmp until route_load uses them) and forest
 * cover (returned; NULL without the file). */
static unsigned char *far_fill(Route *R, const char *dir)
{
    char p[512]; snprintf(p, sizeof(p), "%s/far.bin", dir);
    FILE *f = fopen(p, "rb");
    if (!f) return NULL;
    char magic[4]; int nx, ny; float x0, y0, cell;
    if (fread(magic, 1, 4, f) != 4 || memcmp(magic, "BDF1", 4) || fread(&nx, 4, 1, f) != 1 || fread(&ny, 4, 1, f) != 1 ||
        fread(&x0, 4, 1, f) != 1 || fread(&y0, 4, 1, f) != 1 || fread(&cell, 4, 1, f) != 1) { fclose(f); return NULL; }
    size_t M = (size_t)nx * ny, N = (size_t)R->gx * R->gy;
    float *dem = malloc(M * 4); unsigned char *cov = malloc(M);
    int ok = fread(dem, 4, M, f) == M && fread(cov, 1, M, f) == M;
    fclose(f);
    if (!ok) { free(dem); free(cov); return NULL; }
    R->hdem_far_tmp = malloc(N * sizeof(float));
    unsigned char *out = malloc(N);
    for (int j = 0; j < R->gy; j++)
        for (int i = 0; i < R->gx; i++) {
            float u = (R->gx0 + (float)i * R->cell - x0) / cell, v = (R->gz0 + (float)j * R->cell - y0) / cell;
            size_t k = (size_t)j * R->gx + i;
            if (u < 0 || v < 0 || u >= (float)(nx - 1) || v >= (float)(ny - 1)) { R->hdem_far_tmp[k] = NAN; out[k] = 255; continue; }
            int a = (int)u, b = (int)v; float tu = u - (float)a, tv = v - (float)b;
            const float *d = dem + (size_t)b * nx + a; const unsigned char *c = cov + (size_t)b * nx + a;
            R->hdem_far_tmp[k] = (d[0] * (1 - tu) + d[1] * tu) * (1 - tv) + (d[nx] * (1 - tu) + d[nx + 1] * tu) * tv;
            out[k] = (unsigned char)((c[0] * (1 - tu) + c[1] * tu) * (1 - tv) + (c[nx] * (1 - tu) + c[nx + 1] * tu) * tv);
        }
    free(dem); free(cov);
    return out;
}

static void build_hollow(Route *R, const unsigned char *farcov)
{
    (void)farcov;   /* the far cover is already in canopy where there was no imagery (route_load) */
    int nx = R->gx, ny = R->gy; size_t N = (size_t)nx * ny;
    R->hut_d = malloc(N * sizeof(float));
    for (size_t k = 0; k < N; k++) R->hut_d[k] = 1e9f;
    if ((R->hut_x == 0 && R->hut_z == 0) || !R->canopy) return;
    unsigned char *in = calloc(N, 1), *ok = calloc(N, 1);
    int *queue = malloc(N * sizeof(int)), qh = 0, qt = 0;
    for (int j = 0; j < ny; j++)
        for (int i = 0; i < nx; i++) {
            float x = R->gx0 + (float)i * R->cell, z = R->gz0 + (float)j * R->cell, dh = hypotf(x - R->hut_x, z - R->hut_z);
            size_t k = (size_t)j * nx + i;
            int open = R->canopy[k] < (R->known[k] ? 115 : 140);   /* past the 5 m data the cover is the far land's (20 m, blurrier: a looser line; 140 takes in the clearing's east arm without leaking) */
            ok[k] = (dh < 45.0f) || (R->pad_r > 0 && hypotf(x - R->pad_x, z - R->pad_z) < R->pad_r + 6.0f) || (dh < 650.0f && open);
        }
    int hi = (int)roundf((R->hut_x - R->gx0) / R->cell), hj = (int)roundf((R->hut_z - R->gz0) / R->cell);
    if (hi < 0 || hj < 0 || hi >= nx || hj >= ny) { free(in); free(ok); free(queue); return; }
    in[(size_t)hj * nx + hi] = 1; queue[qt++] = hj * nx + hi;
    while (qh < qt) {
        int k = queue[qh++], i = k % nx, j = k / nx;
        const int di[4] = { 1, -1, 0, 0 }, dj[4] = { 0, 0, 1, -1 };
        for (int d = 0; d < 4; d++) {
            int a = i + di[d], b = j + dj[d];
            if (a < 0 || b < 0 || a >= nx || b >= ny) continue;
            size_t q = (size_t)b * nx + a;
            if (ok[q] && !in[q]) { in[q] = 1; queue[qt++] = (int)q; }
        }
    }
    /* close gaps of a cell or two (single trees in the paddock): grow by 2, shrink by 2 */
    for (int pass = 0; pass < 4; pass++) {
        unsigned char *nw = malloc(N), grow = pass < 2;
        for (int j = 0; j < ny; j++)
            for (int i = 0; i < nx; i++) {
                size_t k = (size_t)j * nx + i; int any = 0, all = 1;
                for (int b = j - 1; b <= j + 1; b++) for (int a = i - 1; a <= i + 1; a++) {
                    int v = (a >= 0 && b >= 0 && a < nx && b < ny) ? in[(size_t)b * nx + a] : 0;
                    any |= v; all &= v;
                }
                nw[k] = grow ? (unsigned char)any : (unsigned char)all;
            }
        free(in); in = nw;
    }
    R->hollow_cells = 0;
    for (size_t k = 0; k < N; k++) { if (in[k]) { R->hut_d[k] = 0; R->hollow_cells++; } }
    float a = R->cell, b = R->cell * 1.41421f; float *d = R->hut_d;
    for (int j = 0; j < ny; j++) for (int i = 0; i < nx; i++) {
        size_t k = (size_t)j * nx + i; float v = d[k];
        if (i > 0) v = fminf(v, d[k - 1] + a);
        if (j > 0) { v = fminf(v, d[k - nx] + a); if (i > 0) v = fminf(v, d[k - nx - 1] + b); if (i < nx - 1) v = fminf(v, d[k - nx + 1] + b); }
        d[k] = v;
    }
    for (int j = ny - 1; j >= 0; j--) for (int i = nx - 1; i >= 0; i--) {
        size_t k = (size_t)j * nx + i; float v = d[k];
        if (i < nx - 1) v = fminf(v, d[k + 1] + a);
        if (j < ny - 1) { v = fminf(v, d[k + nx] + a); if (i < nx - 1) v = fminf(v, d[k + nx + 1] + b); if (i > 0) v = fminf(v, d[k + nx - 1] + b); }
        d[k] = v;
    }
    free(in); free(ok); free(queue);
}

static float vnoise(float x, float z);
static float hut_dist(const Route *R, float x, float z)
{
    if (!R->hut_d) return 1e9f;
    float u = (x - R->gx0) / R->cell, v = (z - R->gz0) / R->cell;
    if (u < 0 || v < 0 || u >= (float)(R->gx - 1) || v >= (float)(R->gy - 1)) return 1e9f;
    int i = (int)u, j = (int)v; float tu = u - (float)i, tv = v - (float)j;
    const float *d = R->hut_d + (size_t)j * R->gx + i;
    return (d[0] * (1 - tu) + d[1] * tu) * (1 - tv) + (d[R->gx] * (1 - tu) + d[R->gx + 1] * tu) * tv;
}

/* The road's own distance: off the side within the route, from the end points beyond them. */
static float road_dist(const Route *R, RtNear n)
{
    if (n.s < 0) return hypotf(n.s, n.lat);
    if (n.s > R->length) return hypotf(n.s - R->length, n.lat);
    return fabsf(n.lat);
}

/* ---------------------------------------------------------- the creek way (build C) */
/* Solomons Creek from the hut's clearing down to the ford (scripts/build_creek.py): a line every 2 m with the bed's height,
 * a bucket index for its nearest point like the road's, and the widths: the bed (gravel, with water down its middle), and
 * the strip (to the bank either side). Where the creek leaves the clearing the strip is a narrow gap through the forest's
 * edge: the way in. */
static float hut_dist(const Route *R, float x, float z);
static uint32_t hash32(uint32_t x);
static float rnd(uint32_t *s);
static float route_heading_at_creek(const Route *R, float s);
/* Build D: the harder creek's features, all from the line (no randomness that a new build could shift):
 * - climb-outs: each log jam and two fallen trees right across the creek get a raised, rocky bench on their open side
 *   (ck_byp_h above the bed, steep rocky ramps at its ends), and the waterfall's ledge starts with a rocky step up;
 * - three rock gardens (about 40 m of humped rocks across the bed);
 * - fallen trees part way across or along the bank every few hundred metres;
 * - ten markers of the old stock route, evenly down it, and the cattleman's camp on a flat before the ford. */
static float creek_flatness(const Route *R, float s0) { return fabsf(route_creek_bed(R, s0 - 10.0f) - route_creek_bed(R, s0 + 10.0f)); }
static int creek_busy(const Route *R, float s0, float room)   /* near a climb-out, the fall or a rock garden */
{
    for (int b = 0; b < R->ck_nbyp; b++) if (fabsf(s0 - R->ck_byp_s[b]) < room) return 1;
    for (int g = 0; g < R->ck_nrock; g++) if (s0 > R->ck_rock_s0[g] - room && s0 < R->ck_rock_s1[g] + room) return 1;
    return 0;
}
static float creek_flattest(const Route *R, float a, float b, float room)
{
    float bs = -1, bg = 1e9f;
    for (float s0 = a; s0 < b; s0 += 4.0f) { if (creek_busy(R, s0, room)) continue; float g = creek_flatness(R, s0); if (g < bg) { bg = g; bs = s0; } }
    return bs;
}
/* ---- build D, part 2: the creek's things to drive over (see route.h). Stones all the way down, thicker in some stretches;
 * twigs and branches; small rotten logs; stumps and boulders at the bed's edges; mossy fallen trees (some right across, two
 * at each old log jam); and the wet logs in the water (main.c moves them). */
#define TYRE_R 0.45f
static void bit_add(Route *R, int *cap, int type, int var, float x, float z, float yaw, float l, float r, float h)
{
    if (R->ck_nbit == *cap) { *cap = *cap ? *cap * 2 : 4096; R->ck_bit = realloc(R->ck_bit, sizeof(R->ck_bit[0]) * (size_t)*cap); }
    struct RtBit *b = &R->ck_bit[R->ck_nbit++];
    b->x = x; b->z = z; b->yaw = yaw; b->l = l; b->r = r; b->h = h; b->type = (unsigned char)type; b->var = (unsigned char)var;
}
static float bit_reach(const struct RtBit *b) { return b->l + b->r + TYRE_R + 0.2f; }
static float bit_bump(const struct RtBit *b, float x, float z, float Rt)
{
    float dx = x - b->x, dz = z - b->z;
    if (b->type == BIT_TWIGS) return 0;
    if (b->type == BIT_STONE || b->type == BIT_BOULDER) {   /* a dome: a ball of radius r, its top at h */
        float d2 = dx * dx + dz * dz, rr = b->r + Rt; if (d2 >= rr * rr) return 0;
        return fmaxf((b->h - b->r) + sqrtf(rr * rr - d2) - Rt, 0.0f);
    }
    if (b->type == BIT_STUMP) {   /* a short post: flat on top, its edge rounded by the tyre */
        float d = sqrtf(dx * dx + dz * dz); if (d <= b->r) return b->h; if (d >= b->r + Rt) return 0;
        float e = d - b->r; return fmaxf(b->h - Rt + sqrtf(Rt * Rt - e * e), 0.0f);
    }
    /* a ridge along yaw: a branch, a log, a fallen tree (mossy ones taper from their butt at -l) */
    float ax = cosf(b->yaw), az = sinf(b->yaw), t = dx * ax + dz * az, n = -dx * az + dz * ax;
    float tc = clampf(t, -b->l, b->l), d = sqrtf((t - tc) * (t - tc) + n * n), r = b->r;
    if (b->type == BIT_MOSSY) r = b->r * (1.0f - 0.65f * (tc + b->l) / fmaxf(2.0f * b->l, 0.1f));
    float rr = r + Rt; if (d >= rr) return 0;
    return fmaxf(r + sqrtf(rr * rr - d * d) - Rt, 0.0f);
}
float route_creek_bump_r(const Route *R, float x, float z, float Rt)
{
    if (!R->ck_nbit || !R->explore) return 0;
    int i = (int)floorf((x - R->ckg_x0) / 4.0f), j = (int)floorf((z - R->ckg_z0) / 4.0f);
    if (i < 0 || j < 0 || i >= R->ckg_nx || j >= R->ckg_nz) return 0;
    int c = j * R->ckg_nx + i; float h = 0;
    for (int q = R->ckg_first[c]; q < R->ckg_first[c + 1]; q++) h = fmaxf(h, bit_bump(&R->ck_bit[R->ckg_list[q]], x, z, Rt));
    return h;
}
float route_creek_bump(const Route *R, float x, float z) { return route_creek_bump_r(R, x, z, TYRE_R); }
static int bit_clear_of(const Route *R, float s, float x, float z, float room)   /* keep off the markers, the camp, the rifle, the fall's lip, the way in */
{
    if (s < R->ck_exit_s + 50.0f || s > R->ck_len - 30.0f) return 0;
    for (int m = 0; m < R->ck_nmark; m++) if (hypotf(R->ck_mark_x[m] - x, R->ck_mark_z[m] - z) < 4.0f + room) return 0;
    if (R->camp_on && hypotf(R->camp_x - x, R->camp_z - z) < 12.0f + room) return 0;
    if (R->camp_on && hypotf(R->trunk_x - x, R->trunk_z - z) < 4.0f + room) return 0;
    if (fabsf(s - (R->ck_len - 380.0f)) < 6.0f + room) return 0;   /* Quayle's rifle on its gravel bar */
    if (R->ck_fall_s > 0 && fabsf(s - R->ck_fall_s) < 10.0f + room) return 0;
    return 1;
}
static void creek_bits_build(Route *R)
{
    free(R->ck_bit); R->ck_bit = NULL; R->ck_nbit = 0; R->ck_nwet = 0;
    free(R->ckg_first); free(R->ckg_list); R->ckg_first = R->ckg_list = NULL;
    int cap = 0; uint32_t sd = 7777u;
    int nm = 0;
    for (float s0 = R->ck_exit_s + 50.0f; s0 < R->ck_len - 30.0f; s0 += 1.0f) {
        float BW = route_creek_bedw(R, s0), hd = route_heading_at_creek(R, s0), cx, cz; route_creek_point(R, s0, &cx, &cz);
        float dens = T.creek_stones * (0.3f + 0.9f * vnoise(s0 / 110.0f, 3.0f));   /* stones a metre down the creek, more in some stretches */
        for (int k = 0; k < 3; k++) {
            if (rnd(&sd) > dens / 3.0f) continue;
            float lat = (rnd(&sd) * 2.0f - 1.0f) * (BW + 1.2f), x = cx - sinf(hd) * lat, z = cz + cosf(hd) * lat;
            if (!bit_clear_of(R, s0, x, z, 0)) continue;
            float u = rnd(&sd), r = u < 0.7f ? 0.17f + 0.17f * rnd(&sd) : u < 0.95f ? 0.32f + 0.15f * rnd(&sd) : 0.47f + 0.15f * rnd(&sd);
            bit_add(R, &cap, BIT_STONE, (int)(rnd(&sd) * 3.0f) % 3, x, z, rnd(&sd) * 6.28f, 0, r, r * (0.55f + 0.3f * rnd(&sd)));
        }
        if (rnd(&sd) < 1.0f / 5.0f) {   /* twig litter */
            float lat = (rnd(&sd) * 2.0f - 1.0f) * (BW + 2.0f), x = cx - sinf(hd) * lat, z = cz + cosf(hd) * lat;
            if (bit_clear_of(R, s0, x, z, 0)) bit_add(R, &cap, BIT_TWIGS, (int)(rnd(&sd) * 2.0f) % 2, x, z, rnd(&sd) * 6.28f, 0, 0, 0);
        }
        if (rnd(&sd) < 1.0f / 18.0f) {   /* a fallen branch, any way round */
            float lat = (rnd(&sd) * 2.0f - 1.0f) * BW, x = cx - sinf(hd) * lat, z = cz + cosf(hd) * lat;
            int v = (int)(rnd(&sd) * 3.0f) % 3; float L = v == 0 ? 2.6f : v == 1 ? 3.0f : 3.5f;
            if (bit_clear_of(R, s0, x, z, 1.5f)) bit_add(R, &cap, BIT_BRANCH, v, x, z, hd + (rnd(&sd) - 0.5f) * 3.0f, 0.5f * L, 0.08f, 0.16f);
        }
        if (rnd(&sd) < 1.0f / 45.0f) {   /* a small rotten log, mostly across */
            float lat = (rnd(&sd) * 2.0f - 1.0f) * BW * 0.7f, x = cx - sinf(hd) * lat, z = cz + cosf(hd) * lat;
            int v = (int)(rnd(&sd) * 2.0f) % 2; float sc = 0.9f + 0.3f * rnd(&sd), L = (v ? 2.4f : 3.0f) * sc, r = (v ? 0.16f : 0.2f) * sc;
            if (bit_clear_of(R, s0, x, z, 2.0f)) bit_add(R, &cap, BIT_SMALLLOG, v, x, z, hd + 1.57f + (rnd(&sd) - 0.5f) * 1.2f, 0.5f * L, r, 2.0f * r);
        }
        if (rnd(&sd) < 1.0f / 80.0f) {   /* a stump at the bed's edge */
            float side = rnd(&sd) < 0.5f ? -1.0f : 1.0f, lat = side * (BW - 0.3f + 1.2f * rnd(&sd)), x = cx - sinf(hd) * lat, z = cz + cosf(hd) * lat;
            int v = rnd(&sd) < 0.5f; float sc = 0.85f + 0.3f * rnd(&sd);
            if (bit_clear_of(R, s0, x, z, 1.5f)) bit_add(R, &cap, BIT_STUMP, v, x, z, rnd(&sd) * 6.28f, 0, 0.4f * sc, (v ? 0.45f : 0.6f) * sc);
        }
        if (rnd(&sd) < 1.0f / 65.0f) {   /* a boulder at the bed's edge */
            float side = rnd(&sd) < 0.5f ? -1.0f : 1.0f, lat = side * (BW + 0.2f + 0.8f * rnd(&sd)), x = cx - sinf(hd) * lat, z = cz + cosf(hd) * lat;
            int v = rnd(&sd) < 0.6f ? 0 : 1; float r = (v ? 1.0f : 0.7f) * (0.85f + 0.3f * rnd(&sd));
            if (bit_clear_of(R, s0, x, z, 2.0f)) bit_add(R, &cap, BIT_BOULDER, v, x, z, rnd(&sd) * 6.28f, 0, r, r * 0.75f);
        }
        (void)nm;
    }
    /* mossy fallen trees: two at each old log jam, and one every 380 m or so, right across or angled */
    float moss_s[32]; int nmoss = 0;
    for (int j = 0; j < R->ck_njam; j++) { moss_s[nmoss++] = R->ck_jam_s[j]; moss_s[nmoss++] = R->ck_jam_s[j] + 6.0f; }
    for (float s0 = R->ck_exit_s + 330.0f; s0 < R->ck_len - 200.0f && nmoss < 30; s0 += 380.0f) {
        float ss = s0 + 80.0f * (vnoise(s0 / 70.0f, 11.0f) - 0.5f); int near = 0;
        for (int k = 0; k < nmoss; k++) if (fabsf(moss_s[k] - ss) < 120.0f) near = 1;
        if (!near) moss_s[nmoss++] = ss;
    }
    R->ck_nmoss = 0;
    for (int k = 0; k < nmoss; k++) {
        float ss = moss_s[k], BW = route_creek_bedw(R, ss), hd = route_heading_at_creek(R, ss), cx, cz; route_creek_point(R, ss, &cx, &cz);
        if (!bit_clear_of(R, ss, cx, cz, 4.0f)) continue;
        int v = 2 + (k & 1); float sc = T.moss_scale * (k < 2 * R->ck_njam ? 1.3f : 1.0f), L = (v == 2 ? 14.0f : 11.0f) * sc;   /* the old jams' are bigger: Low Range */
        float ang = hd + 1.57f + (rnd(&sd) - 0.5f) * 0.8f, side = (k & 2) ? 1.0f : -1.0f, mid = side * (0.5f * L - BW - 1.5f);   /* across the bed, its butt up one bank */
        float mx = cx - sinf(hd) * mid, mz = cz + cosf(hd) * mid;
        /* the butt (x < 0 in the model) on the side the middle was pushed to */
        float bx = mx - cosf(ang) * 0.5f * L, bz = mz - sinf(ang) * 0.5f * L;
        RtNear bn = route_creek_near(R, bx, bz); if (bn.lat * side < 0) ang += (float)M_PI;
        bit_add(R, &cap, BIT_MOSSY, v, mx, mz, ang, 0.5f * L, 0.5f * sc, sc);   /* h holds the drawing scale */
        if (R->ck_nmoss < 32) R->ck_moss_s[R->ck_nmoss++] = ss;
    }
    /* the wet logs: in the water, about one every 280 m, and two more at each jam */
    for (float s0 = R->ck_exit_s + 180.0f; s0 < R->ck_len - 60.0f && R->ck_nwet < 40; s0 += 280.0f) {
        float ss = s0 + 60.0f * (vnoise(s0 / 50.0f, 17.0f) - 0.5f), BW = route_creek_bedw(R, ss), hd = route_heading_at_creek(R, ss), cx, cz; route_creek_point(R, ss, &cx, &cz);
        float lat = (rnd(&sd) - 0.5f) * 0.5f * BW, x = cx - sinf(hd) * lat, z = cz + cosf(hd) * lat;
        if (!bit_clear_of(R, ss, x, z, 3.0f)) continue;
        struct RtWet *w = &R->ck_wet[R->ck_nwet++]; w->x = x; w->z = z; w->yaw = hd + (rnd(&sd) - 0.5f) * 2.4f; w->var = R->ck_nwet & 1;
    }
    for (int j = 0; j < R->ck_njam && R->ck_nwet < 46; j++)
        for (int k = 0; k < 2; k++) {
            float ss = R->ck_jam_s[j] - 6.0f + 15.0f * (float)k, hd = route_heading_at_creek(R, ss), cx, cz; route_creek_point(R, ss, &cx, &cz);
            struct RtWet *w = &R->ck_wet[R->ck_nwet++]; w->x = cx; w->z = cz; w->yaw = hd + 1.2f * (float)(k * 2 - 1); w->var = k;
        }
    if (R->ck_fall_s > 0 && 0) {   /* the waterfall's lip: boulders across the bed (part 2: none; you drive down the chute) */
        float fs = R->ck_fall_s, hd = route_heading_at_creek(R, fs), side = (float)R->ck_fall_side, BW = route_creek_bedw(R, fs), cx, cz;
        route_creek_point(R, fs - 2.0f, &cx, &cz);
        for (float lat = -side * (BW + 6.0f); fabsf(lat + side * (BW + 6.0f)) < 2.0f * BW + 5.7f; lat += side * 1.9f) {
            float bx = cx - sinf(hd) * lat, bz = cz + cosf(hd) * lat, sc = 1.0f + 0.2f * vnoise(lat, 3.0f);
            bit_add(R, &cap, BIT_BOULDER, 1, bx, bz, lat, 0, 1.0f * sc * 0.8f, 0.75f * sc * 0.8f);
        }
    }
    /* the 4 m grid over them */
    float x0 = 1e9f, z0 = 1e9f, x1 = -1e9f, z1 = -1e9f;
    for (int i = 0; i < R->ck_nbit; i++) { float rch = bit_reach(&R->ck_bit[i]); x0 = fminf(x0, R->ck_bit[i].x - rch); z0 = fminf(z0, R->ck_bit[i].z - rch); x1 = fmaxf(x1, R->ck_bit[i].x + rch); z1 = fmaxf(z1, R->ck_bit[i].z + rch); }
    if (!R->ck_nbit) return;
    R->ckg_x0 = x0; R->ckg_z0 = z0; R->ckg_nx = (int)((x1 - x0) / 4.0f) + 1; R->ckg_nz = (int)((z1 - z0) / 4.0f) + 1;
    int nc = R->ckg_nx * R->ckg_nz, *cnt = calloc((size_t)nc, sizeof(int)), tot = 0;
    for (int pass = 0; pass < 2; pass++) {
        if (pass == 1) { R->ckg_first = calloc((size_t)nc + 1, sizeof(int)); for (int c = 0; c < nc; c++) R->ckg_first[c + 1] = R->ckg_first[c] + cnt[c]; R->ckg_list = malloc(sizeof(int) * (size_t)(tot > 0 ? tot : 1)); memset(cnt, 0, sizeof(int) * (size_t)nc); }
        for (int b = 0; b < R->ck_nbit; b++) {
            const struct RtBit *q = &R->ck_bit[b]; if (q->type == BIT_TWIGS) continue;
            float rch = bit_reach(q);
            int i0 = (int)((q->x - rch - x0) / 4.0f), i1 = (int)((q->x + rch - x0) / 4.0f), j0 = (int)((q->z - rch - z0) / 4.0f), j1 = (int)((q->z + rch - z0) / 4.0f);
            for (int j = j0; j <= j1; j++) for (int i = i0; i <= i1; i++) {
                if (i < 0 || j < 0 || i >= R->ckg_nx || j >= R->ckg_nz) continue;
                int c = j * R->ckg_nx + i;
                if (pass == 0) { cnt[c]++; tot++; } else R->ckg_list[R->ckg_first[c] + cnt[c]++] = b;
            }
        }
    }
    free(cnt);
    int nt[7] = { 0 }; for (int b = 0; b < R->ck_nbit; b++) nt[R->ck_bit[b].type]++;
    fprintf(stderr, "route: the creek's things: %d stones, %d stumps, %d branches, %d small logs, %d mossy fallen trees, %d boulders, %d twig litters; %d wet logs\n",
            nt[BIT_STONE], nt[BIT_STUMP], nt[BIT_BRANCH], nt[BIT_SMALLLOG], nt[BIT_MOSSY], nt[BIT_BOULDER], nt[BIT_TWIGS], R->ck_nwet);
}

static void creek_features_d(Route *R)
{
    float L = R->ck_len, ex = R->ck_exit_s;
    R->ck_nbyp = 0; R->ck_nrock = 0; R->ck_ndf = 0; R->ck_nmark = 0; R->camp_on = 0;
    /* part 2: no benches round the jams and the fallen trees (they're driven over now); the waterfall's step stays */
    if (R->ck_fall_s > 0) { int b = R->ck_nbyp++; R->ck_byp_s[b] = R->ck_fall_s; R->ck_byp_side[b] = R->ck_fall_side; R->ck_byp_h[b] = T.creek_bench_h; R->ck_byp_kind[b] = 2; }
    const float rock_at[3] = { 0.16f, 0.48f, 0.87f };
    for (int k = 0; k < 0; k++) {   /* part 2: no rock gardens (stones all the way down instead) */
        float bs = creek_flattest(R, L * rock_at[k] - 200.0f, L * rock_at[k] + 200.0f, 160.0f);
        if (bs < 0) continue;
        int g = R->ck_nrock++; R->ck_rock_s0[g] = bs - 20.0f; R->ck_rock_s1[g] = bs + 20.0f;
    }
    /* the rocks under each rock garden: humps on a jittered grid across the bed (the same numbers every build) */
    free(R->ck_hump); free(R->ck_hump_first);
    R->ck_hump = malloc(sizeof(float) * 4 * 600); R->ck_hump_first = calloc((size_t)R->ck_nrock + 1, sizeof(int)); R->ck_nhump = 0;
    for (int g = 0; g < R->ck_nrock; g++) {
        R->ck_hump_first[g] = R->ck_nhump;
        uint32_t sd = 5150u + (uint32_t)g * 977u;
        for (float s0 = R->ck_rock_s0[g] + 1.0f; s0 < R->ck_rock_s1[g] - 0.5f; s0 += 1.7f) {
            float BW = route_creek_bedw(R, s0), hd = route_heading_at_creek(R, s0), cx, cz; route_creek_point(R, s0, &cx, &cz);
            for (float lat = -BW - 0.6f; lat <= BW + 0.6f && R->ck_nhump < 600; lat += 1.8f) {
                if (rnd(&sd) > 0.72f) continue;
                float l2 = lat + (rnd(&sd) - 0.5f) * 0.9f, ds = (rnd(&sd) - 0.5f) * 0.8f;
                float *q = R->ck_hump + 4 * R->ck_nhump++;
                q[0] = cx - sinf(hd) * l2 + cosf(hd) * ds; q[1] = cz + cosf(hd) * l2 + sinf(hd) * ds;
                q[2] = 0.85f + 0.45f * rnd(&sd); q[3] = T.creek_rock_h * (0.65f + 0.35f * rnd(&sd));
            }
        }
    }
    R->ck_hump_first[R->ck_nrock] = R->ck_nhump;
    /* the camp: the flattest stretch before the ford, clear of the rifle's gravel bar (380 m up) */
    R->camp_s = creek_flattest(R, L - 330.0f, L - 170.0f, 60.0f);
    if (R->camp_s > 0) {
        float hd = route_heading_at_creek(R, R->camp_s), BW = route_creek_bedw(R, R->camp_s), cx, cz; route_creek_point(R, R->camp_s, &cx, &cz);
        R->camp_side = 1;
        float lat = (float)R->camp_side * (BW + 8.0f);
        R->camp_x = cx - sinf(hd) * lat; R->camp_z = cz + cosf(hd) * lat; R->camp_yaw = hd - (float)M_PI * 0.5f * (float)R->camp_side;   /* its open side (the model's +x) to the water */
        float l2 = (float)R->camp_side * (BW + 5.5f);
        R->trunk_x = cx - sinf(hd) * l2 + cosf(hd) * 4.0f; R->trunk_z = cz + cosf(hd) * l2 + sinf(hd) * 4.0f; R->trunk_yaw = hd;
        R->camp_on = 1;
    }
    /* part 2: the old fallen trees and log jams are now the creek's things (creek_bits_build) */
    /* the stock route's markers: ten, evenly from past the bend to the camp, nudged off the busy places */
    float m0 = ex + 300.0f, m1 = (R->camp_on ? R->camp_s : L) - 140.0f;
    static const int mv[10] = { 0, 1, 2, 3, 4, 1, 0, 3, 2, 4 };
    for (int k = 0; k < 10; k++) {
        float ms = m0 + (m1 - m0) * (float)k / 9.0f;
        for (int t = 0; t < 12 && (creek_busy(R, ms, 45.0f) || (R->camp_on && fabsf(ms - R->camp_s) < 60.0f)); t++) ms += (t & 1 ? -1.0f : 1.0f) * 15.0f * (float)(t + 1);
        for (int d = 0; d < R->ck_ndf; d++) if (fabsf(ms - R->ck_df_s[d]) < 18.0f) ms += 25.0f;
        float hd = route_heading_at_creek(R, ms), BW = route_creek_bedw(R, ms), cx, cz; route_creek_point(R, ms, &cx, &cz);
        float side = (k & 1) ? 1.0f : -1.0f, lat = side * fminf(BW + 2.6f, 6.3f);   /* just off the bed, never far from the way down */
        int m = R->ck_nmark++;
        R->ck_mark_s[m] = ms; R->ck_mark_x[m] = cx - sinf(hd) * lat; R->ck_mark_z[m] = cz + cosf(hd) * lat;
        R->ck_mark_yaw[m] = hd + (float)M_PI; R->ck_mark_v[m] = mv[k];   /* the blaze, the shoe or the sign facing upstream: seen as you come down */
    }
    fprintf(stderr, "route: the creek's climb-outs at");
    for (int b = 0; b < R->ck_nbyp; b++) fprintf(stderr, " %.0f (%s)", R->ck_byp_s[b], R->ck_byp_kind[b] == 0 ? "log jam" : R->ck_byp_kind[b] == 1 ? "fallen tree" : "waterfall");
    fprintf(stderr, "; rock gardens at");
    for (int g = 0; g < R->ck_nrock; g++) fprintf(stderr, " %.0f-%.0f", R->ck_rock_s0[g], R->ck_rock_s1[g]);
    fprintf(stderr, " (%d rocks); %d fallen trees; %d markers; the camp at %.0f m\n", R->ck_nhump, R->ck_ndf, R->ck_nmark, R->camp_s);
    creek_bits_build(R);
}

/* Where the creek leaves the clearing (the way in), where it falls (the steepest drop), and two log jams on its flats. */
static void creek_features(Route *R)
{
    float near_s = 0, nd = 1e9f;
    for (int i = 0; i < R->ck_n; i++) { float d = hypotf(R->ck_x[i] - R->hut_x, R->ck_z[i] - R->hut_z); if (d < nd) { nd = d; near_s = (float)i * RT_DS; } }
    R->ck_exit_s = near_s;
    for (float s0 = near_s; s0 < R->ck_len; s0 += 2.0f) { float x, z; route_creek_point(R, s0, &x, &z); if (hut_dist(R, x, z) > 0.5f) { R->ck_exit_s = s0; break; } }
    float best = 0; R->ck_fall_s = 0;
    for (float s0 = 2500.0f; s0 < R->ck_len - 800.0f; s0 += 2.0f) { float d = route_creek_bed(R, s0) - route_creek_bed(R, s0 + 10.0f); if (d > best) { best = d; R->ck_fall_s = s0 + 5.0f; } }
    R->ck_fall_side = 1;
    const float lo[2] = { 0.25f, 0.74f }, hi[2] = { 0.32f, 0.84f };
    R->ck_njam = 0;
    for (int j = 0; j < 2; j++) {
        float bs = 0, bg = 1e9f;
        for (float s0 = R->ck_len * lo[j]; s0 < R->ck_len * hi[j]; s0 += 4.0f) { float g = fabsf(route_creek_bed(R, s0 - 10.0f) - route_creek_bed(R, s0 + 10.0f)); if (g < bg && fabsf(s0 - R->ck_fall_s) > 200.0f) { bg = g; bs = s0; } }
        if (bs > 0) { R->ck_jam_s[R->ck_njam] = bs; R->ck_jam_side[R->ck_njam] = j ? -1 : 1; R->ck_njam++; }
    }
    fprintf(stderr, "route: the creek way leaves the clearing at %.0f m (nearest the hut at %.0f m, %.0f m off); falls %.1f m at %.0f m; log jams at %.0f and %.0f m\n",
            R->ck_exit_s, near_s, nd, best, R->ck_fall_s, R->ck_njam > 0 ? R->ck_jam_s[0] : 0.0f, R->ck_njam > 1 ? R->ck_jam_s[1] : 0.0f);
    creek_features_d(R);
}

static int read_creek(Route *R, const char *dir)
{
    char p[512]; snprintf(p, sizeof(p), "%s/creek.csv", dir);
    FILE *f = fopen(p, "r");
    if (!f) return 0;
    char line[256]; int cap = 0;
    if (!fgets(line, sizeof(line), f)) { fclose(f); return 0; }
    while (fgets(line, sizeof(line), f)) {
        float s0, x, y, bed, dm, hd, k;
        if (sscanf(line, "%f,%f,%f,%f,%f,%f,%f", &s0, &x, &y, &bed, &dm, &hd, &k) != 7) continue;
        if (R->ck_n == cap) {
            cap = cap ? cap * 2 : 4096;
            R->ck_x = realloc(R->ck_x, sizeof(float) * cap); R->ck_z = realloc(R->ck_z, sizeof(float) * cap);
            R->ck_bed = realloc(R->ck_bed, sizeof(float) * cap); R->ck_head = realloc(R->ck_head, sizeof(float) * cap);
        }
        R->ck_x[R->ck_n] = x; R->ck_z[R->ck_n] = y; R->ck_bed[R->ck_n] = bed; R->ck_head[R->ck_n] = hd; R->ck_n++;
    }
    fclose(f);
    if (R->ck_n < 10) { R->ck_n = 0; return 0; }
    R->ck_len = (float)(R->ck_n - 1) * RT_DS;
    /* buckets */
    float minx = 1e30f, maxx = -1e30f, minz = 1e30f, maxz = -1e30f;
    for (int i = 0; i < R->ck_n; i++) { minx = fminf(minx, R->ck_x[i]); maxx = fmaxf(maxx, R->ck_x[i]); minz = fminf(minz, R->ck_z[i]); maxz = fmaxf(maxz, R->ck_z[i]); }
    R->ckb_cell = 20.0f; R->ckb_x0 = minx - 1; R->ckb_z0 = minz - 1;
    R->ckb_nx = (int)((maxx - R->ckb_x0) / R->ckb_cell) + 2; R->ckb_nz = (int)((maxz - R->ckb_z0) / R->ckb_cell) + 2;
    int nb = R->ckb_nx * R->ckb_nz;
    R->ckb_first = calloc((size_t)nb + 1, sizeof(int)); R->ckb_list = malloc(sizeof(int) * (size_t)R->ck_n);
    int *cnt = calloc((size_t)nb, sizeof(int));
    for (int i = 0; i < R->ck_n; i++) cnt[(int)((R->ck_z[i] - R->ckb_z0) / R->ckb_cell) * R->ckb_nx + (int)((R->ck_x[i] - R->ckb_x0) / R->ckb_cell)]++;
    for (int b = 0; b < nb; b++) R->ckb_first[b + 1] = R->ckb_first[b] + cnt[b];
    memset(cnt, 0, sizeof(int) * (size_t)nb);
    for (int i = 0; i < R->ck_n; i++) { int b = (int)((R->ck_z[i] - R->ckb_z0) / R->ckb_cell) * R->ckb_nx + (int)((R->ck_x[i] - R->ckb_x0) / R->ckb_cell); R->ckb_list[R->ckb_first[b] + cnt[b]++] = i; }
    free(cnt);
    R->ckb_near = calloc((size_t)nb, 1);   /* buckets within about 80 m of the line: anywhere else is "not near the creek", at once */
    for (int i = 0; i < R->ck_n; i += 4) {
        int bx = (int)((R->ck_x[i] - R->ckb_x0) / R->ckb_cell), bz = (int)((R->ck_z[i] - R->ckb_z0) / R->ckb_cell);
        for (int j = bz - 5; j <= bz + 5; j++) for (int q = bx - 5; q <= bx + 5; q++) if (q >= 0 && j >= 0 && q < R->ckb_nx && j < R->ckb_nz) R->ckb_near[j * R->ckb_nx + q] = 1;
    }
    R->ck_nch = (R->ck_n - 1 + RT_CHUNK - 1) / RT_CHUNK;
    return 1;
}

RtNear route_creek_near(const Route *R, float x, float z)
{
    RtNear r = { 0, 0, 1e9f, 0 };
    if (R->ck_n < 2) return r;
    int cx = (int)floorf((x - R->ckb_x0) / R->ckb_cell), cz = (int)floorf((z - R->ckb_z0) / R->ckb_cell);
    if (cx < 0 || cz < 0 || cx >= R->ckb_nx || cz >= R->ckb_nz || !R->ckb_near[cz * R->ckb_nx + cx]) return r;
    float best = 1e30f; int bi = -1;
    for (int k = 0; k < 6; k++) {   /* rings out to 120 m; further than that is "not near the creek" */
        for (int j = cz - k; j <= cz + k; j++) {
            if (j < 0 || j >= R->ckb_nz) continue;
            int step = (j == cz - k || j == cz + k) ? 1 : 2 * k;
            for (int i = cx - k; i <= cx + k; i += step > 0 ? step : 1) {
                if (i >= 0 && i < R->ckb_nx) {
                    int b = j * R->ckb_nx + i;
                    for (int q = R->ckb_first[b]; q < R->ckb_first[b + 1]; q++) {
                        int s2 = R->ckb_list[q]; float dx = R->ck_x[s2] - x, dz = R->ck_z[s2] - z, d = dx * dx + dz * dz;
                        if (d < best) { best = d; bi = s2; }
                    }
                }
                if (step == 0) break;
            }
        }
        float ring = (float)k * R->ckb_cell;
        if (bi >= 0 && ring * ring > best) break;
    }
    if (bi < 0) return r;
    float bestd = 1e30f;
    for (int e = -1; e <= 0; e++) {
        int a = bi + e, b = a + 1;
        if (a < 0 || b >= R->ck_n) continue;
        float tx = R->ck_x[b] - R->ck_x[a], tz = R->ck_z[b] - R->ck_z[a], L2 = tx * tx + tz * tz;
        float t = L2 > 0 ? ((x - R->ck_x[a]) * tx + (z - R->ck_z[a]) * tz) / L2 : 0;
        t = clampf(t, 0, 1);
        float fx = R->ck_x[a] + tx * t, fz = R->ck_z[a] + tz * t, dx = x - fx, dz = z - fz, d = dx * dx + dz * dz;
        if (d < bestd) { float L = sqrtf(L2); bestd = d; r.s = ((float)a + t) * RT_DS; r.lat = L > 0 ? (tx * (z - R->ck_z[a]) - tz * (x - R->ck_x[a])) / L : 0; r.dist = sqrtf(d); r.i = t < 0.5f ? a : b; }
    }
    return r;
}
void route_creek_point(const Route *R, float s, float *x, float *z)
{
    float u = clampf(s / RT_DS, 0, (float)(R->ck_n - 1)); int i = (int)u; if (i >= R->ck_n - 1) i = R->ck_n - 2;
    float t = u - (float)i;
    *x = R->ck_x[i] + (R->ck_x[i + 1] - R->ck_x[i]) * t; *z = R->ck_z[i] + (R->ck_z[i + 1] - R->ck_z[i]) * t;
}
float route_creek_bed(const Route *R, float s)
{
    float u = clampf(s / RT_DS, 0, (float)(R->ck_n - 1)); int i = (int)u; if (i >= R->ck_n - 1) i = R->ck_n - 2;
    return R->ck_bed[i] + (R->ck_bed[i + 1] - R->ck_bed[i]) * (u - (float)i);
}
/* The bed's half width: 3 to 6 m, wider on the flats */
float route_creek_bedw(const Route *R, float s)
{
    float w0 = 3.0f + 3.0f * vnoise(s / 140.0f, 4.0f);
    if (!R || R->ck_exit_s <= 0) return w0;
    /* build D: narrower past the way in (2.6 to 5 m), blended in over 40 m */
    float w1 = 2.4f + 2.0f * vnoise(s / 140.0f, 4.0f);   /* part 2: a little narrower again */
    return w0 + (w1 - w0) * smooth01((s - R->ck_exit_s - 40.0f) / 40.0f);
}
/* The strip's half width (out to where its bank starts): about 20 m, but through the forest's edge where the creek leaves
 * the clearing, a gap the ute just fits (the bank starts 3 m inside the line, so it's held 3.5 m out either side) */
float route_creek_half(const Route *R, float s)
{
    float g1 = R->ck_exit_s + 30.0f, wide = 7.0f + 3.0f * vnoise(s / 90.0f, 9.0f);   /* build D: narrow (Build C: 16 to 23 m); part 2: 7 to 10 m */
    wide += 8.0f * smooth01((vnoise(s / 260.0f, 21.0f) - 0.62f) / 0.1f);   /* now and then a wider flat to breathe */
    if (R->ck_fall_s > 0 && s > R->ck_fall_s - 80.0f && s < R->ck_fall_s + 50.0f) wide = fmaxf(wide, route_creek_bedw(R, s) + 14.0f);   /* room for the way round the waterfall */
    if (R->camp_on && fabsf(s - R->camp_s) < 30.0f) wide = fmaxf(wide, (route_creek_bedw(R, s) + 16.0f) * (1.0f - smooth01((fabsf(s - R->camp_s) - 18.0f) / 12.0f)));   /* the camp's flat */
    if (s < g1) return -3.0f;
    return -3.0f + (wide + 3.0f) * smooth01((s - g1) / 45.0f);
}
/* How far into the creek way's bank a point is, by its distance from the line. Build D: through the gap the floor is flat for
 * 1.2 m either side of the line and the bank rises steeply past that (held at 3.4 m: the ute only just fits); where the strip
 * opens out it's the old rule, distance less the strip's half width. */
static float creek_chan_e(const Route *R, float s, float d)
{
    float half = route_creek_half(R, s), k = 1.0f + 1.95f * (1.0f - smooth01((half - 1.2f) / 3.0f));
    return k * (d - fmaxf(half, 1.2f));
}
static float creek_edge(const Route *R, float x, float z)
{
    if (!R->explore || R->ck_n < 2) return 1e9f;
    RtNear c = route_creek_near(R, x, z);
    if (c.dist > 200.0f || c.s < R->ck_exit_s - 30.0f) return 1e9f;   /* upstream of the way in: closed */
    return creek_chan_e(R, c.s, c.dist);
}
float route_creek_line(const Route *R, float s, int *slow)
{
    float lat = 0; int sl = 0;
    for (int b = 0; b < R->ck_nbyp; b++) {
        float js = R->ck_byp_s[b], BW = route_creek_bedw(R, s);
        if (R->ck_byp_kind[b] == 2 && T.fall_ledge != 0) { if (s > js - 66.0f && s < js + 36.0f) { float k = smooth01((s - (js - 66.0f)) / 18.0f) * (1.0f - smooth01((s - (js + 22.0f)) / 14.0f)); lat = (float)R->ck_byp_side[b] * (BW + 5.5f) * k; sl = 1; } }   /* part 2: eased onto the run-in, then straight up the step */
        else if (s > js - 30.0f && s < js + 27.0f) { lat = (float)R->ck_byp_side[b] * (BW + 3.2f); sl = 1; }
    }
    for (int k = 0; k < R->ck_nmoss; k++) if (fabsf(s - R->ck_moss_s[k]) < 14.0f) sl = 1;   /* part 2: crawl over the mossy fallen trees */
    for (int d = 0; d < R->ck_ndf; d++)   /* round a fallen tree part way across: on the open side */
        if (R->ck_df_kind[d] == 0 && fabsf(s - R->ck_df_s[d]) < 14.0f) { lat = -(float)R->ck_df_side[d] * (0.3f * route_creek_bedw(R, s) + 2.0f); sl = 1; }
    if (route_creek_rocky(R, s)) sl = 1;
    if (slow) *slow = sl;
    return lat;
}
int route_creek_rocky(const Route *R, float s)
{
    for (int g = 0; g < R->ck_nrock; g++) if (s > R->ck_rock_s0[g] - 6.0f && s < R->ck_rock_s1[g] + 4.0f) return 1;
    return 0;
}
int route_in_creek(const Route *R, float x, float z)
{
    if (R->ck_n < 2) return 0;
    RtNear c = route_creek_near(R, x, z);
    return c.dist < 200.0f && c.s >= R->ck_exit_s - 30.0f && creek_chan_e(R, c.s, c.dist) < T.edge_stop;
}

/* Build D: the thicket where the creek leaves the clearing (Explorer). From a little before the way in and for 220 m past
 * it, everything beside the creek's own channel is closed, whatever the clearing's hollow says: the bank and the scrub wrap
 * both sides, so the only way into the creek way is the gap itself. It rises over 16 m where it starts, so it's a bank, not a
 * cliff. Below 0 everywhere else. */
static float creek_wall(const Route *R, float x, float z)
{
    if (!R->explore || R->ck_n < 2) return -1e9f;
    RtNear c = route_creek_near(R, x, z);
    float a = R->ck_exit_s - T.creek_wall_from;
    if (c.dist > 110.0f || c.s < a - 16.0f || c.s > R->ck_exit_s + 220.0f) return -1e9f;
    float e = fminf(creek_chan_e(R, c.s, c.dist), 20.0f);   /* build E: capped where the bank's height tops out ... */
    e -= 45.0f * smooth01((c.dist - 80.0f) / 30.0f);   /* ... and eased down to open ground by 110 m out (it ended there in a 4 m cliff) */
    return e - 30.0f * (1.0f - smooth01((c.s - (a - 16.0f)) / 16.0f));
}
float route_edge(const Route *R, RtNear n, float x, float z)
{
    return fmaxf(fminf(fminf(road_dist(R, n) - T.edge_dist, hut_dist(R, x, z) - T.edge_hut_gap), creek_edge(R, x, z)), creek_wall(R, x, z));
}
float route_edge_at(const Route *R, float x, float z) { return route_edge(R, route_near(R, x, z), x, z); }
int route_in_clearing(const Route *R, float x, float z)
{
    return hut_dist(R, x, z) < 2.5f || (R->pad_r > 0 && hypotf(x - R->pad_x, z - R->pad_z) < R->pad_r + 6.0f);
}

/* ------------------------------------------------- the junction (follow-up to build B) */
/* Cattlemans Link Trail (OSM way 395705240) meets the route where Robinsons Track ends and Cattlemens Track turns
 * south for the bridge. Its first metres, from OSM, metres east/north of the OSM junction node (-36.722195, 149.4565126);
 * the game's road was smoothed through the corner and passes 23 m inside it, so the stub is hung on the nearest point. */
#define BOLLARD_R 0.16f   /* a bollard's solid core */
#define SPUR_LEN 42.0f
#define JUNCTION_R 7.5f
static const float SPUR_OSM[][2] = { { 0, 0 }, { -34.7f, 16.5f }, { -53.1f, 30.0f }, { -56.9f, 41.6f }, { -57.2f, 56.8f } };
#define OSM_JX (-546.4f)
#define OSM_JZ (-150.0f)
static void build_spur(Route *R)
{
    R->spur_n = 0; R->gate_t = 0;
    if (R->length < 1000.0f) return;
    RtNear jn = route_near(R, OSM_JX, OSM_JZ);
    if (jn.dist > 40.0f || jn.s < 100.0f) return;   /* not this route */
    R->junction_s = jn.s; route_point(R, jn.s, &R->jx, &R->jz);
    /* a Catmull-Rom curve through the OSM points, every 2 m */
    float px[8], pz[8]; int np = (int)(sizeof(SPUR_OSM) / sizeof(SPUR_OSM[0]));
    for (int k = 0; k < np; k++) { px[k] = R->jx + SPUR_OSM[k][0]; pz[k] = R->jz + SPUR_OSM[k][1]; }
    float t = 0, lx = px[0], lz = pz[0];
    R->spur_x[0] = lx; R->spur_z[0] = lz; R->spur_n = 1;
    for (int k = 0; k + 1 < np && R->spur_n < 64; k++)
        for (int q = 1; q <= 40 && R->spur_n < 64; q++) {
            float u = (float)q / 40.0f, u2 = u * u, u3 = u2 * u;
            int a = k > 0 ? k - 1 : 0, d = k + 2 < np ? k + 2 : np - 1;
            float x = 0.5f * ((2 * px[k]) + (-px[a] + px[k + 1]) * u + (2 * px[a] - 5 * px[k] + 4 * px[k + 1] - px[d]) * u2 + (-px[a] + 3 * px[k] - 3 * px[k + 1] + px[d]) * u3);
            float z = 0.5f * ((2 * pz[k]) + (-pz[a] + pz[k + 1]) * u + (2 * pz[a] - 5 * pz[k] + 4 * pz[k + 1] - pz[d]) * u2 + (-pz[a] + 3 * pz[k] - 3 * pz[k + 1] + pz[d]) * u3);
            float step = hypotf(x - lx, z - lz);
            if (step < 2.0f) continue;
            t += step; lx = x; lz = z;
            R->spur_x[R->spur_n] = x; R->spur_z[R->spur_n] = z; R->spur_n++;
            if (t > SPUR_LEN + 4.0f) { k = np; break; }
        }
    /* heights: the DEM along it, smoothed, easing out of the road's own height over the first 20 m */
    float raw[64];
    for (int i = 0; i < R->spur_n; i++) raw[i] = route_dem(R, R->spur_x[i], R->spur_z[i]);
    float h0 = route_road_h(R, R->junction_s);
    for (int i = 0; i < R->spur_n; i++) {
        float sum = 0; int n = 0;
        for (int j = i - 3; j <= i + 3; j++) if (j >= 0 && j < R->spur_n) { sum += raw[j]; n++; }
        float ti = 2.0f * (float)i;
        R->spur_h[i] = h0 + (sum / (float)n - h0) * smooth01(ti / 20.0f);
    }
    /* the gate across it, about 20 m up from the edge of the road */
    R->gate_t = 24.0f;
    int g = (int)(R->gate_t / 2.0f);
    R->gate_x = R->spur_x[g]; R->gate_z = R->spur_z[g];
    R->gate_yaw = atan2f(R->spur_z[g + 1] - R->spur_z[g - 1], R->spur_x[g + 1] - R->spur_x[g - 1]);   /* along the side road */
}

float route_spur_dist(const Route *R, float x, float z, float *t)
{
    if (t) *t = 1e9f;
    if (R->spur_n < 2 || fabsf(x - R->jx) > 90.0f || fabsf(z - R->jz) > 90.0f) return 1e9f;
    float best = 1e9f, bt = 1e9f;
    for (int i = 0; i + 1 < R->spur_n; i++) {
        float ax = R->spur_x[i], az = R->spur_z[i], dx = R->spur_x[i + 1] - ax, dz = R->spur_z[i + 1] - az, L2 = dx * dx + dz * dz;
        float u = L2 > 0 ? clampf(((x - ax) * dx + (z - az) * dz) / L2, 0, 1) : 0;
        float d = hypotf(x - ax - dx * u, z - az - dz * u);
        if (d < best) { best = d; bt = 2.0f * ((float)i + u); }
    }
    if (t) *t = bt;
    return best;
}
static float spur_h_at(const Route *R, float t)
{
    float u = clampf(t / 2.0f, 0, (float)(R->spur_n - 1)); int i = (int)u; if (i >= R->spur_n - 1) i = R->spur_n - 2;
    return R->spur_h[i] + (R->spur_h[i + 1] - R->spur_h[i]) * (u - (float)i);
}
int route_on_spur(const Route *R, float x, float z)
{
    float t, d = route_spur_dist(R, x, z, &t);
    return d < route_half_width() + T.verge_w && t < SPUR_LEN;
}

/* The cross-section: road (a slight crown), a grass verge, the ditch, then a bank blending into the DEM. */
static float profile(const Route *R, RtNear n, float x, float z, int *surf)
{
    float W = route_half_width(), V = T.verge_w, D = fmaxf(T.ditch_w, 0.2f), B = fmaxf(T.bank_w, 1.0f);
    float e1 = W + V, e2 = e1 + D, a = fabsf(n.lat);
    float hr = route_road_h(R, n.s), h;
    int sf;
    if (in_bridge(R, n.s)) {
        if (a <= W + 0.2f) { h = hr; sf = SURF_DECK; }
        else { h = route_dem(R, x, z); sf = SURF_ROUGH; }
    } else if (a <= W) { float q = a / W; h = hr + 0.06f * (1.0f - q * q); sf = SURF_ROAD; }
    else if (a <= e1) { h = hr - 0.04f; sf = SURF_VERGE; }
    else if (a <= e2) { h = hr - 0.04f - T.ditch_depth * sinf((float)M_PI * (a - e1) / D); sf = SURF_DITCH; }
    else { h = hr - 0.04f + (route_dem(R, x, z) - (hr - 0.04f)) * smooth01((a - e2) / B); sf = SURF_ROUGH; }
    /* past the ends of the line: rough ground */
    if (n.s < -1.0f || n.s > R->length + 1.0f) { if (sf != SURF_ROUGH) sf = SURF_ROUGH; }
    /* the junction: the side road's own cross-section where it is the higher-ranking surface */
    if (R->spur_n > 1 && sf != SURF_ROAD && sf != SURF_DECK) {
        float t2, d2 = route_spur_dist(R, x, z, &t2);
        float da = hypotf(x - R->spur_x[2], z - R->spur_z[2]);   /* the junction's gravel splay, round the side road's mouth */
        if (da < JUNCTION_R + 1.0f) { float ha = spur_h_at(R, 4.0f); if (da < JUNCTION_R) { h = ha + 0.03f; sf = SURF_ROAD; } else if (sf != SURF_VERGE) { h = ha - 0.04f; sf = SURF_VERGE; } d2 = 1e9f; }
        if (d2 < e2 + B && t2 < SPUR_LEN) {
            float hs = spur_h_at(R, t2), fade = 1.0f - smooth01((t2 - (SPUR_LEN - 6.0f)) / 6.0f);   /* it fades out into the bush at its end */
            if (d2 <= W && fade > 0.5f) { float q = d2 / W; h = hs + 0.06f * (1.0f - q * q); sf = SURF_ROAD; }
            else if (d2 <= e1 && fade > 0.5f) { h = hs - 0.04f; sf = SURF_VERGE; }
            else if (d2 <= e2 && sf == SURF_ROUGH) { h = hs - 0.04f - T.ditch_depth * fade * sinf((float)M_PI * (d2 - e1) / D); sf = SURF_DITCH; }
            else if (sf == SURF_ROUGH) { float hb = hs - 0.04f; h = hb + (h - hb) * fmaxf(smooth01((d2 - e2) / B), 1.0f - fade); }
        }
    }
    /* build C: the creek way's bed, cut into the ground along its line (never near the road: the road's own ford takes it) */
    if (R->ck_n > 1 && sf == SURF_ROUGH) {
        RtNear cn = route_creek_near(R, x, z);
        float BW = route_creek_bedw(R, cn.s);
        if (cn.dist < BW + 12.0f) {
            float wr = smooth01((road_dist(R, n) - 30.0f) / 18.0f);
            if (wr > 0) {
                float a2 = cn.dist, bed = route_creek_bed(R, cn.s);
                float hb = bed + (a2 < BW ? 0.04f * (a2 / BW) : 0.04f + 0.22f * (a2 - BW));
                float hc = hb + (h - hb) * smooth01((a2 - BW) / 11.0f);
                if (hc < h) h += (hc - h) * wr;
                if (wr > 0.5f) { if (a2 < 0.3f * BW + 0.5f * vnoise(cn.s / 20.0f, 2.0f)) sf = SURF_WATER; else if (a2 < BW) sf = SURF_BED; }
                /* the way round the waterfall: a ledge cut into the bank on one side, an even ramp down past the drop. Build D: it
                 * starts with a rocky step up of creek_bench_h (Low Range), then eases down to below the fall */
                if (T.fall_ledge != 0 && R->ck_fall_s > 0 && cn.s > R->ck_fall_s - 66.0f && cn.s < R->ck_fall_s + 40.0f && cn.lat * (float)R->ck_fall_side > 0 && a2 > BW + 0.6f) {
                    float L0 = R->ck_fall_s - 32.0f, L1 = R->ck_fall_s + 32.0f, hl = route_creek_bed(R, L0) + (route_creek_bed(R, L1) - route_creek_bed(R, L0)) * smooth01((cn.s - L0) / (L1 - L0)) + 0.15f;
                    float up = smooth01((cn.s - (L0 + 1.0f)) / T.creek_ramp) * (1.0f - smooth01((cn.s - (R->ck_fall_s + 4.0f)) / 22.0f));
                    hl += T.fall_step_h * up;   /* part 2: 0 by default (with the sprung body the step and its run-in tripped the ute; the ledge alone is the way round) */
                    hl = fmaxf(hl, route_creek_bed(R, cn.s) + 0.15f);   /* part 2: never a pit below the bed where it rejoins */
                    float ds_ = cn.s - R->ck_fall_s, ws = ds_ < 0 ? 1.0f - smooth01((-ds_ - 56.0f) / 10.0f) : 1.0f - smooth01((ds_ - 28.0f) / 10.0f), wl = 1.0f - smooth01((a2 - (BW + 9.0f)) / 5.0f);   /* part 2: wider, and a flat run-in beside the bed before the step */
                    wl *= smooth01((a2 - BW - 0.6f) / 3.5f);   /* part 2: its inner side a slope down to the bed, not a sheer drop (the ute tipped off it) */
                    h += (hl - h) * ws * wl;
                    if (ws * wl > 0.5f && sf != SURF_WATER) sf = up > 0.03f && up < 0.97f && cn.s < R->ck_fall_s ? SURF_ROCK : SURF_BED;
                }
                /* round each log jam and each fallen tree right across: a bench on its open side. Build D: raised creek_bench_h
                 * over the bed, with steep rocky ramps at its ends (Low Range) */
                for (int jm = 0; jm < R->ck_nbyp; jm++) {
                    if (R->ck_byp_kind[jm] == 2) continue;
                    float js = R->ck_byp_s[jm], ds = fabsf(cn.s - js);
                    if (ds > 32.0f || cn.lat * (float)R->ck_byp_side[jm] <= 0 || a2 < BW) continue;
                    float rise = 1.0f - smooth01((ds - 20.0f) / T.creek_ramp);
                    float hl = route_creek_bed(R, cn.s) + 0.2f + R->ck_byp_h[jm] * rise, ws = 1.0f - smooth01((ds - 20.0f - T.creek_ramp) / 5.0f), wl = 1.0f - smooth01((a2 - (BW + 6.5f)) / 5.0f);
                    h += (hl - h) * ws * wl;
                    if (ws * wl > 0.5f && sf != SURF_WATER) sf = rise > 0.03f && rise < 0.97f ? SURF_ROCK : SURF_BED;
                }
                if (R->explore && R->ck_nbit) {   /* build D, part 2: the creek's things to drive over */
                    float bump = route_creek_bump(R, x, z);
                    if (bump > 0.0f) { h += bump; if (bump > 0.06f && sf != SURF_WATER) sf = SURF_ROCK; }
                }
                /* build D: the rock gardens: rocks humped across the bed */
                for (int g = 0; g < R->ck_nrock; g++) {
                    if (cn.s < R->ck_rock_s0[g] - 3.0f || cn.s > R->ck_rock_s1[g] + 3.0f || a2 > BW + 2.5f) continue;
                    float bump = 0;
                    for (int q = R->ck_hump_first[g]; q < R->ck_hump_first[g + 1]; q++) {
                        const float *hq = R->ck_hump + 4 * q; float dx = x - hq[0], dz = z - hq[1], d2 = dx * dx + dz * dz, r2 = hq[2] * hq[2];
                        if (d2 < r2) { float u = 1.0f - d2 / r2; bump = fmaxf(bump, hq[3] * u * u); }
                    }
                    h += bump;
                    if (cn.s > R->ck_rock_s0[g] && cn.s < R->ck_rock_s1[g] && a2 < BW + 1.2f) sf = SURF_ROCK;
                }
            }
        }
    }
    /* build B: the map's edge, a cut bank rising out of the bush (uneven along its length so it reads as ground) */
    if (sf == SURF_ROUGH || sf == SURF_BED || sf == SURF_WATER || sf == SURF_ROCK) {
        float e = route_edge(R, n, x, z);
        if (e > -2.0f) {
            float wob = 0.8f + 0.4f * vnoise(x / 23.0f, z / 23.0f);
            h += T.edge_h * wob * smooth01(e / fmaxf(T.edge_w, 1.0f)) + 0.35f * T.edge_h * smooth01((e - T.edge_w) / 10.0f);
        }
    }
    /* the start pad: a flat clearing, all of it drivable */
    if (R->pad_r > 0) {
        float dx = x - R->pad_x, dz = z - R->pad_z, dp = sqrtf(dx * dx + dz * dz);
        if (dp < R->pad_r + 6.0f) {
            float w = 1.0f - smooth01((dp - R->pad_r) / 6.0f);
            h += (R->pad_h - h) * w;
            if (dp < R->pad_r) sf = SURF_ROAD;
        }
    }
    if (surf) *surf = sf;
    return h;
}

float route_height(const Route *R, float x, float z, int *surf)
{
    return profile(R, route_near(R, x, z), x, z, surf);
}

float route_min_radius_ahead(const Route *R, float s, float ahead)
{
    int i0 = (int)clampf(s / RT_DS, 0, (float)(R->n - 1)), i1 = (int)clampf((s + ahead) / RT_DS, 0, (float)(R->n - 1));
    float k = 0;
    for (int i = i0; i <= i1; i++) k = fmaxf(k, fabsf(R->curv[i]));
    return k > 1e-5f ? 1.0f / k : 1e5f;
}

/* The fastest the road allows here (ours, session 2, for the road bot): every bend in the next `ahead`
 * metres taken at `alat` sideways, reached by braking at `abrake` from here. */
float route_speed_ahead(const Route *R, float s, float ahead, float alat, float abrake)
{
    int i0 = (int)clampf(s / RT_DS, 0, (float)(R->n - 1)), i1 = (int)clampf((s + ahead) / RT_DS, 0, (float)(R->n - 1));
    float best = 1e5f;
    for (int i = i0; i <= i1; i++) {
        float k = fabsf(R->curv[i]);
        if (k < 1e-5f) continue;
        float v2 = alat / k + 2.0f * abrake * fmaxf((float)i * RT_DS - s, 0);
        if (v2 < best) best = v2;
    }
    return sqrtf(best);
}

/* ------------------------------------------------------------ randomness */

static uint32_t hash32(uint32_t x) { x ^= x >> 16; x *= 0x7feb352dU; x ^= x >> 15; x *= 0x846ca68bU; x ^= x >> 16; return x; }
static float rnd(uint32_t *s) { *s = hash32(*s + 0x9e3779b9U); return (float)(*s >> 8) / 16777216.0f; }
static float vnoise(float x, float z)
{
    int ix = (int)floorf(x), iz = (int)floorf(z);
    float fx = x - (float)ix, fz = z - (float)iz;
    fx = fx * fx * (3 - 2 * fx); fz = fz * fz * (3 - 2 * fz);
#define HV(a, b) ((float)(hash32((uint32_t)(a) * 73856093U ^ (uint32_t)(b) * 19349663U) >> 8) / 16777216.0f)
    float v00 = HV(ix, iz), v10 = HV(ix + 1, iz), v01 = HV(ix, iz + 1), v11 = HV(ix + 1, iz + 1);
#undef HV
    return (v00 * (1 - fx) + v10 * fx) * (1 - fz) + (v01 * (1 - fx) + v11 * fx) * fz;
}

/* ------------------------------------------------- terrain cells and trees */

static float canopy_at(const Route *R, float x, float z)
{
    if (!R->canopy) return 0.75f;
    int i = (int)clampf((x - R->gx0) / R->cell + 0.5f, 0, (float)(R->gx - 1));
    int j = (int)clampf((z - R->gz0) / R->cell + 0.5f, 0, (float)(R->gy - 1));
    return (float)R->canopy[(size_t)j * R->gx + i] / 255.0f;
}

#define TERRAIN_REACH 64.0f   /* terrain is built this far from the centreline */

static void assign_cells(Route *R)
{
    int ncx = R->gx - 1, ncz = R->gy - 1;
    R->owner = malloc(sizeof(short) * (size_t)ncx * ncz);
    unsigned char *near = calloc((size_t)ncx * ncz, 1);
    int reach = (int)(TERRAIN_REACH / R->cell) + 2;
    for (int s = 0; s < R->n; s += 2) { /* cells that could be within reach of the line */
        int ci = (int)((R->x[s] - R->gx0) / R->cell), cj = (int)((R->z[s] - R->gz0) / R->cell);
        for (int j = cj - reach; j <= cj + reach; j++)
            for (int i = ci - reach; i <= ci + reach; i++)
                if (i >= 0 && j >= 0 && i < ncx && j < ncz) near[(size_t)j * ncx + i] = 1;
    }
    int *cnt = calloc(RT_MAX_CHUNKS, sizeof(int));
    for (int j = 0; j < ncz; j++)
        for (int i = 0; i < ncx; i++) {
            float x = R->gx0 + ((float)i + 0.5f) * R->cell, z = R->gz0 + ((float)j + 0.5f) * R->cell;
            float dp = hypotf(x - R->pad_x, z - R->pad_z);
            short o = -1;
            int pad = R->pad_r > 0 && dp < R->pad_r + 40.0f;
            int hollow = hut_dist(R, x, z) < T.edge_hut_gap + T.edge_w + 7.0f;   /* build B: the hut's whole frost hollow and its edge */
            if (near[(size_t)j * ncx + i] || pad || hollow) {
                RtNear n = route_near(R, x, z);
                if ((n.dist < TERRAIN_REACH && route_edge(R, n, x, z) < T.edge_w + 7.0f) || pad || hollow)
                    o = (short)clampf(floorf(n.s / (RT_DS * RT_CHUNK)), 0, (float)(R->nchunks - 1));
            }
            R->owner[(size_t)j * ncx + i] = o;
            if (o >= 0) cnt[o]++;
            if (o >= 0) { float cx, cz, r; route_chunk_centre(R, o, &cx, &cz, &r); r = hypotf(x - cx, z - cz) + R->cell; if (r > R->chunk_r[o]) R->chunk_r[o] = r; }
        }
    free(near);
    if (R->ck_n > 1) {   /* build C: the creek way's strip, past the road's and the clearing's own cells: its own chunks */
        int reach2 = (int)(48.0f / R->cell) + 1;
        unsigned char *cand = calloc((size_t)ncx * ncz, 1);
        for (int i = 0; i < R->ck_n; i += 2) {
            if ((float)i * RT_DS < R->ck_exit_s - 60.0f) continue;
            int ci = (int)((R->ck_x[i] - R->gx0) / R->cell), cj = (int)((R->ck_z[i] - R->gz0) / R->cell);
            for (int j = cj - reach2; j <= cj + reach2; j++) for (int q = ci - reach2; q <= ci + reach2; q++) if (q >= 0 && j >= 0 && q < ncx && j < ncz) cand[(size_t)j * ncx + q] = 1;
        }
        for (int k = 0; k < ncx * ncz; k++) {
            if (!cand[k] || R->owner[k] >= 0) continue;
            float x = R->gx0 + ((float)(k % ncx) + 0.5f) * R->cell, z = R->gz0 + ((float)(k / ncx) + 0.5f) * R->cell;
            RtNear cn = route_creek_near(R, x, z);
            if (cn.dist > route_creek_half(R, cn.s) + T.edge_w + 7.0f || cn.s < R->ck_exit_s - 60.0f) continue;
            int o = R->nchunks + (int)clampf(floorf(cn.s / (RT_DS * RT_CHUNK)), 0, (float)(R->ck_nch - 1));
            R->owner[k] = (short)o; cnt[o]++;
            float cx, cz, r; route_chunk_centre(R, o, &cx, &cz, &r); r = hypotf(x - cx, z - cz) + R->cell; if (r > R->chunk_r[o]) R->chunk_r[o] = r;
        }
        free(cand);
    }
    R->cell_first = calloc(RT_MAX_CHUNKS + 1, sizeof(int));
    for (int c = 0; c < RT_MAX_CHUNKS; c++) R->cell_first[c + 1] = R->cell_first[c] + cnt[c];
    R->cell_list = malloc(sizeof(int) * (size_t)(R->cell_first[RT_MAX_CHUNKS] + 1));
    memset(cnt, 0, sizeof(int) * RT_MAX_CHUNKS);
    for (int k = 0; k < ncx * ncz; k++) { int o = R->owner[k]; if (o >= 0) R->cell_list[R->cell_first[o] + cnt[o]++] = k; }
    free(cnt);
}

void route_add_props(Route *R);
static void place_creek(Route *R);
static float route_heading_at_creek(const Route *R, float s) { int i = (int)clampf(s / RT_DS, 0, (float)(R->ck_n - 2)); return atan2f(R->ck_z[i + 1] - R->ck_z[i], R->ck_x[i + 1] - R->ck_x[i]); }
/* ------------------------------------------------------ plant templates (session 3) */

static RtTemplate *tpl; static int ntpl;

int route_templates_load(const char *path)
{
    FILE *f = fopen(path, "r");
    if (!f) { fprintf(stderr, "route: no %s (tools/make_plants.py, tools/make_hut.py)\n", path); return 0; }
    char line[512]; int got = 0, k = 0;
    RtTemplate *t = NULL;
    while (fgets(line, sizeof(line), f)) {
        if (!strncmp(line, "plant ", 6)) {
            tpl = realloc(tpl, sizeof(RtTemplate) * (size_t)(ntpl + 1));
            t = &tpl[ntpl++]; memset(t, 0, sizeof(*t));
            if (sscanf(line + 6, "%15s %d %d %f %f %d %3s", t->name, &t->variant, &t->lod, &t->h, &t->r, &t->ntri, t->zone) < 6) { ntpl--; t = NULL; continue; }
            t->kind = malloc((size_t)t->ntri); t->rgb = malloc(sizeof(float) * 3 * (size_t)t->ntri); t->p = malloc(sizeof(float) * 9 * (size_t)t->ntri);
            k = 0; got++;
        } else if (t && line[0] == 't' && line[1] == ' ' && k < t->ntri) {
            char kc; float *c = t->rgb + 3 * k, *q = t->p + 9 * k;
            if (sscanf(line + 2, " %c %f %f %f %f %f %f %f %f %f %f %f %f", &kc, c, c + 1, c + 2, q, q + 1, q + 2, q + 3, q + 4, q + 5, q + 6, q + 7, q + 8) == 13) t->kind[k++] = kc;
        }
    }
    fclose(f);
    return got;
}
int route_template_find(const char *name, int variant, int lod)
{
    for (int i = 0; i < ntpl; i++) if (tpl[i].variant == variant && tpl[i].lod == lod && !strcmp(tpl[i].name, name)) return i;
    return -1;
}
int route_template_count(void) { return ntpl; }
const RtTemplate *route_template(int i) { return &tpl[i]; }

/* ------------------------------------------------- the forest along the road (session 3) */
/* From the NSW vegetation map (docs/forest/species.md): brown barrel fern forest at the hut edge and in stretches of
 * the plateau, messmate forest for most of it, a short peppermint patch, mountain grey gum ("monkey gum") down the
 * descent to the ford and the bridge; tree ferns, shield ferns and the silver-leaved gully trees in the damp gullies. */
enum { FT_BB, FT_MM, FT_PP, FT_GG };
static int forest_type(float s, float u)
{
    if (s < 350 || (s > 2930 && s < 3400) || (s > 3820 && s < 4000) || s > 7600) return FT_BB;
    if (s > 1900 && s < 2100) return FT_PP;
    if (s > 6300) return FT_GG;
    if (s > 4000 && u < 0.3f) return FT_GG;
    return FT_MM;
}
static float window01(float s, float c, float half) { return 1.0f - smooth01((fabsf(s - c) - half * 0.5f) / (half * 0.5f + 1.0f)); }
static float wetness(const Route *R, float s)
{
    float w = window01(s, 3910.0f, 110.0f);
    if (R->ford_s > 0) w = fmaxf(w, window01(s, R->ford_s, 240.0f));
    if (R->bridge_s0 > 0) w = fmaxf(w, window01(s, R->bridge_s0, 300.0f));
    return w;
}

static int tsel(const char *name, uint32_t *sd, int nvar, int big_chance)
{
    int v = (big_chance && rnd(sd) < 0.09f) ? 3 : (int)(rnd(sd) * (float)nvar) % nvar;
    int i = route_template_find(name, v, 0);
    return i >= 0 ? i : route_template_find(name, 0, 0);
}

typedef struct { RtTree *t; int n, cap; } TreeList;
static RtTree *tl_add(TreeList *L, float x, float z, int t0, int kind, uint32_t *sd, float lat)
{
    if (t0 < 0) return NULL;
    if (L->n == L->cap) { L->cap = L->cap ? L->cap * 2 : 1 << 16; L->t = realloc(L->t, sizeof(RtTree) * (size_t)L->cap); }
    RtTree *r = &L->t[L->n++];
    memset(r, 0, sizeof(*r));
    const RtTemplate *T0 = &tpl[t0];
    r->x = x; r->z = z; r->seed = *sd; r->kind = (unsigned char)kind; r->lat = lat;
    r->t0 = (short)t0; r->t1 = (short)route_template_find(T0->name, T0->variant, 1);
    r->yaw = rnd(sd) * 6.2831853f; r->scale = 0.88f + 0.24f * rnd(sd);
    r->h = T0->h * r->scale; r->r = kind == 0 ? T0->r * r->scale : 0.0f;
    return r;
}

/* The gate across the side road (follow-up): a locked boom gate, drawn, with hidden solid posts along the boom, and a
 * line of fence posts out into the bush either side. Spliced into the junction chunk's list. */
static void add_gate(Route *R)
{
    if (R->gate_t <= 0 || R->nchunks < 1) return;
    int c = (int)clampf(floorf(R->junction_s / (RT_DS * RT_CHUNK)), 0, (float)(R->nchunks - 1));
    RtTree add[64]; int na = 0;
    float ax = -sinf(R->gate_yaw), az = cosf(R->gate_yaw);   /* across the side road */
    int tg = route_template_find("boomgate", 0, 0), tf = route_template_find("fencepost", 0, 0);
    if (tg >= 0) {
        RtTree *q = &add[na++]; memset(q, 0, sizeof(*q));
        q->x = R->gate_x; q->z = R->gate_z; q->kind = 4; q->t0 = (short)tg; q->t1 = -1; q->lat = 99.0f; q->scale = 1.0f; q->h = route_template(tg)->h;
        q->yaw = R->gate_yaw + (float)(M_PI / 2);
    }
    for (float a = -4.4f; a <= 4.41f; a += 0.45f) {   /* the boom: solid all the way across */
        RtTree *q = &add[na++]; memset(q, 0, sizeof(*q));
        q->x = R->gate_x + ax * a; q->z = R->gate_z + az * a; q->r = 0.25f; q->kind = 3; q->t0 = q->t1 = -1; q->lat = 99.0f;
    }
    for (int sd = -1; sd <= 1; sd += 2)   /* fence posts out past the ditch, 1.5 m apart */
        for (float a = 5.9f; a < 13.0f && na < 64; a += 1.5f) {
            float x = R->gate_x + ax * a * (float)sd, z = R->gate_z + az * a * (float)sd;
            RtTree *q = &add[na++]; memset(q, 0, sizeof(*q));
            q->x = x; q->z = z; q->r = 0.12f; q->kind = tf >= 0 ? 0 : 3; q->t0 = (short)tf; q->t1 = -1; q->lat = 99.0f; q->scale = 1.0f; q->h = 1.3f;
            q->yaw = R->gate_yaw;
        }
    R->tree = realloc(R->tree, sizeof(RtTree) * (size_t)(R->ntree + na));
    int at = R->tree_first[c + 1];
    memmove(R->tree + at + na, R->tree + at, sizeof(RtTree) * (size_t)(R->ntree - at));
    memcpy(R->tree + at, add, sizeof(RtTree) * (size_t)na);
    R->ntree += na;
    for (int k = c + 1; k <= R->nchunks; k++) R->tree_first[k] += na;
}

/* Build C, the creek way: where no trunk may stand: the bed and a margin, the way in through the forest's edge, and a lane
 * round each log jam and the waterfall (the bypass, on the side away from the obstacle). */
static int creek_keep_clear(const Route *R, float s, float lat)
{
    float BW = route_creek_bedw(R, s), a = fabsf(lat);
    if (a < BW + 2.5f) return 1;
    if (s > R->ck_exit_s - 45.0f && s < R->ck_exit_s + 45.0f && a < 7.0f) return 1;
    for (int j = 0; j < R->ck_nbyp; j++) if (R->ck_byp_kind[j] != 2 && fabsf(s - R->ck_byp_s[j]) < 28.0f && lat * (float)R->ck_byp_side[j] > 0 && a < BW + 10.0f) return 1;
    if (R->ck_fall_s > 0 && fabsf(s - R->ck_fall_s) < 34.0f && lat * (float)R->ck_fall_side > 0 && a < BW + 11.0f) return 1;
    if (R->camp_on && fabsf(s - R->camp_s) < 22.0f && lat * (float)R->camp_side > 0 && a < BW + 16.0f) return 1;   /* build D: the camp's flat */
    for (int m = 0; m < R->ck_nmark; m++) if (fabsf(s - R->ck_mark_s[m]) < 5.0f) return 1;   /* and room round each marker */
    return 0;
}
static void tl_post(TreeList *L, float x, float z, float r, int kind)
{
    if (L->n == L->cap) { L->cap = L->cap ? L->cap * 2 : 1024; L->t = realloc(L->t, sizeof(RtTree) * (size_t)L->cap); }
    RtTree *q = &L->t[L->n++]; memset(q, 0, sizeof(*q));
    q->x = x; q->z = z; q->r = r; q->kind = (unsigned char)kind; q->t0 = q->t1 = -1; q->lat = 99.0f;
}
static void tl_prop(TreeList *L, const char *name, int variant, float x, float z, float yaw, float scale, int kind, float lat)
{
    int t0 = route_template_find(name, variant, 0); if (t0 < 0) return;
    uint32_t sd = (uint32_t)(x * 31.0f) ^ (uint32_t)(z * 17.0f);
    RtTree *t = tl_add(L, x, z, t0, kind, &sd, lat);
    if (t) { t->yaw = yaw; t->scale = scale; t->h = tpl[t0].h * scale; t->r = kind == 0 ? tpl[t0].r * scale : 0; if (lat > 98.0f) t->t1 = (short)t0; }
}
/* Part 2: the creek's many small things (stones, twigs, branches) drawn like the ground plants: full detail only near
 * (their simpler model further off), not as always-full props */
static void small_bit(TreeList *L, const char *name, const struct RtBit *q, float scale)
{
    int t0 = route_template_find(name, q->var, 0); if (t0 < 0) return;
    uint32_t sd = (uint32_t)(q->x * 13.0f) ^ (uint32_t)(q->z * 7.0f);
    RtTree *t = tl_add(L, q->x, q->z, t0, 4, &sd, 10.0f);
    if (t) { t->yaw = q->yaw; t->scale = scale; t->h = tpl[t0].h * scale; }
}
/* The creek way's bush, chunk by chunk along it: tall gums back from the water, tree ferns, blackwood and the silver gully
 * trees on the flats and banks, shield ferns and Lomandra by the water; its own bank with a row of trunks and thick scrub;
 * two log jams and the waterfall's rocks; firewood lying on the flats; and at the way in, the tells. */
static void place_creek(Route *R)
{
    TreeList L = { R->tree, R->ntree, R->ntree };
    int *bit_chunk = malloc(sizeof(int) * (size_t)(R->ck_nbit > 0 ? R->ck_nbit : 1));   /* part 2: the chunk each of the creek's things is drawn in */
    for (int b = 0; b < R->ck_nbit; b++) { RtNear bn = route_creek_near(R, R->ck_bit[b].x, R->ck_bit[b].z); bit_chunk[b] = (int)clampf(floorf(bn.s / (RT_DS * RT_CHUNK)), 0, (float)(R->ck_nch - 1)); }
    int ncx = R->gx - 1, nlogs = 0;
    float tell_s = R->ck_exit_s - 8.0f; int tell_c = (int)clampf(floorf(tell_s / (RT_DS * RT_CHUNK)), 0, (float)(R->ck_nch - 1));
    { float hd = route_heading_at_creek(R, tell_s), cx, cz; route_creek_point(R, tell_s, &cx, &cz);
      R->tell_x = cx - sinf(hd) * 6.3f; R->tell_z = cz + cosf(hd) * 6.3f; R->tell_yaw = -(hd + (float)M_PI);   /* the blaze (the model's +x) turned back to the clearing */
      R->post_x = cx + sinf(hd) * 5.6f; R->post_z = cz - cosf(hd) * 5.6f; R->post_yaw = -(hd + 0.5f); }
    for (int c = 0; c < R->ck_nch; c++) {
        int idx = R->nchunks + c;
        R->tree_first[idx] = L.n;
        if (c == tell_c) {   /* the way in's tells, soft (they're only drawn in Explorer, so they mustn't stop a Drift ute unseen) */
            tl_prop(&L, "tell", 0, R->tell_x, R->tell_z, R->tell_yaw, 1.0f, 2, 99.0f);
            tl_prop(&L, "tell", 1, R->post_x, R->post_z, R->post_yaw, 1.0f, 2, 99.0f);
            for (int q = 0; q < 14; q++) {   /* a screen of thick scrub across the mouth: soft, the ute pushes through it */
                float ss = R->ck_exit_s - 2.0f + 4.0f * (float)(q % 2) + 1.5f * vnoise((float)q, 1.0f), lat = -6.5f + 13.0f * (float)(q / 2) / 6.0f + (vnoise((float)q, 5.0f) - 0.5f) * 1.5f;
                float hd = route_heading_at_creek(R, ss), cx, cz; route_creek_point(R, ss, &cx, &cz);
                uint32_t sd = 4242u + (uint32_t)q; int t0 = tsel("edgescrub", &sd, 3, 0);
                RtTree *t = tl_add(&L, cx - sinf(hd) * lat, cz + cosf(hd) * lat, t0, 2, &sd, fabsf(lat));
                if (t) t->scale = 0.8f + 0.25f * vnoise((float)q, 9.0f);
            }
            /* build D: the thicket wrapping both banks where the creek leaves the clearing: scrub thick on the bank's face all
             * round (soft, but the bank behind it holds the ute), and a line of trunks along its top */
            float a0 = R->ck_exit_s - T.creek_wall_from;
            uint32_t ws = 6060u; int nth = 0;
            for (float ss = a0 - 18.0f; ss < R->ck_exit_s + 70.0f; ss += 2.6f)
                for (float lat = -48.0f; lat <= 48.0f; lat += 2.6f) {
                    float jx = (rnd(&ws) - 0.5f) * 1.8f, jl = (rnd(&ws) - 0.5f) * 1.8f, roll = rnd(&ws);
                    float hd = route_heading_at_creek(R, ss + jx), cx, cz; route_creek_point(R, ss + jx, &cx, &cz);
                    float l2 = lat + jl, px = cx - sinf(hd) * l2, pz = cz + cosf(hd) * l2;
                    RtNear c2 = route_creek_near(R, px, pz);
                    if (c2.s < a0 - 16.0f || c2.dist < 4.6f) continue;
                    float ramp = 30.0f * (1.0f - smooth01((c2.s - (a0 - 16.0f)) / 16.0f)), wv = creek_chan_e(R, c2.s, c2.dist) - ramp;
                    float k = fmaxf(creek_chan_e(R, c2.s, c2.dist + 1.0f) - creek_chan_e(R, c2.s, c2.dist), 0.3f), wm = wv / k;   /* metres into the wall */
                    if (wv < 0.3f || wm > 13.0f) continue;
                    if (wm < 8.5f && roll < 0.88f) {
                        uint32_t sd = 7070u + (uint32_t)nth * 31u; int t0 = tsel("edgescrub", &sd, 3, 0);
                        RtTree *t = tl_add(&L, px, pz, t0, 2, &sd, 99.0f); if (t) t->scale = 0.85f + 0.3f * rnd(&ws);
                        nth++;
                    } else if (wm >= 8.5f && roll < 0.35f) {
                        uint32_t sd = 7171u + (uint32_t)nth * 37u; int t0 = tsel(roll < 0.17f ? "greygum" : "blackwood", &sd, 3, 0);
                        tl_add(&L, px, pz, t0, 2, &sd, 99.0f); nth++;   /* soft: they stand in Drift's clearing too, unseen (the bank holds the ute in Explorer) */
                    }
                }
            fprintf(stderr, "route: the thicket round the way in: %d plants\n", nth);
        }
        for (int q = R->cell_first[idx]; q < R->cell_first[idx + 1]; q++) {
            int k = R->cell_list[q], i = k % ncx, j = k / ncx;
            uint32_t sd = hash32((uint32_t)k * 2246822519U + 777U);
            for (int draw = 0; draw < 9; draw++) {
                float x = R->gx0 + ((float)i + rnd(&sd)) * R->cell, z = R->gz0 + ((float)j + rnd(&sd)) * R->cell, roll = rnd(&sd), r2 = rnd(&sd);
                RtNear cn = route_creek_near(R, x, z);
                if (cn.dist > 150.0f) continue;
                float BW = route_creek_bedw(R, cn.s), a = cn.dist, e = a - route_creek_half(R, cn.s);
                int t0 = -1, kind = 2;
                if (e > T.edge_w + 5.0f) continue;
                if (draw == 0) {   /* tall gums, back from the water */
                    if (a > BW + 6.0f && e < T.edge_stop && roll < 0.30f) { t0 = tsel(r2 < 0.45f ? "greygum" : r2 < 0.75f ? "brownbarrel" : "messmate", &sd, 3, 1); kind = 0; }
                } else if (draw == 1) {   /* tree ferns: the wet gully */
                    if (a > BW + 2.5f && a < BW + 18.0f && e < T.edge_stop && roll < 0.30f) { t0 = tsel(r2 < 0.5f ? "softfern" : "roughfern", &sd, 3, 0); kind = 0; }
                } else if (draw == 2) {   /* blackwood and the silver gully trees */
                    if (a > BW + 3.0f && e < T.edge_stop && roll < 0.22f) { if (r2 < 0.5f) { t0 = tsel("blackwood", &sd, 3, 0); kind = 0; } else t0 = tsel("gullytree", &sd, 3, 0); }
                } else if (draw == 7) {   /* the bank's row of trunks */
                    if (e > T.edge_stop + 2.0f && roll < T.edge_trees) { t0 = tsel(r2 < 0.5f ? "greygum" : "blackwood", &sd, 3, 0); kind = 0; }
                } else if (draw == 8) {   /* and its scrub (build D: thicker, and down onto the banks by the bed: drive-through, but tight) */
                    if (e > -4.0f && roll < 0.9f * T.edge_scrub) t0 = tsel("edgescrub", &sd, 3, 0);
                    else if (cn.s > R->ck_exit_s + 60.0f && a > BW + 1.5f && roll < T.creek_scrub && !creek_keep_clear(R, cn.s, cn.lat)) t0 = tsel("edgescrub", &sd, 3, 0);   /* not on the climb-outs */
                } else {   /* the ground layer: ferns and Lomandra by the water, bracken and litter further up */
                    if (a < BW * 0.8f) continue;   /* the gravel and the water stay bare */
                    if (roll < 0.7f * T.ground_density) t0 = tsel(a < BW + 4.0f ? (r2 < 0.5f ? "lomandra" : "shieldfern") : r2 < 0.35f ? "shieldfern" : r2 < 0.6f ? "bracken" : r2 < 0.8f ? "litter" : "lomandra", &sd, 3, 0);
                }
                if (t0 < 0) continue;
                if (kind == 0 && creek_keep_clear(R, cn.s, cn.lat)) continue;
                if (kind == 0 && fabsf(route_near(R, x, z).lat) < route_half_width() + T.verge_w + T.ditch_w + 1.0f) continue;   /* off the road at the ford */
                tl_add(&L, x, z, t0, kind, &sd, fminf(a, 97.0f));
            }
        }
        /* this chunk's obstacles and firewood */
        float s0 = (float)(c * RT_CHUNK) * RT_DS, s1 = s0 + RT_CHUNK * RT_DS;
        /* build D, part 2: the creek's things, drawn on their bumps (nothing here is solid: the ute drives over them) */
        for (int b = 0; b < R->ck_nbit; b++) {
            if (bit_chunk[b] != c) continue;
            const struct RtBit *q = &R->ck_bit[b];
            switch (q->type) {
            case BIT_STONE: small_bit(&L, "creekrock", q, q->r / (q->var == 0 ? 0.35f : q->var == 1 ? 0.5f : 0.7f) * 1.1f); break;
            case BIT_BOULDER: tl_prop(&L, "boulder", q->var, q->x, q->z, q->yaw, q->r / (q->var == 0 ? 0.7f : 1.0f), 4, 99.0f); break;
            case BIT_STUMP: tl_prop(&L, "stump", q->var, q->x, q->z, q->yaw, q->r / 0.4f, 4, 99.0f); break;
            case BIT_BRANCH: small_bit(&L, "branch", q, 1.0f); break;
            case BIT_SMALLLOG: tl_prop(&L, "smalllog", q->var, q->x, q->z, q->yaw, q->r / (q->var ? 0.16f : 0.2f), 4, 99.0f); break;
            case BIT_MOSSY: tl_prop(&L, "deadfall", q->var, q->x, q->z, q->yaw, q->h, 4, 99.0f); break;
            case BIT_TWIGS: small_bit(&L, "twigs", q, 1.0f); break;
            }
        }
        /* build D: the stock route's markers (part 2: drawn only, nothing solid) */
        for (int m = 0; m < R->ck_nmark; m++) {
            if (R->ck_mark_s[m] < s0 || R->ck_mark_s[m] >= s1) continue;
            tl_prop(&L, "marker", R->ck_mark_v[m], R->ck_mark_x[m], R->ck_mark_z[m], R->ck_mark_yaw[m], 1.3f, 4, 99.0f);   /* a little larger than life, to be seen from the way down */
        }
        /* build D: the cattleman's camp: the fireplace (solid stones) and the trunk (drawn by the game: it opens) */
        if (R->camp_on && R->camp_s >= s0 && R->camp_s < s1) {
            tl_prop(&L, "camp", 0, R->camp_x, R->camp_z, R->camp_yaw, 1.0f, 4, 99.0f);   /* part 2: drawn only (drive over its low stones) */
        }
        for (float ls = ceilf((s0 - 60.0f) / 340.0f) * 340.0f + 60.0f; ls < s1; ls += 340.0f) {   /* firewood on the flats, about one every 340 m */
            if (ls < R->ck_exit_s + 80.0f || ls > R->ck_len - 60.0f) continue;
            /* build D: where build C had them (the bed's old width), so Drift's log bot (which sees every log) is unchanged */
            float BW = 3.0f + 3.0f * vnoise(ls / 140.0f, 4.0f), hd = route_heading_at_creek(R, ls), side = ((int)(ls / 340.0f) & 1) ? 1.0f : -1.0f, cx, cz;
            route_creek_point(R, ls, &cx, &cz);
            float lat = side * (BW + 3.5f), lx = cx - sinf(hd) * lat, lz = cz + cosf(hd) * lat;
            int tl = route_template_find("log", 2, 0); if (tl < 0) tl = route_template_find("log", 0, 0);
            if (tl < 0) continue;
            uint32_t sd = (uint32_t)ls; RtTree *t = tl_add(&L, lx, lz, tl, 2, &sd, BW + 3.5f);
            if (!t) continue;
            t->yaw = hd + 0.2f * side; t->wood = 1; t->scale = 1.0f;
            float half = 0.5f * 4.5f, rad = 0.25f, cxl = cosf(t->yaw), czl = sinf(t->yaw);
            int m = (int)(2.0f * half / (rad * 1.2f)) + 1;
            for (int q2 = 0; q2 <= m; q2++) { float f = -half + 2.0f * half * (float)q2 / (float)m; tl_post(&L, lx + cxl * f, lz + czl * f, rad, 5); L.t[L.n - 1].wood = 1; }
            nlogs++;
        }
    }
    R->tree_first[R->nchunks + R->ck_nch] = L.n;
    R->tree = L.t; R->ntree = L.n;
    R->wood_creek = nlogs; R->nwood += nlogs;
    free(bit_chunk);
}

/* Build C: the firewood. Drift keeps every log (they splinter and come back each run); Explorer's logs don't come back, so
 * the level 3 fire takes roaming. One log in each wood_every metres of road (the one with most room round it, near the
 * road: the play test wanted them easy to find), a few more near the bridge, and any lying in the hut's clearing. Marked wood = 1 with
 * their posts. */
static void pick_firewood(Route *R)
{
    float every = fmaxf(T.wood_every, 50.0f);
    int nb = (int)(R->length / every) + 1, *best = malloc(sizeof(int) * (size_t)nb), tl = route_template_find("log", 0, 0);
    float *bl = malloc(sizeof(float) * (size_t)nb);
    for (int b = 0; b < nb; b++) { best[b] = -1; bl[b] = -1; }
    R->nwood = R->wood_clearing = R->wood_road = R->wood_creek = R->wood_bridge = 0;
    int bridge_n = 0;
    for (int k = 0; k < R->ntree; k++) {
        RtTree *t = &R->tree[k];
        if (t->t0 < 0 || strcmp(tpl[t->t0].name, "log")) continue;
        (void)tl;
        RtNear n = route_near(R, t->x, t->z);
        if (route_in_clearing(R, t->x, t->z)) { t->wood = 1; R->wood_clearing++; continue; }
        if (R->bridge_s0 > 0 && n.s > R->bridge_s0 - 400.0f && fabsf(n.lat) > 12.0f && bridge_n < (int)T.wood_bridge && (int)(n.s / 80.0f) % 2 == 0) {
            int dup = 0; for (int q = 0; q < k; q++) if (R->tree[q].wood && R->tree[q].t0 >= 0 && hypotf(R->tree[q].x - t->x, R->tree[q].z - t->z) < 60.0f) dup = 1;
            if (!dup) { t->wood = 1; bridge_n++; R->wood_bridge++; continue; }
        }
        int b = (int)clampf(n.s / every, 0, (float)(nb - 1));
        if (fabsf(n.lat) < 6.0f || fabsf(n.lat) > 26.0f) continue;   /* off the road, but not deep in the bush (play test: easy to see from it) */
        int crowd = 0;   /* the one in each stretch with the most room round it (fewest trunks within 5 m) */
        for (int q = k - 400 > 0 ? k - 400 : 0; q < k + 400 && q < R->ntree; q++) if (R->tree[q].kind == 0 && hypotf(R->tree[q].x - t->x, R->tree[q].z - t->z) < 5.0f) crowd++;
        float score = 100.0f - (float)crowd * 10.0f - fabsf(n.lat) * 0.3f;   /* room round it, and near the road */
        if (score > bl[b]) { bl[b] = score; best[b] = k; }
    }
    for (int b = 0; b < nb; b++) if (best[b] >= 0 && !R->tree[best[b]].wood) { R->tree[best[b]].wood = 1; R->wood_road++; }
    for (int k = 0; k < R->ntree; k++)   /* the posts take their log's mark */
        if (R->tree[k].kind == 5) { int q = k; while (q > 0 && R->tree[q].kind == 5) q--; R->tree[k].wood = R->tree[q].wood; }
    R->nwood = R->wood_clearing + R->wood_road + R->wood_bridge;
    free(best); free(bl);
}

/* Trees: per 5 m cell, by how wooded the imagery says it is and where along the road it is. Open frost-hollow
 * grass round the hut with a snow gum fringe and a hard forest edge; tall gums with a midstorey and a ground layer
 * along the track; tree ferns and gully trees near the creeks. Trunks stay out past the ditch; soft plants may
 * reach the verge. */
static void place_trees(Route *R)
{
    float W = route_half_width(), e1 = W + T.verge_w, e2 = e1 + T.ditch_w;
    int ncx = R->gx - 1;
    TreeList L = { 0 };
    int have = ntpl > 0, hut = R->hut_x != 0 || R->hut_z != 0;
    for (int c = 0; c < R->nchunks; c++) {
        R->tree_first[c] = L.n;
        if (!have) continue;
        for (int q = R->cell_first[c]; q < R->cell_first[c + 1]; q++) {
            int k = R->cell_list[q], i = k % ncx, j = k / ncx;
            uint32_t sd = hash32((uint32_t)k * 2654435761U + 12345U);
            float cx = R->gx0 + ((float)i + 0.5f) * R->cell, cz = R->gz0 + ((float)j + 0.5f) * R->cell;
            RtNear nc = route_near(R, cx, cz);
            float cov = canopy_at(R, cx, cz), dh = hut ? hypotf(cx - R->hut_x, cz - R->hut_z) : 1e9f;
            float wet = wetness(R, nc.s), u = vnoise(cx / 60.0f, cz / 60.0f);
            int ft = forest_type(nc.s, u);
            float hollow = hut ? 1.0f - smooth01((dh - R->clearing_r * 0.9f) / (R->clearing_r * 0.9f)) : 0.0f; /* 1 in the hut's frost hollow */
            float forest = smooth01((cov - 0.42f) / 0.2f);   /* a hard edge: the imagery's canopy, sharpened */
            hollow *= 1.0f - forest * smooth01((dh - 30.0f) / 10.0f);   /* session 3b: the hollow is the open grass; where the imagery has forest
                                                                         * 35 m or more out (the edge west of the hut) it stays forest; the
                                                                         * scrub band hugging the hut's north-east side does not grow gums */
            float ec = route_edge(R, nc, cx, cz);
            if (ec > T.edge_w + 6.0f) continue;   /* build B: nothing past the edge's trees (was 62 m from the road) */
            for (int draw = 0; draw < 10; draw++) {
                float x = R->gx0 + ((float)i + rnd(&sd)) * R->cell, z = R->gz0 + ((float)j + rnd(&sd)) * R->cell;
                RtNear nr = route_near(R, x, z);
                float a = fabsf(nr.lat), dp = hypotf(x - R->pad_x, z - R->pad_z), roll = rnd(&sd);
                int on_route = nr.s > -2.0f && nr.s < R->length + 2.0f;
                int t0 = -1, kind = 2;
                float p = 0, e = draw >= 8 || draw == 0 ? route_edge(R, nr, x, z) : 0;
                if (draw == 0 && e > T.edge_w + 5.0f) continue;
                if (a > 64.0f && draw != 8 && rnd(&sd) < 0.5f) continue;   /* build B: the hut clearing's forest edge, out past the old strip: half as thick */
                if (draw == 8 && a > 64.0f && rnd(&sd) < 0.35f) continue;
                if (draw == 2 && a > 40.0f) continue;                  /* build B: 40 m out, background only: no midstorey ... */
                if (draw == 0 && a > 40.0f && rnd(&sd) < 0.4f) continue;   /* ... and fewer tall gums (the edge's row of trunks closes the view) */
                if (draw == 8) {            /* build B: the edge, a close row of trunks along the top of the bank */
                    if (e < T.edge_stop + 2.0f || e > T.edge_w + 5.0f) continue;
                    p = T.edge_trees;
                    if (roll < p) { float r2 = rnd(&sd); t0 = tsel(r2 < 0.45f ? "messmate" : r2 < 0.7f ? "brownbarrel" : r2 < 0.85f ? "blackwood" : "greygum", &sd, 3, 0); kind = 0; }
                } else if (draw == 9) {     /* ... and thick scrub on its face */
                    if (e < -4.0f || e > T.edge_w + 4.0f) continue;
                    p = 0.9f * T.edge_scrub;
                    if (roll < p) t0 = tsel("edgescrub", &sd, 3, 0);
                } else if (draw == 0) {            /* a tall gum */
                    p = T.tree_density * forest * (1.0f - 0.85f * hollow);
                    if (roll < p) {
                        float r2 = rnd(&sd);
                        const char *sp = ft == FT_BB ? (r2 < 0.6f ? "brownbarrel" : wet > 0.3f || r2 < 0.8f ? "greygum" : "messmate")
                                       : ft == FT_MM ? (r2 < 0.65f ? "messmate" : r2 < 0.8f ? "brownbarrel" : r2 < 0.95f ? "greygum" : "peppermint")
                                       : ft == FT_PP ? (r2 < 0.5f ? "peppermint" : r2 < 0.8f ? "greygum" : "messmate")
                                       : (r2 < 0.65f ? "greygum" : r2 < 0.85f ? "brownbarrel" : "messmate");
                        t0 = tsel(sp, &sd, !strcmp(sp, "peppermint") ? 3 : 3, strcmp(sp, "peppermint") != 0); kind = 0;
                    }
                } else if (draw == 1) {     /* the frost hollow's rim: snow gums (black sallee shares the shape) */
                    float rim = hut ? window01(dh, R->clearing_r * 1.05f, R->clearing_r * 0.7f) : 0.0f;
                    p = 0.22f * rim * (0.3f + 0.7f * smooth01((cov - 0.15f) / 0.4f));
                    if (roll < p) { t0 = tsel("snowgum", &sd, 3, 0); kind = 0; }
                } else if (draw == 2) {     /* midstorey: blackwood and wattle; tree ferns and gully trees where it's wet */
                    p = T.mid_density * forest * (1.0f - hollow) * (0.6f + 0.8f * wet);
                    if (roll < p) {
                        float r2 = rnd(&sd);
                        if (wet > 0.35f && r2 < 0.45f) { t0 = tsel(r2 < 0.25f ? "softfern" : "roughfern", &sd, 3, 0); kind = 0; }
                        else if (wet > 0.35f && r2 < 0.75f) t0 = tsel("gullytree", &sd, 3, 0);
                        else { t0 = tsel(r2 < 0.85f ? "blackwood" : "wattle", &sd, 3, 0); kind = 0; }
                    }
                } else {                    /* the ground layer (near the road only: further out the far land shows) */
                    if (a > 36.0f) continue;
                    float r2 = rnd(&sd);
                    if (hollow > 0.5f && forest < 0.5f) {   /* frost-hollow grass: kangaroo grass and Poa tussocks */
                        p = 0.55f * T.ground_density;
                        if (roll < p) t0 = tsel(r2 < 0.55f ? "kangaroo" : "poa", &sd, 3, 0);
                    } else {
                        float f2 = 0.25f + 0.75f * forest;
                        p = T.scrub_density * T.ground_density * f2 * (draw < 6 ? 1.0f : 0.5f);
                        if (roll < p) {
                            if (wet > 0.4f && r2 < 0.35f) t0 = tsel("shieldfern", &sd, 3, 0);
                            else if (r2 < 0.40f) t0 = tsel("bracken", &sd, 3, 0);
                            else if (r2 < 0.58f) t0 = tsel("poa", &sd, 3, 0);
                            else if (r2 < 0.72f) t0 = tsel("lomandra", &sd, 3, 0);
                            else if (r2 < 0.82f) t0 = tsel("heath", &sd, 3, 0);
                            else if (r2 < 0.88f) t0 = tsel("coprosma", &sd, 3, 0);
                            else if (r2 < 0.97f) t0 = tsel("litter", &sd, 3, 0);
                            else if (forest > 0.5f) t0 = tsel("log", &sd, 3, 0);
                        }
                    }
                }
                if (t0 < 0) continue;
                if (R->ck_n > 1 && (kind == 0 || draw == 8 || !strcmp(tpl[t0].name, "log"))) {   /* build C: keep the creek way clear */
                    RtNear cn = route_creek_near(R, x, z);
                    if (cn.dist < 80.0f && cn.s > R->ck_exit_s - 40.0f && (creek_keep_clear(R, cn.s, cn.lat) || ((draw == 8 || draw == 9) && cn.dist < route_creek_half(R, cn.s) + T.edge_stop + 3.0f))) continue;
                }
                const RtTemplate *tt = &tpl[t0];
                int is_log = !strcmp(tt->name, "log"), is_litter = !strcmp(tt->name, "litter");
                float keep = kind == 0 ? e2 + tt->r * 1.6f + 0.5f : is_log ? e2 + 2.5f : is_litter ? W + 0.3f : e1 - 0.2f;
                if (on_route && a < keep) continue;
                if (R->spur_n > 2) { float t2, d2 = route_spur_dist(R, x, z, &t2); if ((d2 < keep && t2 < SPUR_LEN - 4.0f) || hypotf(x - R->spur_x[2], z - R->spur_z[2]) < JUNCTION_R + keep - W) continue; }   /* off the side road and its splay too */
                if (dp < R->pad_r + (kind == 0 ? 5.0f : 1.0f)) continue;
                if (in_bridge(R, nr.s) && a < W + 3.0f) continue;
                if (hut && dh < 12.0f && (kind == 0 || hypotf(x - R->hut_x, z - R->hut_z) < 9.0f)) continue;
                RtTree *nt = tl_add(&L, x, z, t0, kind, &sd, fminf(a, 97.0f));   /* build B: plants far out in the hollow are still plants (lat over 90 = the hut's props) */
                if (nt && is_log) {   /* session 3b (the author): logs are hard. Near the road they lie roughly along it; hidden posts along each make it solid */
                    static const float logL[3] = { 7.0f, 11.0f, 4.5f }, logR[3] = { 0.35f, 0.5f, 0.25f };
                    float half = 0.5f * logL[tt->variant % 3] * nt->scale, rad = logR[tt->variant % 3] * nt->scale;
                    if (on_route && a < 25.0f) {
                        float dy = (rnd(&sd) - 0.5f) * 0.6f, room = a - e2 - rad - 0.3f;   /* the ends must stay out of the ditch */
                        if (fabsf(sinf(dy)) * half > room) dy = room > 0 ? asinf(clampf(room / half, 0, 1)) * (dy < 0 ? -1.0f : 1.0f) : 0.0f;
                        if (room < 0) { L.n--; continue; }
                        nt->yaw = route_heading_at(R, nr.s) + dy;
                    }
                    float cx_ = cosf(nt->yaw), cz_ = sinf(nt->yaw), lx = nt->x, lz = nt->z;
                    int m = (int)(2.0f * half / (rad * 1.2f)) + 1, log_at = L.n - 1, bad = 0;
                    for (int q = 0; q <= m && !bad; q++) {   /* a straight log beside a bend: drop it if any of it comes inside the ditch */
                        float f = -half + 2.0f * half * (float)q / (float)m;
                        RtNear pn = route_near(R, lx + cx_ * f, lz + cz_ * f);
                        if (pn.s > -2.0f && pn.s < R->length + 2.0f && fabsf(pn.lat) < e2 + rad + 0.5f) bad = 1;
                    }
                    if (bad) { L.n = log_at; continue; }
                    for (int q = 0; q <= m; q++) {
                        float f = -half + 2.0f * half * (float)q / (float)m;
                        if (L.n == L.cap) { L.cap *= 2; L.t = realloc(L.t, sizeof(RtTree) * (size_t)L.cap); }
                        RtTree *pp = &L.t[L.n++]; memset(pp, 0, sizeof(*pp));
                        pp->x = lx + cx_ * f; pp->z = lz + cz_ * f; pp->r = rad; pp->kind = 5; pp->t0 = pp->t1 = -1; pp->lat = 99.0f;
                    }
                }
            }
        }
    }
    R->tree_first[R->nchunks] = L.n;
    R->tree = L.t; R->ntree = L.n;
    pick_firewood(R);
    add_gate(R);
    route_add_props(R);
    if (R->ck_n > 1) place_creek(R);   /* build C: the creek way's bush, its obstacles and its firewood, in its own chunks */
    /* by cell, for collisions: trunks, posts and the hut's walls */
    int ncz = R->gy - 1, nc = ncx * ncz;
    free(R->tcell_first); free(R->tcell_list);
    R->tcell_first = calloc((size_t)nc + 1, sizeof(int));
    int *cnt = calloc((size_t)nc, sizeof(int)), nt = 0;
    for (int k = 0; k < R->ntree; k++) {
        const RtTree *t = &R->tree[k];
        if (t->kind != 0 && t->kind != 3 && t->kind != 5) continue;
        int i = (int)clampf((t->x - R->gx0) / R->cell, 0, (float)(ncx - 1)), j = (int)clampf((t->z - R->gz0) / R->cell, 0, (float)(ncz - 1));
        cnt[j * ncx + i]++; nt++;
    }
    for (int c = 0; c < nc; c++) R->tcell_first[c + 1] = R->tcell_first[c] + cnt[c];
    R->tcell_list = malloc(sizeof(int) * (size_t)(nt + 1));
    memset(cnt, 0, sizeof(int) * (size_t)nc);
    for (int k = 0; k < R->ntree; k++) {
        const RtTree *t = &R->tree[k];
        if (t->kind != 0 && t->kind != 3 && t->kind != 5) continue;
        int i = (int)clampf((t->x - R->gx0) / R->cell, 0, (float)(ncx - 1)), j = (int)clampf((t->z - R->gz0) / R->cell, 0, (float)(ncz - 1));
        R->tcell_list[R->tcell_first[j * ncx + i] + cnt[j * ncx + i]++] = k;
    }
    R->nsolid = nt;
    free(cnt);
}

/* The hut, its tanks and the campground (docs/hut/reference.md), all in chunk 0. Offsets are metres east and north of
 * the hut's OSM centroid. The hut is drawn from its template; rings of hidden posts along its walls make it solid. */
/* Moves a prop sideways off the road until it is `keep` metres from the centreline (the OSM spots are a few metres out,
 * and the track runs through the paddock). */
static void nudge_off_road(const Route *R, float *e, float *n, float keep)
{
    for (int it = 0; it < 40; it++) {
        RtNear nr = route_near(R, R->hut_x + *e, R->hut_z + *n);
        if (fabsf(nr.lat) >= keep || nr.s < -1.0f) return;
        float hd = route_heading_at(R, nr.s), side = nr.lat >= 0 ? 1.0f : -1.0f;
        *e += -sinf(hd) * side * 0.5f; *n += cosf(hd) * side * 0.5f;
    }
}
static void solid_post(Route *R, TreeList *L, float e, float n, float r)   /* one hidden solid post, metres E/N of the hut */
{
    if (L->n == L->cap) { L->cap *= 2; L->t = realloc(L->t, sizeof(RtTree) * (size_t)L->cap); }
    RtTree *q = &L->t[L->n++]; memset(q, 0, sizeof(*q));
    q->x = R->hut_x + e; q->z = R->hut_z + n; q->r = r; q->kind = 3; q->t0 = q->t1 = -1; q->lat = 99.0f;
}
static void prop_at(Route *R, TreeList *L, const char *name, float e, float n, float yaw, int kind, float r)
{
    int t0 = route_template_find(name, 0, 0);
    if (t0 < 0) return;
    if (strcmp(name, "hut")) nudge_off_road(R, &e, &n, route_half_width() + T.verge_w + T.ditch_w + 1.0f + tpl[t0].h * 0.0f + (strcmp(name, "plough") ? 0.5f : 3.0f));
    uint32_t sd = hash32((uint32_t)(e * 100.0f) * 73856093U ^ (uint32_t)(n * 100.0f) * 19349663U);
    RtTree *t = tl_add(L, R->hut_x + e, R->hut_z + n, t0, kind, &sd, 99.0f);
    if (!t) return;
    t->yaw = yaw; t->scale = 1.0f; t->h = tpl[t0].h; t->r = r; t->t1 = (short)t0;
    /* session 3b (the author): bollards, fence posts and the hitching rail's posts are hard, like trunks: a solid post
     * stops the ute and ends the run if hit faster than tree_min_speed */
    if (!strcmp(name, "bollard")) solid_post(R, L, e, n, BOLLARD_R);
    else if (!strcmp(name, "fencepost")) solid_post(R, L, e, n, 0.11f);
    else if (!strcmp(name, "rail")) { solid_post(R, L, e - cosf(yaw) * 1.3f, n - sinf(yaw) * 1.3f, 0.12f); solid_post(R, L, e + cosf(yaw) * 1.3f, n + sinf(yaw) * 1.3f, 0.12f); }
}
static void solid_ring(Route *R, TreeList *L, float e0, float n0, float e1, float n1, float r)
{
    float d = hypotf(e1 - e0, n1 - n0); int m = (int)(d / (r * 1.4f)) + 1;
    for (int k = 0; k <= m; k++) {
        float t = (float)k / (float)m;
        if (L->n == L->cap) { L->cap *= 2; L->t = realloc(L->t, sizeof(RtTree) * (size_t)L->cap); }
        RtTree *q = &L->t[L->n++]; memset(q, 0, sizeof(*q));
        q->x = R->hut_x + e0 + (e1 - e0) * t; q->z = R->hut_z + n0 + (n1 - n0) * t; q->r = r; q->kind = 3; q->t0 = q->t1 = -1; q->lat = 99.0f;
    }
}
void route_add_props(Route *R)
{
    if ((R->hut_x == 0 && R->hut_z == 0) || route_template_find("hut", 0, 0) < 0 || R->nchunks < 1) return;
    /* the props go at the end of chunk 0's list: shift the other chunks' trees along */
    TreeList L = { 0 };
    int n0 = R->tree_first[1];
    L.cap = R->ntree + 256; L.t = malloc(sizeof(RtTree) * (size_t)L.cap);
    memcpy(L.t, R->tree, sizeof(RtTree) * (size_t)n0); L.n = n0;
    prop_at(R, &L, "hut", 0, 0, 0, 4, 0);
    /* the hut's outline (OSM way 1347517164) and its tanks, as solid posts */
    static const float hv[7][2] = { { 3.4f, -1.9f }, { 1.1f, -5.9f }, { -6.3f, -1.5f }, { -1.8f, 6.2f }, { 3.0f, 3.55f }, { 0.6f, -0.45f }, { 3.4f, -1.9f } };
    for (int k = 0; k < 6; k++) solid_ring(R, &L, hv[k][0], hv[k][1], hv[k + 1][0], hv[k + 1][1], 0.5f);
    /* session 3b: the verandah's front and end, the squat tank in front of it, the chimney's firebox, the slim tank */
    solid_ring(R, &L, 3.4f, -1.9f, 4.9f, 0.7f, 0.4f); solid_ring(R, &L, 4.9f, 0.7f, 2.2f, 2.3f, 0.4f);
    solid_ring(R, &L, 4.1f, -1.7f, 5.1f, -1.1f, 1.0f);
    solid_ring(R, &L, -5.9f, 0.1f, -5.3f, 1.2f, 0.7f);
    solid_ring(R, &L, -6.1f, -2.3f, -6.1f, -2.3f, 0.55f);
    prop_at(R, &L, "toilet", -37.3f, 0.0f, 0, 4, 0); solid_ring(R, &L, -37.3f, -0.6f, -37.3f, 0.6f, 0.9f);
    prop_at(R, &L, "picnic", -10.5f, 9.5f, 0.5f, 4, 0);
    prop_at(R, &L, "firering", -6.5f, 11.0f, 0.3f, 4, 0);
    /* session 3b: the old blackwoods round the hut, dense and round-crowned (tools/make_plants.py "hutwood"), at the
     * crowns on the NSW aerial: a clump by the wing and the verandah to the north and north-east, the big one over the
     * picnic spot and two more towards the forest edge, and the big one on the north side of the track before the loop */
    { static const float bw[7][4] = { { 0.5f, 9.5f, 0, 1.0f }, { 6.0f, 5.8f, 2, 0.95f }, { 9.5f, 11.5f, 1, 1.0f }, { -9.0f, 14.0f, 1, 1.15f },
                                      { -14.5f, 12.0f, 0, 0.85f }, { -20.0f, 9.5f, 2, 0.9f }, { 28.3f, -20.1f, 1, 1.2f } };
      for (int k = 0; k < 7; k++) { int t0 = route_template_find("hutwood", (int)bw[k][2], 0); if (t0 < 0) continue;
          uint32_t sd = 777u + (uint32_t)k; RtTree *t = tl_add(&L, R->hut_x + bw[k][0], R->hut_z + bw[k][1], t0, 0, &sd, 99.0f);
          if (t) { t->scale = bw[k][3]; t->h = tpl[t0].h * bw[k][3]; t->r = tpl[t0].r * bw[k][3]; } } }
    /* bollards both sides of the track past the hut (every 2.4 m), the NPWS sign at the approach, an old fence line
     * running east across the paddock from the verandah */
    /* build C (the author, 21 Oct): bollards only right beside the track, a row along each edge, and none by the car park
     * or round the day-use spot. They stand bollard_gap apart, face to face: a little wider than the ute at its mirrors,
     * so it can just drive out between two, straight. */
    float W = route_half_width(), off = W + T.verge_w + T.ditch_w + 0.9f, step = fmaxf(T.bollard_gap, 1.0f) + 2.0f * BOLLARD_R;
    float boff = W + T.verge_w + T.ditch_w + 1.6f;   /* just past where prop_at would nudge them to anyway */
    float placed[128][2]; int nplaced = 0;
    for (int side = -1; side <= 1; side += 2) {   /* walked along each row's own line, so the spacing holds round the bends */
        float run = step, px = 0, pz = 0; int have = 0;
        for (float s = 0.0f; s < 90.0f; s += 0.05f) {
            float x, z; route_point(R, s, &x, &z);
            float hd = route_heading_at(R, s), bx = x - sinf(hd) * boff * (float)side, bz = z + cosf(hd) * boff * (float)side;
            if (have) run += hypotf(bx - px, bz - pz);
            px = bx; pz = bz; have = 1;
            if (run < step) continue;
            RtNear nb = route_near(R, bx, bz);
            if (fabsf(nb.lat) < boff - 0.3f) continue;   /* the inside of the hairpin, where the line folds back near the road */
            if (hypotf(bx - R->pad_x, bz - R->pad_z) < R->pad_r + 8.0f || hypotf(bx - R->hut_x, bz - R->hut_z) < 7.0f) continue;
            int close = 0;
            for (int k = 0; k < nplaced; k++) if (hypotf(placed[k][0] - bx, placed[k][1] - bz) < step - 0.02f) close = 1;
            if (close) continue;
            if (nplaced < 128) { placed[nplaced][0] = bx; placed[nplaced][1] = bz; nplaced++; }
            prop_at(R, &L, "bollard", bx - R->hut_x, bz - R->hut_z, (float)side, 4, 0);   /* hard posts (session 3b): prop_at adds the solid core */
            run = 0;
        }
    }
    { float x, z; route_point(R, 34.0f, &x, &z); float hd = route_heading_at(R, 34.0f);
      prop_at(R, &L, "sign", x - sinf(hd) * (off + 0.8f) - R->hut_x, z + cosf(hd) * (off + 0.8f) - R->hut_z, (float)(M_PI / 2) - hd, 4, 0); }
    /* session 3b, from the drive-in photo: the hitching rail along the track's north edge in front of the hut, the plough
     * just behind the sign, the old iron gate on its post by the big blackwood */
    { float x, z; route_point(R, 22.0f, &x, &z); float hd = route_heading_at(R, 22.0f);
      prop_at(R, &L, "rail", x - sinf(hd) * (off + 0.6f) - R->hut_x, z + cosf(hd) * (off + 0.6f) - R->hut_z, hd, 4, 0); }
    { float x, z; route_point(R, 36.0f, &x, &z); float hd = route_heading_at(R, 36.0f);
      float pe = x - sinf(hd) * (off + 3.8f) - R->hut_x, pn = z + cosf(hd) * (off + 3.8f) - R->hut_z;
      prop_at(R, &L, "plough", pe, pn, 2.4f, 4, 0); solid_ring(R, &L, pe - 1.0f, pn - 0.5f, pe + 1.0f, pn + 0.5f, 0.6f); }
    { float x, z; route_point(R, 44.0f, &x, &z); float hd = route_heading_at(R, 44.0f);
      prop_at(R, &L, "gate", x - sinf(hd) * (off + 1.6f) - R->hut_x, z + cosf(hd) * (off + 1.6f) - R->hut_z, hd + 0.5f, 4, 0); }
    for (int k = 0; k < 12; k++) {
        float e = 8.0f + (float)k * 3.6f, n = -3.0f - (float)k * 0.9f;
        RtNear nn = route_near(R, R->hut_x + e, R->hut_z + n);
        if (fabsf(nn.lat) < off + 0.5f) continue;
        prop_at(R, &L, "fencepost", e, n, 0.3f * (float)k, 4, 0);
    }
    int added = L.n - n0;
    if (L.n + (R->ntree - n0) > L.cap) { L.cap = L.n + (R->ntree - n0); L.t = realloc(L.t, sizeof(RtTree) * (size_t)L.cap); }
    memcpy(L.t + L.n, R->tree + n0, sizeof(RtTree) * (size_t)(R->ntree - n0));
    L.n += R->ntree - n0;
    for (int c = 1; c <= R->nchunks; c++) R->tree_first[c] += added;
    free(R->tree);
    R->tree = L.t; R->ntree = L.n;
}

/* Build C: the ute's own shape for posts (bollards, fence posts, the gate's boom): its half width in slices along it, from
 * the model, so threading it between two bollards is judged on the real ute (2.05 m at the mirrors, 1.77 m at the tail)
 * and not on its box. Trunks still use the box. */
static int post_n; static float post_z0[24], post_z1[24], post_hw[24];
void route_set_post_outline(int n, const float *z0, const float *z1, const float *hw)
{
    post_n = n < 24 ? n : 24;
    for (int i = 0; i < post_n; i++) { post_z0[i] = z0[i]; post_z1[i] = z1[i]; post_hw[i] = hw[i]; }
}
static int post_touch(float lf, float lr, float r)
{
    for (int i = 0; i < post_n; i++) {
        float of = lf > post_z1[i] ? lf - post_z1[i] : (lf < post_z0[i] ? post_z0[i] - lf : 0), orr = fmaxf(fabsf(lr) - post_hw[i], 0);
        if (of * of + orr * orr < r * r) return 1;
    }
    return 0;
}

int route_tree_hit(const Route *R, float cx, float cz, float yaw, float half_w, float front, float rear)
{
    int ncx = R->gx - 1, ncz = R->gy - 1;
    int ci = (int)floorf((cx - R->gx0) / R->cell), cj = (int)floorf((cz - R->gz0) / R->cell);
    float fx = sinf(yaw), fz = cosf(yaw), rx = cosf(yaw), rz = -sinf(yaw);
    for (int j = cj - 1; j <= cj + 1; j++)
        for (int i = ci - 1; i <= ci + 1; i++) {
            if (i < 0 || j < 0 || i >= ncx || j >= ncz) continue;
            int c = j * ncx + i;
            for (int q = R->tcell_first[c]; q < R->tcell_first[c + 1]; q++) {
                const RtTree *t = &R->tree[R->tcell_list[q]];
                if (t->kind == 6) continue;   /* a broken log's posts */
                if (t->kind == 5 && R->explore && !t->wood) continue;   /* build C: not there in Explorer */
                float dx = t->x - cx, dz = t->z - cz;
                float lf = dx * fx + dz * fz, lr = dx * rx + dz * rz;
                if (post_n > 0 && t->kind == 3 && t->r <= 0.2f) { if (post_touch(lf, lr, t->r)) return R->tcell_list[q]; continue; }
                float of = lf > front ? lf - front : (lf < rear ? rear - lf : 0), orr = fmaxf(fabsf(lr) - half_w, 0);
                if (of * of + orr * orr < t->r * t->r) return R->tcell_list[q];
            }
        }
    return -1;
}

#define CREEK_REACH 40.0f   /* the creek way's ground is built this far from its line (more where its strip is wider) */
static void creek_chunk_span(const Route *R, int index, int *i0, int *i1) { *i0 = (index - R->nchunks) * RT_CHUNK; *i1 = *i0 + RT_CHUNK; if (*i1 > R->ck_n - 1) *i1 = R->ck_n - 1; }
void route_chunk_centre(const Route *R, int index, float *x, float *z, float *radius)
{
    if (index >= R->nchunks) {   /* build C: a creek way chunk */
        int i0, i1; creek_chunk_span(R, index, &i0, &i1);
        float sx = 0, sz = 0; int k = 0;
        for (int i = i0; i <= i1; i++) { sx += R->ck_x[i]; sz += R->ck_z[i]; k++; }
        sx /= (float)k; sz /= (float)k;
        float r = 0; for (int i = i0; i <= i1; i++) r = fmaxf(r, hypotf(R->ck_x[i] - sx, R->ck_z[i] - sz));
        *x = sx; *z = sz; *radius = fmaxf(r + CREEK_REACH, R->chunk_r[index]);
        return;
    }
    int i0 = index * RT_CHUNK, i1 = i0 + RT_CHUNK; if (i1 > R->n - 1) i1 = R->n - 1;
    float sx = 0, sz = 0; int k = 0;
    for (int i = i0; i <= i1; i++) { sx += R->x[i]; sz += R->z[i]; k++; }
    sx /= (float)k; sz /= (float)k;
    float r = 0;
    for (int i = i0; i <= i1; i++) r = fmaxf(r, hypotf(R->x[i] - sx, R->z[i] - sz));
    *x = sx; *z = sz; *radius = fmaxf(r + TERRAIN_REACH, R->chunk_r[index]);
}
float route_chunk_line_r(const Route *R, int index)   /* build B: the chunk's own stretch of road (its full radius can take in the hut's clearing) */
{
    if (index >= R->nchunks) { float x, z, r; route_chunk_centre(R, index, &x, &z, &r); return fminf(r, 60.0f + CREEK_REACH); }
    int i0 = index * RT_CHUNK, i1 = i0 + RT_CHUNK; if (i1 > R->n - 1) i1 = R->n - 1;
    float sx = 0, sz = 0; int k = 0;
    for (int i = i0; i <= i1; i++) { sx += R->x[i]; sz += R->z[i]; k++; }
    sx /= (float)k; sz /= (float)k;
    float r = 0;
    for (int i = i0; i <= i1; i++) r = fmaxf(r, hypotf(R->x[i] - sx, R->z[i] - sz));
    return r + TERRAIN_REACH;
}

int route_load(Route *R, const char *dir)
{
    memset(R, 0, sizeof(*R));
    char p[512], q[512];
    float pad_elev;
    snprintf(p, sizeof(p), "%s/route.csv", dir);
    if (!read_route_csv(R, p)) return 0;
    snprintf(p, sizeof(p), "%s/meta.txt", dir);
    int meta = read_meta(R, p, &pad_elev);
    if (!meta) return 0;
    snprintf(p, sizeof(p), "%s/terrain.bin", dir);
    snprintf(q, sizeof(q), "%s/canopy.bin", dir);
    if (!read_grid(R, p, q)) return 0;
    if (R->hut_x != 0 || R->hut_z != 0) {   /* build B: the grid stops 120 m north of the car park, short of the hut's clearing:
                                             * grow it north (no data there yet: far_fill gives it the far land's heights) */
        int need = (int)ceilf((R->hut_z + 680.0f - (R->gz0 + (float)(R->gy - 1) * R->cell)) / R->cell);
        if (need > 0) {
            size_t old = (size_t)R->gx * R->gy, add = (size_t)R->gx * need;
            R->dem = realloc(R->dem, (old + add) * sizeof(float));
            for (size_t k = old; k < old + add; k++) R->dem[k] = NAN;
            if (R->canopy) { R->canopy = realloc(R->canopy, old + add); memset(R->canopy + old, 255, add); }
            R->gy += need;
        }
    }
    /* heights relative to the start pad */
    R->elev0 = isfinite(pad_elev) ? pad_elev : R->h[0];
    for (int i = 0; i < R->n; i++) { R->h[i] -= R->elev0; R->hdem[i] -= R->elev0; }
    size_t N = (size_t)R->gx * R->gy;
    R->known = malloc(N);
    unsigned char *farcov = far_fill(R, dir);   /* build B: the far land's heights where the 5 m DEM has none (was flat at the pad's level) */
    if (farcov && R->canopy) for (size_t k = 0; k < N; k++) if (!isfinite(R->dem[k]) && R->canopy[k] == 255) R->canopy[k] = farcov[k];   /* no imagery or DEM: the far land's cover */
    for (size_t k = 0; k < N; k++) { R->known[k] = isfinite(R->dem[k]); R->dem[k] = isfinite(R->dem[k]) ? R->dem[k] - R->elev0 : 0.0f; }
    for (size_t k = 0; k < N; k++) if (!R->known[k] && farcov && isfinite(R->hdem_far_tmp[k])) R->dem[k] = R->hdem_far_tmp[k] - R->elev0;
    free(R->hdem_far_tmp); R->hdem_far_tmp = NULL;
    if (read_creek(R, dir)) {   /* build C: the creek way, and its 5 m DEM where the road's has none, blended in over its last 25 m */
        for (int i = 0; i < R->ck_n; i++) R->ck_bed[i] -= R->elev0;
        char cp[512]; snprintf(cp, sizeof(cp), "%s/creek_dem.bin", dir);
        FILE *f = fopen(cp, "rb"); char mg[4]; int32_t cnx, cny; float cx0, cy0, cc;
        if (f && fread(mg, 1, 4, f) == 4 && !memcmp(mg, "BDT1", 4) && fread(&cnx, 4, 1, f) == 1 && fread(&cny, 4, 1, f) == 1 && fread(&cx0, 4, 1, f) == 1 && fread(&cy0, 4, 1, f) == 1 && fread(&cc, 4, 1, f) == 1 && cnx > 0 && cny > 0) {
            float *cd = malloc(sizeof(float) * (size_t)cnx * cny);
            if (fread(cd, sizeof(float), (size_t)cnx * cny, f) == (size_t)cnx * cny) {
                int merged = 0;
                for (int j = 0; j < cny; j++) for (int i = 0; i < cnx; i++) {
                    float v = cd[(size_t)j * cnx + i]; if (!isfinite(v)) continue;
                    float x = cx0 + (float)i * cc, z = cy0 + (float)j * cc;
                    int gi = (int)roundf((x - R->gx0) / R->cell), gj = (int)roundf((z - R->gz0) / R->cell);
                    if (gi < 0 || gj < 0 || gi >= R->gx || gj >= R->gy) continue;
                    size_t k = (size_t)gj * R->gx + gi;
                    if (R->known[k]) continue;
                    float d = route_creek_near(R, x, z).dist, w = 1.0f - smooth01((d - 62.0f) / 25.0f);
                    R->dem[k] += (v - R->elev0 - R->dem[k]) * w; merged++;
                }
                fprintf(stderr, "route: the creek way, %.0f m, %d chunks; its DEM in %d grid points\n", R->ck_len, R->ck_nch, merged);
            }
            free(cd);
        }
        if (f) fclose(f);
        if (R->nchunks + R->ck_nch > RT_MAX_CHUNKS) { fprintf(stderr, "route: too many chunks with the creek way: it's left out\n"); R->ck_n = 0; R->ck_nch = 0; }
    }
    if (!pad_h_given) R->pad_h = 0;
    if (R->bridge_s1 <= R->bridge_s0) R->bridge_s0 = R->bridge_s1 = 0;
    if (meta < 2) { R->start_x = R->x[3]; R->start_z = R->z[3]; R->start_yaw = (float)(M_PI / 2) - R->head[3]; }
    if (T.start_brg != 0 && (R->hut_x != 0 || R->hut_z != 0)) {   /* session 3b: parked on the patch facing the hut, as in the car-park photo */
        R->start_x = R->hut_x + T.start_e; R->start_z = R->hut_z + T.start_n; R->start_yaw = T.start_brg * (float)(M_PI / 180.0);
    }
    R->nchunks = (R->n - 1 + RT_CHUNK - 1) / RT_CHUNK;
    if (R->nchunks > RT_MAX_CHUNKS) { fprintf(stderr, "route: too long (%d chunks)\n", R->nchunks); return 0; }
    if (R->nchunks + R->ck_nch > RT_MAX_CHUNKS) { fprintf(stderr, "route: too many chunks with the creek way: it's left out\n"); R->ck_n = 0; R->ck_nch = 0; }
    build_buckets(R);
    /* the ford's water: where the dipped road lies below the level */
    if (R->ford_s > 0) {
        float lo = 1e9f;
        for (float s = R->ford_s - 10; s <= R->ford_s + 10; s += 0.5f) lo = fminf(lo, route_road_h(R, s));
        R->ford_water = lo + 0.2f;
        R->ford_lo = R->ford_hi = R->ford_s;
        while (R->ford_lo > R->ford_s - 30 && route_road_h(R, R->ford_lo) < R->ford_water) R->ford_lo -= 0.5f;
        while (R->ford_hi < R->ford_s + 30 && route_road_h(R, R->ford_hi) < R->ford_water) R->ford_hi += 0.5f;
    }
    build_spur(R);
    build_hollow(R, farcov);
    free(farcov);
    if (R->ck_n > 1) {
        creek_features(R);
        /* the bed no steeper than 22 % (0.44 m in 2 m) but at the waterfall: cut down into the ground above each steeper drop */
        int f0 = (int)((R->ck_fall_s - 10.0f) / RT_DS), f1 = (int)((R->ck_fall_s + 10.0f) / RT_DS), eased = 0;
        for (int i = R->ck_n - 2; i >= 0; i--) {
            if (i >= f0 && i <= f1) continue;
            if (R->ck_bed[i] > R->ck_bed[i + 1] + 0.44f) { R->ck_bed[i] = R->ck_bed[i + 1] + 0.44f; eased++; }
        }
        if (eased) fprintf(stderr, "route: the creek bed eased at %d points (no steeper than 22%% but at the waterfall)\n", eased);
    }
    assign_cells(R);
    place_trees(R);
    route_place_stags(R);   /* build D */
    R->loaded = 1;
    return 1;
}

void route_replace_trees(Route *R) { free(R->tree); R->tree = NULL; place_trees(R); }

/* Build D: the stags. 25 to 40 standing dead gums in Explorer's world: along the road strip (14 to 30 m out, mostly not by
 * the road), round the clearing's edge, and on the creek way's banks. Each gets the way it falls, chosen so it never lands
 * across the road or the creek's line, with its three logs lying where it lands, all on open, drivable ground. */
static int stag_open(const Route *R, float x, float z, float room, int creek_ok)
{
    if (route_tree_hit(R, x, z, 0, room, room, -room) >= 0) return 0;
    RtNear n = route_near(R, x, z);
    if (fabsf(n.lat) < route_half_width() + T.verge_w + T.ditch_w + 3.0f && n.s > -5.0f && n.s < R->length + 5.0f) return 0;   /* never on or by the road */
    if (route_edge(R, n, x, z) > T.edge_stop - 3.0f) return 0;
    RtNear c = route_creek_near(R, x, z);
    if (c.dist < route_creek_bedw(R, c.s) + 1.5f && !creek_ok) return 0;   /* not in the creek's bed */
    if (c.dist < 30.0f && c.s > R->ck_exit_s - 40.0f && c.s < R->ck_exit_s + 60.0f) return 0;   /* well away from the way in */
    float gx = route_height(R, x + 1.0f, z, NULL) - route_height(R, x - 1.0f, z, NULL), gz = route_height(R, x, z + 1.0f, NULL) - route_height(R, x, z - 1.0f, NULL);
    return hypotf(gx, gz) * 0.5f < 0.3f;
}
static int stag_try(Route *R, float x, float z, const float *dirs, int ndir, int where, uint32_t *sd)
{
    if (R->nstag >= 48 || !stag_open(R, x, z, 2.2f, 0)) return 0;
    for (int k = 0; k < R->nstag; k++) if (hypotf(R->stag[k].x - x, R->stag[k].z - z) < 40.0f) return 0;
    if (R->camp_on && hypotf(R->camp_x - x, R->camp_z - z) < 30.0f) return 0;
    for (int m = 0; m < R->ck_nmark; m++) if (hypotf(R->ck_mark_x[m] - x, R->ck_mark_z[m] - z) < 15.0f) return 0;
    for (int d = 0; d < ndir; d++) {
        float a = dirs[d], ok = 1;
        for (int q = 0; q < 3 && ok; q++) {
            float l = 2.8f + 3.1f * (float)q, lx = x + cosf(a) * l, lz = z + sinf(a) * l;
            if (!stag_open(R, lx, lz, 1.0f, 0)) ok = 0;
            if (where == 2) { RtNear c = route_creek_near(R, lx, lz); if (c.dist < route_creek_bedw(R, c.s) + 1.0f || creek_chan_e(R, c.s, c.dist) > T.edge_stop - 2.5f) ok = 0; }   /* on the bank, short of the hold */
        }
        if (!ok) continue;
        struct RtStag *g = &R->stag[R->nstag++];
        g->x = x; g->z = z; g->fall = a; g->variant = (int)(rnd(sd) * 3.0f) % 3; g->where = where;
        g->h = (7.0f + 2.5f * (float)g->variant) * (0.9f + 0.2f * rnd(sd)); g->r = 0.30f + 0.06f * (float)g->variant;
        g->log_yaw = a + 0.12f * (rnd(sd) - 0.5f);
        for (int q = 0; q < 3; q++) { float l = 2.8f + 3.1f * (float)q; g->log_x[q] = x + cosf(a) * l + 0.3f * (rnd(sd) - 0.5f); g->log_z[q] = z + sinf(a) * l + 0.3f * (rnd(sd) - 0.5f); }
        return 1;
    }
    return 0;
}
void route_place_stags(Route *R)
{
    R->nstag = 0;
    uint32_t sd = 4848u;
    int keep = R->explore; R->explore = 1;   /* Explorer's world (the creek way open) */
    /* the road strip: one in each 300 m or so, alternating sides, 14 to 30 m out; falling away from the road, or along it */
    for (float s0 = 160.0f; s0 < R->length - 150.0f && R->nstag < 26; s0 += 290.0f) {
        if (R->bridge_s0 > 0 && s0 > R->bridge_s0 - 60.0f) break;
        for (int t = 0; t < 6; t++) {
            float s1 = s0 + 40.0f * (float)t, side = ((int)(s0 / 290.0f) + t) & 1 ? 1.0f : -1.0f, lat = side * (14.0f + 16.0f * rnd(&sd)), x, z;
            if (fabsf(s1 - R->ford_s) < 60.0f || (R->junction_s > 0 && fabsf(s1 - R->junction_s) < 60.0f)) continue;
            float hd = route_heading_at(R, s1); route_point(R, s1, &x, &z);
            x -= sinf(hd) * lat; z += cosf(hd) * lat;
            if (route_in_clearing(R, x, z)) continue;
            float out = hd + (side > 0 ? (float)M_PI * 0.5f : -(float)M_PI * 0.5f);   /* away from the road (lat + is left of the way) */
            float dirs[5] = { out, out + 0.5f, out - 0.5f, hd, hd + (float)M_PI };
            if (stag_try(R, x, z, dirs, 5, 0, &sd)) break;
        }
    }
    /* round the clearing's edge: just into the bush past the open hollow, falling into the bush (or along the edge) */
    for (int k = 0; k < 32 && R->nstag < 33; k++) {
        float a = (float)k * 0.196f + 0.3f;
        for (float rr = 40.0f; rr < 300.0f; rr += 3.0f) {
            float x = R->hut_x + cosf(a) * rr, z = R->hut_z + sinf(a) * rr, hdist = hut_dist(R, x, z);
            if (hdist < 1.0f) continue;
            if (hdist > 15.0f) break;
            float dirs[4] = { a, a + 0.6f, a - 0.6f, a + 1.57f };
            if (stag_try(R, x, z, dirs, 4, 1, &sd)) break;
        }
    }
    /* the creek way's banks: falling downstream along the bank, never across the bed */
    for (float s0 = R->ck_exit_s + 420.0f; s0 < R->ck_len - 300.0f && R->nstag < 40; s0 += 640.0f) {
        for (int t = 0; t < 5; t++) {
            float s1 = s0 + 30.0f * (float)t; if (creek_busy(R, s1, 50.0f)) continue;
            float BW = route_creek_bedw(R, s1), side = t & 1 ? 1.0f : -1.0f, lat = side * (BW + 4.5f), hd = route_heading_at_creek(R, s1), x, z;
            route_creek_point(R, s1, &x, &z); x -= sinf(hd) * lat; z += cosf(hd) * lat;
            float dirs[3] = { hd + 0.15f * side, hd - (float)M_PI - 0.15f * side, hd };
            if (stag_try(R, x, z, dirs, 3, 2, &sd)) break;
        }
    }
    R->explore = keep;
    int nw[3] = { 0, 0, 0 }; for (int k = 0; k < R->nstag; k++) nw[R->stag[k].where]++;
    fprintf(stderr, "route: %d stags (road strip %d, clearing edge %d, creek way %d), %d firewood logs in them\n", R->nstag, nw[0], nw[1], nw[2], 3 * R->nstag);
}

/* Logs that break (follow-up): a log is its drawn tree followed by its hidden posts (kind 5). Hitting a post breaks the
 * log: it stops being drawn and its posts stop being solid (kind 6) until route_logs_restore. Returns the log's chunk. */
static int broken[4096], nbroken;
static int chunk_of(const Route *R, int k) { int c = 0, nt = R->nchunks + R->ck_nch; while (c < nt - 1 && R->tree_first[c + 1] <= k) c++; return c; }
int route_log_break(Route *R, int post)
{
    if (post < 0 || post >= R->ntree || R->tree[post].kind != 5) return -1;
    int k = post;
    while (k > 0 && R->tree[k].kind == 5) k--;
    RtTree *log = &R->tree[k];
    if (log->t0 < 0 || strcmp(route_template(log->t0)->name, "log")) return -1;
    if (nbroken < 4096) broken[nbroken++] = k;
    log->seed = (uint32_t)log->t0 | ((uint32_t)(unsigned short)log->t1 << 16);   /* kept to put it back */
    log->t0 = log->t1 = -1;
    for (int q = k + 1; q < R->ntree && R->tree[q].kind == 5; q++) R->tree[q].kind = 6;
    return chunk_of(R, k);
}
int route_log_index(const Route *R, int post)
{
    if (post < 0 || post >= R->ntree) return -1;
    int k = post;
    while (k > 0 && (R->tree[k].kind == 5 || R->tree[k].kind == 6)) k--;
    return k;
}
int route_log_break_at(Route *R, float x, float z)
{
    for (int k = 0; k + 1 < R->ntree; k++) {
        const RtTree *t = &R->tree[k];
        if (R->tree[k + 1].kind != 5 || fabsf(t->x - x) > 0.3f || fabsf(t->z - z) > 0.3f || t->t0 < 0) continue;
        return route_log_break(R, k + 1);
    }
    return -1;
}
void route_logs_restore(Route *R, void (*changed)(int chunk))
{
    for (int i = 0; i < nbroken; i++) {
        RtTree *log = &R->tree[broken[i]];
        log->t0 = (short)(log->seed & 0xffff); log->t1 = (short)(log->seed >> 16);
        for (int q = broken[i] + 1; q < R->ntree && R->tree[q].kind == 6; q++) R->tree[q].kind = 5;
        if (changed) changed(chunk_of(R, broken[i]));
    }
    nbroken = 0;
}

/* ------------------------------------------------------------- geometry */

typedef struct { float r, g, b; } Col;
static Col col(float r, float g, float b) { Col c = { r, g, b }; return c; }
static Col cmul(Col a, float k) { return col(a.r * k, a.g * k, a.b * k); }
static Col cmix(Col a, Col b, float t) { return col(a.r + (b.r - a.r) * t, a.g + (b.g - a.g) * t, a.b + (b.b - a.b) * t); }

/* Golden hour: a low warm sun from the west-north-west, a cool lilac sky light. */
static V3 sun_dir(void) { return vnorm(v3(-0.72f, 0.55f, 0.42f)); } /* session 3: 33 degrees up (was 25): shadows 1.5 x a tree's height */
void route_sun_dir(float *x, float *y, float *z) { V3 s = sun_dir(); *x = s.x; *y = s.y; *z = s.z; }
static const Col AMB = { 0.50f, 0.47f, 0.62f }, SUN = { 1.00f, 0.80f, 0.55f };

/* The builder: one part at a time, a default uv, and the light baked per triangle. */
typedef struct { RtGeom *g; int part; const Route *R; float u, v; int sunlit; } GB;
static void gb_vert(GB *b, V3 p, Col c, float sun_share, int bit, float u, float v)
{
    RtPart *P = &b->g->part[b->part];
    if (P->nverts == P->cap) {
        P->cap = P->cap ? P->cap * 2 : 4096;
        P->v = realloc(P->v, sizeof(float) * 3 * (size_t)P->cap);
        P->t = realloc(P->t, sizeof(float) * 2 * (size_t)P->cap);
        P->c = realloc(P->c, 4 * (size_t)P->cap);
    }
    float *o = P->v + (size_t)P->nverts * 3;
    o[0] = p.x - b->g->base_x; o[1] = p.y; o[2] = -(p.z - b->g->base_z); /* drawing axes mirror z */
    P->t[P->nverts * 2] = u; P->t[P->nverts * 2 + 1] = v;
    unsigned char *q = P->c + (size_t)P->nverts * 4;
    q[0] = (unsigned char)(clamp01(c.r) * 255.0f); q[1] = (unsigned char)(clamp01(c.g) * 255.0f); q[2] = (unsigned char)(clamp01(c.b) * 255.0f);
    q[3] = (unsigned char)((int)(clamp01(sun_share) * 127.0f + 0.5f) * 2 + (bit ? 1 : 0));
    P->nverts++;
}
/* Flat-lit triangle. wrap > 0 softens the light round the back (leaves); the sun's share goes in the alpha. */
static float dapple(const Route *R, float x, float z);
static void gb_tri_ex(GB *b, V3 p0, V3 p1, V3 p2, Col base, V3 n, float wrap, const float *uv)
{
    float d = vdot(vnorm(n), sun_dir());
    d = wrap > 0 ? fmaxf((d + wrap) / (1.0f + wrap), 0.0f) : fmaxf(d, 0.0f);
    Col c = col(base.r * (AMB.r + SUN.r * d), base.g * (AMB.g + SUN.g * d), base.b * (AMB.b + SUN.b * d));
    float share = (0.84f * d) / (0.53f + 0.84f * d + 1e-4f);
    int bit = b->sunlit >= 0 ? b->sunlit : dapple(b->R, (p0.x + p1.x + p2.x) / 3.0f, (p0.z + p1.z + p2.z) / 3.0f) > 0.5f;
    gb_vert(b, p0, c, share, bit, uv ? uv[0] : b->u, uv ? uv[1] : b->v);
    gb_vert(b, p1, c, share, bit, uv ? uv[2] : b->u, uv ? uv[3] : b->v);
    gb_vert(b, p2, c, share, bit, uv ? uv[4] : b->u, uv ? uv[5] : b->v);
}
/* up = 1 turns its normal to face upwards (ground), else away from `out` (a centre). */
static void gb_tri(GB *b, V3 p0, V3 p1, V3 p2, Col base, int up, V3 out)
{
    V3 n = vcross(vsub(p1, p0), vsub(p2, p0));
    if (up) { if (n.y < 0) n = vscale(n, -1); }
    else { V3 m = vscale(vadd(vadd(p0, p1), p2), 1.0f / 3.0f); if (vdot(n, vsub(m, out)) < 0) n = vscale(n, -1); }
    gb_tri_ex(b, p0, p1, p2, base, n, 0, NULL);
}
static void gb_quad_up(GB *b, V3 p0, V3 p1, V3 p2, V3 p3, Col base)
{
    gb_tri(b, p0, p1, p2, base, 1, p0);
    gb_tri(b, p0, p2, p3, base, 1, p0);
}
static void gb_quad_out(GB *b, V3 p0, V3 p1, V3 p2, V3 p3, Col base, V3 centre)
{
    gb_tri(b, p0, p1, p2, base, 0, centre);
    gb_tri(b, p0, p2, p3, base, 0, centre);
}
/* A tapering prism (bridge posts). */
static void gb_prism(GB *b, V3 a, V3 top, float r0, float r1, int sides, Col base)
{
    V3 axis = vnorm(vsub(top, a));
    V3 u = vnorm(vcross(axis, fabsf(axis.y) < 0.9f ? v3(0, 1, 0) : v3(1, 0, 0))), w = vcross(axis, u);
    for (int k = 0; k < sides; k++) {
        float a0 = (float)k * 6.2831853f / (float)sides, a1 = (float)(k + 1) * 6.2831853f / (float)sides;
        V3 d0 = vadd(vscale(u, cosf(a0)), vscale(w, sinf(a0))), d1 = vadd(vscale(u, cosf(a1)), vscale(w, sinf(a1)));
        gb_quad_out(b, vadd(a, vscale(d0, r0)), vadd(a, vscale(d1, r0)), vadd(top, vscale(d1, r1)), vadd(top, vscale(d0, r1)), base, vlerp(a, top, 0.5f));
    }
}

static const Col C_CROWN = { 0.62f, 0.50f, 0.36f }, C_TRACK = { 0.80f, 0.68f, 0.50f }, C_ROADSIDE = { 0.66f, 0.54f, 0.38f };
static const Col C_VERGE = { 0.55f, 0.56f, 0.28f }, C_DITCH = { 0.36f, 0.38f, 0.20f }, C_FLOOR = { 0.36f, 0.33f, 0.22f };
static const Col C_GRASS = { 0.62f, 0.62f, 0.30f }, C_PAD = { 0.70f, 0.58f, 0.42f };

/* Dappled light beyond the shadow map: sun or shade, by a noise pattern that is denser where the canopy is. */
static float dapple(const Route *R, float x, float z)
{
    float n = 0.65f * vnoise(x / 3.2f, z / 3.2f) + 0.35f * vnoise(x / 1.3f + 17.0f, z / 1.3f - 5.0f);
    float shade_share = 0.15f + 0.6f * canopy_at(R, x, z);
    return n < shade_share ? 0.0f : 1.0f;
}

static Col ground_col(const Route *R, float x, float z, float slope)
{
    float cov = canopy_at(R, x, z), nz = vnoise(x / 9.0f, z / 9.0f);
    Col c = cmix(C_GRASS, C_FLOOR, smooth01(cov * 1.6f - 0.5f));
    c = cmix(c, col(0.45f, 0.48f, 0.25f), 0.35f * nz);
    if (slope > 0.5f) c = cmix(c, col(0.42f, 0.34f, 0.26f), clamp01((slope - 0.5f) * 1.5f)); /* steep banks: bare earth */
    if (R->hut_x != 0 || R->hut_z != 0) {   /* session 3b: the mown lawn round the hut (the NPWS photos) */
        float dh = hypotf(x - R->hut_x, z - R->hut_z);
        c = cmix(c, cmix(col(0.47f, 0.56f, 0.25f), col(0.42f, 0.50f, 0.22f), nz), 0.7f * (1.0f - smooth01((dh - 28.0f) / 18.0f)));
    }
    float dp = hypotf(x - R->pad_x, z - R->pad_z);
    if (R->pad_r > 0 && dp < R->pad_r + 2.0f) c = cmix(C_PAD, c, smooth01((dp - R->pad_r + 3.0f) / 5.0f));
    if (R->ck_n > 1) {   /* build C: the creek's gravel bed and its damp, mossy banks */
        RtNear cn = route_creek_near(R, x, z);
        if (cn.dist < 20.0f && road_dist(R, route_near(R, x, z)) > 32.0f) {
            float BW = route_creek_bedw(R, cn.s);
            Col gravel = cmix(col(0.56f, 0.53f, 0.46f), col(0.46f, 0.44f, 0.40f), nz), damp = cmix(col(0.30f, 0.36f, 0.20f), col(0.36f, 0.40f, 0.22f), nz);
            c = cmix(cmix(gravel, damp, smooth01((cn.dist - BW + 1.0f) / 2.5f)), c, smooth01((cn.dist - BW - 4.0f) / 6.0f));
        }
    }
    return c;
}

static void build_ribbon(const Route *R, int index, GB *b)
{
    float W = route_half_width(), e1 = W + T.verge_w, e2 = e1 + T.ditch_w, ro = e2 + 4.0f;
    float st[16]; int ns = 0;
    st[ns++] = 0; st[ns++] = 0.45f; st[ns++] = 1.15f;
    if (W > 2.2f) st[ns++] = 0.5f * (W + 1.15f);
    st[ns++] = W; st[ns++] = e1; st[ns++] = e1 + 0.5f * T.ditch_w; st[ns++] = e2; st[ns++] = e2 + 1.8f; st[ns++] = ro;
    int i0 = index * RT_CHUNK, i1 = i0 + RT_CHUNK; if (i1 > R->n - 1) i1 = R->n - 1;
    int nsides = 2 * ns - 1;
    V3 prev[32]; float plat[32]; int have_prev = 0;
    for (int i = i0; i <= i1; i++) {
        float s = (float)i * RT_DS;
        float nx = -sinf(R->head[i]), nz = cosf(R->head[i]); /* to the left */
        float inner = R->curv[i] != 0 ? 0.85f / fabsf(R->curv[i]) : 1e9f;
        V3 row[32]; float lrow[32];
        for (int k = 0; k < nsides; k++) {
            int side = k < ns - 1 ? -1 : 1, si = side < 0 ? ns - 1 - k : k - (ns - 1);
            float a = st[si];
            if ((side > 0) == (R->curv[i] > 0) && a > inner) a = inner; /* the inside of a tight bend: don't fold over */
            float lat = (float)side * a, x = R->x[i] + nx * lat, z = R->z[i] + nz * lat;
            RtNear n = { s, lat, fabsf(lat), i };
            row[k] = v3(x, profile(R, n, x, z, NULL), z);
            lrow[k] = lat;
        }
        if (have_prev && !(in_bridge(R, s) && in_bridge(R, s - RT_DS))) {
            for (int k = 0; k < nsides - 1; k++) {
#define SI(k) ((k) < ns - 1 ? ns - 1 - (k) : (k) - (ns - 1)) /* row position to station */
                float amid = 0.5f * (st[SI(k)] + st[SI(k + 1)]);
#undef SI
                Col c;
                if (amid < 0.45f) c = C_CROWN;
                else if (amid < 1.15f) c = C_TRACK;
                else if (amid < W) c = C_ROADSIDE;
                else if (amid < e1) c = C_VERGE;
                else if (amid < e2) c = C_DITCH;
                else c = cmix(C_DITCH, ground_col(R, row[k].x, row[k].z, 0.3f), clamp01((amid - e2) / 4.0f));
                float mx = 0.25f * (row[k].x + row[k + 1].x + prev[k].x + prev[k + 1].x), mz = 0.25f * (row[k].z + row[k + 1].z + prev[k].z + prev[k + 1].z);
                if (amid < W) { /* gravel speckle, and the odd grassy patch on the outside */
                    float g = vnoise(mx / 1.7f, mz / 1.7f);
                    c = cmul(c, 0.92f + 0.16f * g);
                    if (amid >= 1.15f && vnoise(mx / 4.0f + 50, mz / 4.0f) > 0.72f) c = cmix(c, C_VERGE, 0.6f);
                }
                /* road surfaces carry (s, 100 + lat) for the drawn detail; the rest reads as terrain */
                float road = amid < W ? 100.0f : 1.0f, sp = s - RT_DS;
                float uv0[6] = { sp, road > 50 ? 100 + plat[k] : 1, sp, road > 50 ? 100 + plat[k + 1] : 1, s, road > 50 ? 100 + lrow[k + 1] : 1 };
                float uv1[6] = { sp, road > 50 ? 100 + plat[k] : 1, s, road > 50 ? 100 + lrow[k + 1] : 1, s, road > 50 ? 100 + lrow[k] : 1 };
                if (road < 50) { uv0[0] = uv0[2] = uv0[4] = uv1[0] = uv1[2] = uv1[4] = 0; }
                V3 n0 = vcross(vsub(prev[k + 1], prev[k]), vsub(row[k + 1], prev[k])); if (n0.y < 0) n0 = vscale(n0, -1);
                V3 n1 = vcross(vsub(row[k + 1], prev[k]), vsub(row[k], prev[k])); if (n1.y < 0) n1 = vscale(n1, -1);
                gb_tri_ex(b, prev[k], prev[k + 1], row[k + 1], c, n0, 0, uv0);
                gb_tri_ex(b, prev[k], row[k + 1], row[k], c, n1, 0, uv1);
                b->g->ntri_road += 2;
            }
        }
        memcpy(prev, row, sizeof(V3) * (size_t)nsides);
        memcpy(plat, lrow, sizeof(float) * (size_t)nsides);
        have_prev = 1;
    }
}

static float terrain_vertex_h(const Route *R, float x, float z, float ro)
{
    RtNear n = route_near(R, x, z);
    float h = profile(R, n, x, z, NULL);
    if (in_bridge(R, n.s) || n.s < -2.0f || n.s > R->length + 2.0f) return h;
    float dp = hypotf(x - R->pad_x, z - R->pad_z);
    if (R->pad_r > 0 && dp < R->pad_r + 2.0f && fabsf(n.lat) > route_half_width()) return h;
    if (fabsf(n.lat) < ro - 0.5f) h -= 0.6f; /* under the road ribbon: kept out of sight */
    else { float t2, d2 = route_spur_dist(R, x, z, &t2); if ((d2 < route_half_width() + T.verge_w - 0.3f && t2 < SPUR_LEN - 6.0f) || (R->spur_n > 2 && hypotf(x - R->spur_x[2], z - R->spur_z[2]) < JUNCTION_R - 0.3f)) h -= 0.4f; }   /* and under the side road's */
    return h;
}

static void build_terrain(const Route *R, int index, GB *b)
{
    float W = route_half_width(), ro = W + T.verge_w + T.ditch_w + 4.0f;
    int ncx = R->gx - 1;
    b->u = 0; b->v = 1;
    for (int q = R->cell_first[index]; q < R->cell_first[index + 1]; q++) {
        int k = R->cell_list[q], i = k % ncx, j = k / ncx;
        float x0 = R->gx0 + (float)i * R->cell, z0 = R->gz0 + (float)j * R->cell, x1 = x0 + R->cell, z1 = z0 + R->cell;
        RtNear nc = route_near(R, 0.5f * (x0 + x1), 0.5f * (z0 + z1));
        float dp = hypotf(0.5f * (x0 + x1) - R->pad_x, 0.5f * (z0 + z1) - R->pad_z);
        int pad = R->pad_r > 0 && dp < R->pad_r + 4.0f;
        if (!pad && !in_bridge(R, nc.s) && nc.s > 0 && nc.s < R->length && fabsf(nc.lat) + R->cell * 0.71f < ro - 1.0f) continue; /* all under the ribbon */
        V3 p00 = v3(x0, terrain_vertex_h(R, x0, z0, ro), z0), p10 = v3(x1, terrain_vertex_h(R, x1, z0, ro), z0);
        V3 p01 = v3(x0, terrain_vertex_h(R, x0, z1, ro), z1), p11 = v3(x1, terrain_vertex_h(R, x1, z1, ro), z1);
        float slope = fmaxf(fabsf(p10.y - p00.y), fabsf(p01.y - p00.y)) / R->cell;
        Col c = ground_col(R, 0.5f * (x0 + x1), 0.5f * (z0 + z1), slope);
        b->sunlit = pad || canopy_at(R, x0, z0) < 0.3f ? 1 : -1;
        gb_tri(b, p00, p10, p11, c, 1, p00);
        gb_tri(b, p00, p11, p01, cmul(c, 0.97f), 1, p00);
        b->g->ntri_terrain += 2;
    }
    b->sunlit = -1; b->u = 0; b->v = 0;
}

/* One copy of a template: turned, scaled, set on the ground, its light baked. Crowns high in the tree count as in the
 * sun for the baked dapple; shadows near the car come from the shadow map. */
static void put_template(GB *b, const RtTemplate *t, float x, float gy, float z, float yaw, float scale)
{
    float cy = cosf(yaw), sy = sinf(yaw);
    for (int k = 0; k < t->ntri; k++) {
        const float *q = t->p + 9 * k;
        V3 p[3];
        for (int j = 0; j < 3; j++) {
            float px = q[3 * j] * scale, py = q[3 * j + 1] * scale, pz = q[3 * j + 2] * scale;
            p[j] = v3(x + px * cy - pz * sy, gy + py, z + px * sy + pz * cy);
        }
        V3 n = vcross(vsub(p[1], p[0]), vsub(p[2], p[0]));
        char kd = t->kind[k];
        float wrap = kd == 'l' ? 0.6f : kd == 'g' ? 0.4f : 0.0f;
        if (kd != 'b' || 1) { /* two-sided: face the sun if it can */
            V3 sd = sun_dir();
            if (vdot(n, sd) < 0 && kd != 'b') n = vscale(n, -1);
            else if (kd == 'b') { V3 m = vscale(vadd(vadd(p[0], p[1]), p[2]), 1.0f / 3.0f); V3 out = vsub(m, v3(x, m.y, z)); if (vdot(n, out) < 0) n = vscale(n, -1); }
        }
        Col c = col(t->rgb[3 * k], t->rgb[3 * k + 1], t->rgb[3 * k + 2]);
        float hy = (p[0].y + p[1].y + p[2].y) / 3.0f - gy;
        b->sunlit = hy > 0.72f * t->h * scale && t->h > 6.0f ? 1 : -1;
        gb_tri_ex(b, p[0], p[1], p[2], c, n, wrap, NULL);
    }
    b->sunlit = -1;
}

static void build_trees(const Route *R, int index, GB *b)
{
    for (int k = R->tree_first[index]; k < R->tree_first[index + 1]; k++) {
        const RtTree *t = &R->tree[k];
        if (t->t0 < 0) continue;
        if (R->explore && !t->wood && !strcmp(tpl[t->t0].name, "log")) continue;   /* build C: Explorer has only the firewood */
        float gy = route_height(R, t->x, t->z, NULL);
        const RtTemplate *t0 = &tpl[t->t0];
        int prop = t->lat > 98.0f;   /* the hut's props are at 99 (plants are capped at 97) */
        int is_log = !strcmp(t0->name, "log");
        float y = gy - (is_log ? 0.15f : 0.0f);
        int nt0 = 0;
        if (prop) { b->part = RT_PART_BACK; put_template(b, t0, t->x, y, t->z, t->yaw, t->scale); nt0 = t0->ntri; b->g->ntri_other += nt0; continue; }
        if (t->lat < RT_NEAR_LAT) {
            b->part = RT_PART_NEAR; put_template(b, t0, t->x, y, t->z, t->yaw, t->scale); nt0 = t0->ntri;
            if (t->t1 >= 0) { b->part = RT_PART_NEARLO; put_template(b, &tpl[t->t1], t->x, y, t->z, t->yaw, t->scale); }
        } else if (t->t1 >= 0) {
            b->part = t->lat >= 40.0f ? RT_PART_EDGE : RT_PART_BACK; put_template(b, &tpl[t->t1], t->x, y, t->z, t->yaw, t->scale); nt0 = tpl[t->t1].ntri;
        }
        if (t->kind == 0) b->g->ntri_trees += nt0; else b->g->ntri_scrub += nt0;
    }
    b->part = RT_PART_GROUND;
}

/* A water sheet across the road between s0 and s1, out to `half` either side, at height y. */
static void build_water(const Route *R, float s0, float s1, float half, float y, GB *b)
{
    const Col water = { 0.40f, 0.52f, 0.58f };
    b->sunlit = 1;
    for (float s = s0; s < s1; s += RT_DS) {
        float sa = s, sb = fminf(s + RT_DS, s1), xa, za, xb, zb;
        route_point(R, sa, &xa, &za); route_point(R, sb, &xb, &zb);
        float ha = route_heading_at(R, sa), hb = route_heading_at(R, sb);
        V3 p0 = v3(xa - sinf(ha) * -half, y, za + cosf(ha) * -half), p1 = v3(xa - sinf(ha) * half, y, za + cosf(ha) * half);
        V3 p2 = v3(xb - sinf(hb) * half, y, zb + cosf(hb) * half), p3 = v3(xb - sinf(hb) * -half, y, zb + cosf(hb) * -half);
        float glint = vnoise(xa / 2.0f, za / 2.0f) > 0.6f ? 1.25f : 1.0f;
        gb_quad_up(b, p0, p1, p2, p3, cmul(water, glint));
        b->g->ntri_other += 2;
    }
    b->sunlit = -1;
}

static void build_bridge(const Route *R, GB *b)
{
    float W = route_half_width();
    const Col deck = { 0.56f, 0.46f, 0.34f }, side = { 0.42f, 0.36f, 0.30f }, rail = { 0.74f, 0.70f, 0.62f };
    float s0 = R->bridge_s0, s1 = R->bridge_s1;
    /* the creek under it */
    float lo = 1e9f;
    for (float s = s0; s <= s1; s += 1.0f) { float x, z; route_point(R, s, &x, &z); lo = fminf(lo, route_dem(R, x, z)); }
    build_water(R, s0 - 6.0f, s1 + 6.0f, 30.0f, lo + 0.5f, b);
    for (float s = s0; s < s1 - 0.01f; s += 1.0f) {
        float sb = fminf(s + 1.0f, s1), xa, za, xb, zb;
        route_point(R, s, &xa, &za); route_point(R, sb, &xb, &zb);
        float ha = route_heading_at(R, s), hb = route_heading_at(R, sb), ya = route_road_h(R, s), yb = route_road_h(R, sb);
        float lxa = -sinf(ha), lza = cosf(ha), lxb = -sinf(hb), lzb = cosf(hb);
        V3 a0 = v3(xa - lxa * W, ya, za - lza * W), a1 = v3(xa + lxa * W, ya, za + lza * W);
        V3 b1 = v3(xb + lxb * W, yb, zb + lzb * W), b0 = v3(xb - lxb * W, yb, zb - lzb * W);
        gb_quad_up(b, a0, a1, b1, b0, ((int)(s - s0) & 1) ? deck : cmul(deck, 0.9f));
        V3 dn = v3(0, -0.8f, 0), mid = v3(0.5f * (xa + xb), 0.5f * (ya + yb) - 0.4f, 0.5f * (za + zb));
        gb_quad_out(b, a0, b0, vadd(b0, dn), vadd(a0, dn), side, mid);
        gb_quad_out(b, a1, b1, vadd(b1, dn), vadd(a1, dn), side, mid);
        V3 up = v3(0, 1.0f, 0), up2 = v3(0, 0.85f, 0);
        for (int e = 0; e < 2; e++) { /* rails: a top rail, and a post every 2 m */
            V3 pa = e ? a1 : a0, pb = e ? b1 : b0;
            gb_quad_out(b, vadd(pa, up2), vadd(pb, up2), vadd(pb, up), vadd(pa, up), rail, mid);
            if (((int)(s - s0) & 1) == 0) gb_prism(b, pa, vadd(pa, up), 0.08f, 0.08f, 4, rail);
        }
        b->g->ntri_other += 2 + 4 + 4 + 8;
    }
}

/* The side road's gravel and verges (the ditches and banks come from the terrain), from where it leaves the road's
 * edge to where it fades into the bush; a hair above the ground under it. */
static void build_spur_ribbon(const Route *R, GB *b)
{
    float W = route_half_width(), e1 = W + T.verge_w;
    const float st[] = { -e1, -W, -0.5f * (W + 1.15f), -1.15f, -0.45f, 0, 0.45f, 1.15f, 0.5f * (W + 1.15f), W, e1 };
    const int ns = (int)(sizeof(st) / sizeof(st[0]));
    V3 prev[16]; int have = 0;
    for (int i = 0; i < R->spur_n && 2.0f * (float)i < SPUR_LEN - 5.0f; i++) {
        int a = i > 0 ? i - 1 : 0, c = i + 1 < R->spur_n ? i + 1 : i;
        float hd = atan2f(R->spur_z[c] - R->spur_z[a], R->spur_x[c] - R->spur_x[a]), nx = -sinf(hd), nz = cosf(hd);
        V3 row[16];
        for (int k = 0; k < ns; k++) {
            float x = R->spur_x[i] + nx * st[k], z = R->spur_z[i] + nz * st[k];
            row[k] = v3(x, route_height(R, x, z, NULL) + 0.02f, z);
        }
        if (have)
            for (int k = 0; k + 1 < ns; k++) {
                float amid = fabsf(0.5f * (st[k] + st[k + 1]));
                float mx = 0.25f * (row[k].x + row[k + 1].x + prev[k].x + prev[k + 1].x), mz = 0.25f * (row[k].z + row[k + 1].z + prev[k].z + prev[k + 1].z);
                RtNear mn = route_near(R, mx, mz);
                if (fabsf(mn.lat) < W + 0.3f) continue;   /* over the main road: it shows through */
                Col col2 = amid < 0.45f ? C_CROWN : amid < 1.15f ? C_TRACK : amid < W ? C_ROADSIDE : C_VERGE;
                if (amid < W) col2 = cmul(col2, 0.92f + 0.16f * vnoise(mx / 1.7f, mz / 1.7f));
                gb_tri(b, prev[k], prev[k + 1], row[k + 1], col2, 1, prev[k]);
                gb_tri(b, prev[k], row[k + 1], row[k], col2, 1, prev[k]);
                b->g->ntri_road += 2;
            }
        memcpy(prev, row, sizeof(V3) * (size_t)ns); have = 1;
    }
    /* the splay: a disc of gravel round the mouth, rings out to JUNCTION_R and a grass edge */
    float cx = R->spur_x[2], cz = R->spur_z[2];
    const float rr[] = { 0, 2.5f, 5.0f, JUNCTION_R, JUNCTION_R + 1.0f };
    for (int ri = 0; ri < 4; ri++)
        for (int k = 0; k < 24; k++) {
            float a0 = (float)k * 6.2831853f / 24.0f, a1 = (float)(k + 1) * 6.2831853f / 24.0f;
            V3 q[4]; float r0 = rr[ri], r1 = rr[ri + 1];
            float px4[4] = { cx + cosf(a0) * r0, cx + cosf(a1) * r0, cx + cosf(a1) * r1, cx + cosf(a0) * r1 };
            float pz4[4] = { cz + sinf(a0) * r0, cz + sinf(a1) * r0, cz + sinf(a1) * r1, cz + sinf(a0) * r1 };
            for (int m = 0; m < 4; m++) q[m] = v3(px4[m], route_height(R, px4[m], pz4[m], NULL) + 0.02f, pz4[m]);
            float mx = 0.25f * (px4[0] + px4[1] + px4[2] + px4[3]), mz = 0.25f * (pz4[0] + pz4[1] + pz4[2] + pz4[3]);
            if (fabsf(route_near(R, mx, mz).lat) < W + 0.3f) continue;   /* the main road shows through */
            Col c2 = ri == 3 ? C_VERGE : cmul(ri == 0 ? C_CROWN : C_TRACK, 0.92f + 0.16f * vnoise(mx / 1.7f, mz / 1.7f));
            gb_tri(b, q[0], q[1], q[2], c2, 1, q[0]);
            gb_tri(b, q[0], q[2], q[3], c2, 1, q[0]);
            b->g->ntri_road += 2;
        }
}

/* Build C: the creek's water down the middle of its bed, from sample i0 to i1. owner >= 0: only where that road chunk owns the
 * ground (the creek crossing the hut's clearing is drawn by the clearing's chunk, in both modes). Not under a closed bank,
 * not near the road (the ford has its own). At the waterfall the water runs white. */
static void build_creek_water(const Route *R, GB *b, int i0, int i1, int owner)
{
    const Col water = { 0.36f, 0.48f, 0.54f }, foam = { 0.86f, 0.90f, 0.92f };
    int ncx = R->gx - 1;
    b->sunlit = 1;
    for (int i = i0; i < i1 && i + 1 < R->ck_n; i++) {
        float s0 = (float)i * RT_DS, s1 = s0 + RT_DS;
        if (owner >= 0) {
            int ci = (int)((R->ck_x[i] - R->gx0) / R->cell), cj = (int)((R->ck_z[i] - R->gz0) / R->cell);
            if (ci < 0 || cj < 0 || ci >= ncx || cj >= R->gy - 1 || R->owner[(size_t)cj * ncx + ci] != owner) continue;
        }
        RtNear rn = route_near(R, R->ck_x[i], R->ck_z[i]);
        if (road_dist(R, rn) < 42.0f || route_edge(R, rn, R->ck_x[i], R->ck_z[i]) > -0.5f) continue;
        float h0 = route_creek_bed(R, s0) + 0.1f, h1 = route_creek_bed(R, s1) + 0.1f;
        float w0 = 0.3f * route_creek_bedw(R, s0) + 0.5f * vnoise(s0 / 20.0f, 2.0f) - 0.1f, w1 = 0.3f * route_creek_bedw(R, s1) + 0.5f * vnoise(s1 / 20.0f, 2.0f) - 0.1f;
        float a0 = R->ck_head[i], a1 = R->ck_head[i + 1];
        V3 p0 = v3(R->ck_x[i] + sinf(a0) * w0, h0, R->ck_z[i] - cosf(a0) * w0), p1 = v3(R->ck_x[i] - sinf(a0) * w0, h0, R->ck_z[i] + cosf(a0) * w0);
        V3 p2 = v3(R->ck_x[i + 1] - sinf(a1) * w1, h1, R->ck_z[i + 1] + cosf(a1) * w1), p3 = v3(R->ck_x[i + 1] + sinf(a1) * w1, h1, R->ck_z[i + 1] - cosf(a1) * w1);
        float steep = (h0 - h1) / RT_DS, glint = vnoise(R->ck_x[i] / 2.0f, R->ck_z[i] / 2.0f) > 0.62f ? 1.22f : 1.0f;
        gb_quad_up(b, p0, p1, p2, p3, steep > 0.12f ? foam : cmul(water, glint));
        b->g->ntri_other += 2;
        if (R->ck_fall_s > 0 && s0 > R->ck_fall_s + 3.0f && s0 < R->ck_fall_s + 11.0f) {   /* foam spreading at the fall's foot */
            float fw = w0 + 1.2f;
            V3 q0 = v3(R->ck_x[i] + sinf(a0) * fw, h0 + 0.02f, R->ck_z[i] - cosf(a0) * fw), q1 = v3(R->ck_x[i] - sinf(a0) * fw, h0 + 0.02f, R->ck_z[i] + cosf(a0) * fw);
            V3 q2 = v3(R->ck_x[i + 1] - sinf(a1) * fw, h1 + 0.02f, R->ck_z[i + 1] + cosf(a1) * fw), q3 = v3(R->ck_x[i + 1] + sinf(a1) * fw, h1 + 0.02f, R->ck_z[i + 1] - cosf(a1) * fw);
            if (((int)(s0 / 2.0f)) % 2 == 0) { gb_quad_up(b, q0, q1, q2, q3, foam); b->g->ntri_other += 2; }
        }
    }
    b->sunlit = -1;
}
/* Faint wheel ruts across the grass to the way in: two darker strips 1.8 m apart, easing in from the bank to the creek's line. */
static void build_ruts(const Route *R, GB *b)
{
    const Col rut = { 0.40f, 0.42f, 0.24f };
    float e0 = R->ck_exit_s - 70.0f, e1 = R->ck_exit_s + 4.0f;
    V3 prev[2][2]; int have = 0;
    for (float s0 = e0; s0 <= e1; s0 += 2.0f) {
        float x, z, hd = route_heading_at_creek(R, s0); route_creek_point(R, s0, &x, &z);
        float off = (route_creek_bedw(R, s0) + 3.0f) * (1.0f - smooth01((s0 - e0) / (e1 - e0)));
        V3 row[2][2];
        for (int w = 0; w < 2; w++) for (int k = 0; k < 2; k++) {
            float lat = off + (w ? 0.9f : -0.9f) + (k ? 0.18f : -0.18f), px = x - sinf(hd) * lat, pz = z + cosf(hd) * lat;
            row[w][k] = v3(px, route_height(R, px, pz, NULL) + 0.03f, pz);
        }
        if (have) for (int w = 0; w < 2; w++) { gb_quad_up(b, prev[w][0], prev[w][1], row[w][1], row[w][0], rut); b->g->ntri_other += 2; }
        memcpy(prev, row, sizeof(prev)); have = 1;
    }
}
static void build_creek_chunk(const Route *R, int index, RtGeom *g)
{
    int i0, i1; creek_chunk_span(R, index, &i0, &i1);
    g->base_x = R->ck_x[i0]; g->base_z = R->ck_z[i0];
    GB b = { g, RT_PART_GROUND, R, 0, 0, -1 };
    build_terrain(R, index, &b);
    build_trees(R, index, &b);
    build_creek_water(R, &b, i0, i1, -1);
    float es = R->ck_exit_s - 8.0f;
    if (es >= (float)i0 * RT_DS && es < (float)i1 * RT_DS) build_ruts(R, &b);
}

void route_build_chunk(const Route *R, int index, RtGeom *g)
{
    if (index >= R->nchunks) {   /* build C: the creek way's */
        struct timespec u0, u1; clock_gettime(CLOCK_MONOTONIC, &u0);
        memset(g, 0, sizeof(*g)); g->index = index;
        build_creek_chunk(R, index, g);
        clock_gettime(CLOCK_MONOTONIC, &u1);
        g->build_ms = (double)(u1.tv_sec - u0.tv_sec) * 1000.0 + (double)(u1.tv_nsec - u0.tv_nsec) / 1e6;
        return;
    }
    struct timespec t0, t1;
    clock_gettime(CLOCK_MONOTONIC, &t0);
    memset(g, 0, sizeof(*g));
    g->index = index;
    int i0 = index * RT_CHUNK;
    g->base_x = R->x[i0]; g->base_z = R->z[i0];
    GB b = { g, RT_PART_GROUND, R, 0, 0, -1 };
    build_ribbon(R, index, &b);
    if (R->spur_n > 1 && index == (int)(R->junction_s / (RT_DS * RT_CHUNK))) build_spur_ribbon(R, &b);
    build_terrain(R, index, &b);
    build_trees(R, index, &b);
    float c0 = (float)i0 * RT_DS, c1 = c0 + RT_CHUNK * RT_DS;
    if (R->ford_s > 0 && R->ford_s >= c0 && R->ford_s < c1)
        build_water(R, R->ford_lo - 1.0f, R->ford_hi + 1.0f, route_half_width() + T.verge_w + T.ditch_w + 6.0f, R->ford_water, &b);
    if (R->bridge_s1 > R->bridge_s0 && R->bridge_s0 >= c0 && R->bridge_s0 < c1) build_bridge(R, &b);
    if (R->ck_n > 1) build_creek_water(R, &b, 0, (int)((R->ck_exit_s + 6.0f) / RT_DS), index);   /* build C: the creek across the clearing */
    clock_gettime(CLOCK_MONOTONIC, &t1);
    g->build_ms = (double)(t1.tv_sec - t0.tv_sec) * 1000.0 + (double)(t1.tv_nsec - t0.tv_nsec) / 1e6;
}

/* Build B: one template on its own (a deer), set on the ground at x, z and lit like the plants, into g's GROUND part. */
void route_build_model(const Route *R, int t, float x, float z, float yaw, float scale, RtGeom *g)
{
    memset(g, 0, sizeof(*g));
    g->index = -1; g->base_x = x; g->base_z = z;
    if (t < 0 || t >= ntpl) return;
    GB b = { g, RT_PART_GROUND, R, 0, 0, -1 };
    put_template(&b, &tpl[t], x, route_height(R, x, z, NULL), z, yaw, scale);
}

void route_geom_free(RtGeom *g)
{
    for (int p = 0; p < RT_NPARTS; p++) { free(g->part[p].v); free(g->part[p].t); free(g->part[p].c); memset(&g->part[p], 0, sizeof(RtPart)); }
}

/* ------------------------------------------------------------- the far land (session 3) */
/* far.bin: 20 m cells of DEM, forest cover and distance to the road, a few km round the route. Drawn as the top of the
 * canopy where it's forest (so from the road the forest beyond the last modelled trees is a wall of crowns, and from a
 * rise the hills are wooded), as ground where it's open; sunk out of sight where the road's own ground is. */
int route_far_build(const Route *R, const char *path, RtGeom *tiles, int max_tiles)
{
    FILE *f = fopen(path, "rb");
    if (!f) { fprintf(stderr, "route: no far land (%s; scripts/fetch_far.py)\n", path); return 0; }
    char magic[4]; int nx, ny; float x0, y0, cell;
    if (fread(magic, 1, 4, f) != 4 || memcmp(magic, "BDF1", 4) || fread(&nx, 4, 1, f) != 1 || fread(&ny, 4, 1, f) != 1 ||
        fread(&x0, 4, 1, f) != 1 || fread(&y0, 4, 1, f) != 1 || fread(&cell, 4, 1, f) != 1) { fclose(f); return 0; }
    size_t N = (size_t)nx * ny;
    float *dem = malloc(N * 4), *dist = malloc(N * 4); unsigned char *cov = malloc(N);
    int ok = fread(dem, 4, N, f) == N && fread(cov, 1, N, f) == N && fread(dist, 4, N, f) == N;
    fclose(f);
    if (!ok) { free(dem); free(cov); free(dist); return 0; }
    const int step = 2, tile = 32;                    /* 40 m cells, tiles of 32 cells */
    int cx = (nx - 1) / step, cz = (ny - 1) / step, ntx = (cx + tile - 1) / tile, ntz = (cz + tile - 1) / tile, nt = 0;
    /* the height a far-land vertex is drawn at */
    float *vh = malloc(sizeof(float) * (size_t)(cx + 1) * (cz + 1)), *vsk = calloc((size_t)(cx + 1) * (cz + 1), sizeof(float));   /* build C: vsk, how far each sinks along the creek way */
    unsigned char *vc = malloc((size_t)(cx + 1) * (cz + 1));
    for (int j = 0; j <= cz; j++)
        for (int i = 0; i <= cx; i++) {
            size_t k = (size_t)(j * step) * nx + (size_t)(i * step);
            float c = 0; int nc = 0;                     /* cover averaged over the 40 m cell's corners */
            for (int dj = -1; dj <= 1; dj++) for (int di = -1; di <= 1; di++) {
                int ii = i * step + di, jj = j * step + dj;
                if (ii >= 0 && jj >= 0 && ii < nx && jj < ny) { c += cov[(size_t)jj * nx + ii]; nc++; }
            }
            c = c / (float)nc / 255.0f;
            float d = dist[k], forest = smooth01((c - 0.35f) / 0.4f);
            /* build B: the hut's frost hollow is near ground too, out to its own edge (the same distance into the edge as by the road) */
            d = fminf(d, hut_dist(R, x0 + (float)(i * step) * cell, y0 + (float)(j * step) * cell) - T.edge_hut_gap + T.edge_dist);
            float canopy = 28.0f * forest * (0.8f + 0.4f * vnoise((float)i * 0.7f, (float)j * 0.7f));
            float wall = smooth01((d - 58.0f) / 55.0f);
            float hv = dem[k] - R->elev0 + canopy * wall - 2.0f * (1.0f - smooth01((d - 52.0f) / 10.0f));
            if (d < 66.0f) {   /* follow-up: under the near ground the 40 m triangles poked up through steep hillsides out in the bush:
                                * drop each point to the lowest near ground within a cell of it, less 1.5 m */
                float vx = x0 + (float)(i * step) * cell, vz = y0 + (float)(j * step) * cell, lo = 1e9f;
                for (float dz = -22.0f; dz <= 22.0f; dz += 5.5f) for (float dx = -22.0f; dx <= 22.0f; dx += 5.5f) lo = fminf(lo, route_dem(R, vx + dx, vz + dz));
                hv += (fminf(hv, lo - 1.5f) - hv) * (1.0f - smooth01((d - 58.0f) / 8.0f));
            }
            vh[(size_t)j * (cx + 1) + i] = hv;
            vc[(size_t)j * (cx + 1) + i] = (unsigned char)(forest * 255.0f);
            if (R->ck_n > 1) {   /* build C: under the creek way's ground when it's there (the shader sinks it while you're in it) */
                float vx = x0 + (float)(i * step) * cell, vz = y0 + (float)(j * step) * cell;
                RtNear cn = route_creek_near(R, vx, vz);
                if (cn.dist < 70.0f && cn.s > R->ck_exit_s - 60.0f) {
                    float lo = 1e9f;
                    for (float dz = -22.0f; dz <= 22.0f; dz += 5.5f) for (float dx = -22.0f; dx <= 22.0f; dx += 5.5f) lo = fminf(lo, route_dem(R, vx + dx, vz + dz));
                    lo = fminf(lo, route_creek_bed(R, cn.s));
                    vsk[(size_t)j * (cx + 1) + i] = fmaxf(hv - (lo - 2.0f), 0.0f) * (1.0f - smooth01((cn.dist - 48.0f) / 20.0f));
                }
            }
        }
    for (int tz = 0; tz < ntz && nt < max_tiles; tz++)
        for (int tx = 0; tx < ntx && nt < max_tiles; tx++) {
            RtGeom *g = &tiles[nt];
            memset(g, 0, sizeof(*g));
            g->index = -1;
            g->base_x = x0 + (float)(tx * tile * step) * cell; g->base_z = y0 + (float)(tz * tile * step) * cell;
            GB b = { g, RT_PART_GROUND, R, 0, 3, 1 };
            for (int j = tz * tile; j < (tz + 1) * tile && j < cz; j++)
                for (int i = tx * tile; i < (tx + 1) * tile && i < cx; i++) {
                    V3 p[4]; float f4 = 0, sk[4];
                    for (int q = 0; q < 4; q++) {
                        int ii = i + (q & 1), jj = j + (q >> 1);
                        p[q] = v3(x0 + (float)(ii * step) * cell, vh[(size_t)jj * (cx + 1) + ii], y0 + (float)(jj * step) * cell);
                        f4 += (float)vc[(size_t)jj * (cx + 1) + ii] / 255.0f * 0.25f;
                        sk[q] = vsk[(size_t)jj * (cx + 1) + ii];
                    }
                    float mx = 0.5f * (p[0].x + p[3].x), mz = 0.5f * (p[0].z + p[3].z), nzv = vnoise(mx / 90.0f, mz / 90.0f);
                    Col grass = cmix(C_GRASS, col(0.50f, 0.52f, 0.28f), nzv), crown = cmix(col(0.20f, 0.28f, 0.15f), col(0.27f, 0.34f, 0.18f), nzv);
                    Col c = cmix(grass, crown, smooth01(f4 * 1.4f - 0.2f));
                    if (sk[0] + sk[1] + sk[2] + sk[3] > 0) {   /* build C: the sink rides in the texture coordinate (u), the shader takes it off when wanted */
                        float uva[6] = { sk[0], 3, sk[1], 3, sk[3], 3 }, uvb[6] = { sk[0], 3, sk[3], 3, sk[2], 3 };
                        V3 na = vcross(vsub(p[1], p[0]), vsub(p[3], p[0])), nb2 = vcross(vsub(p[3], p[0]), vsub(p[2], p[0]));
                        if (na.y < 0) na = vscale(na, -1);
                        if (nb2.y < 0) nb2 = vscale(nb2, -1);
                        gb_tri_ex(&b, p[0], p[1], p[3], c, na, 0, uva); gb_tri_ex(&b, p[0], p[3], p[2], cmul(c, 0.97f), nb2, 0, uvb);
                    } else {
                        gb_tri(&b, p[0], p[1], p[3], c, 1, p[0]);
                        gb_tri(&b, p[0], p[3], p[2], cmul(c, 0.97f), 1, p[0]);
                    }
                }
            if (g->part[RT_PART_GROUND].nverts) nt++;
        }
    free(vh); free(vsk); free(vc); free(dem); free(cov); free(dist);
    return nt;
}

/* -------------------------------------------------------------- streaming */

enum { CS_NONE, CS_BUILDING, CS_READY, CS_TAKEN };
static struct {
    pthread_t th; pthread_mutex_t mu; pthread_cond_t cv;
    int running, stop;
    const Route *R;
    unsigned char want[RT_MAX_CHUNKS], state[RT_MAX_CHUNKS], redo[RT_MAX_CHUNKS];
    float dist[RT_MAX_CHUNKS];
    RtGeom ready[RT_MAX_CHUNKS];
    RtStreamStats st;
} S = { .mu = PTHREAD_MUTEX_INITIALIZER, .cv = PTHREAD_COND_INITIALIZER };

#ifndef __EMSCRIPTEN__
static void *stream_main(void *arg)
{
    (void)arg;
    pthread_mutex_lock(&S.mu);
    while (!S.stop) {
        int pick = -1; float best = 1e30f;
        for (int c = 0; c < S.R->nchunks + S.R->ck_nch; c++)
            if (S.want[c] && S.state[c] == CS_NONE && S.dist[c] < best) { best = S.dist[c]; pick = c; }
        if (pick < 0) { pthread_cond_wait(&S.cv, &S.mu); continue; }
        S.state[pick] = CS_BUILDING;
        pthread_mutex_unlock(&S.mu);
        RtGeom g;
        route_build_chunk(S.R, pick, &g);
        pthread_mutex_lock(&S.mu);
        S.st.built++; S.st.ms_sum += g.build_ms; if (g.build_ms > S.st.ms_max) S.st.ms_max = g.build_ms;
        if (S.redo[pick]) { S.redo[pick] = 0; route_geom_free(&g); S.state[pick] = CS_NONE; }   /* changed while it was being built */
        else if (S.want[pick]) { S.ready[pick] = g; S.state[pick] = CS_READY; }
        else { route_geom_free(&g); S.state[pick] = CS_NONE; }
    }
    pthread_mutex_unlock(&S.mu);
    return NULL;
}
#endif

void route_stream_start(const Route *R)
{
    route_stream_stop();
    memset(S.want, 0, sizeof(S.want)); memset(S.state, 0, sizeof(S.state));
    memset(&S.st, 0, sizeof(S.st));
    S.R = R; S.stop = 0;
#ifndef __EMSCRIPTEN__
    pthread_create(&S.th, NULL, stream_main, NULL);
#endif
    S.running = 1;
}

void route_stream_stop(void)
{
    if (!S.running) return;
    pthread_mutex_lock(&S.mu); S.stop = 1; pthread_cond_signal(&S.cv); pthread_mutex_unlock(&S.mu);
#ifndef __EMSCRIPTEN__
    pthread_join(S.th, NULL);
#endif
    for (int c = 0; c < RT_MAX_CHUNKS; c++) if (S.state[c] == CS_READY) route_geom_free(&S.ready[c]);
    S.running = 0;
}

void route_stream_want(float x, float z, float radius)
{
    if (!S.running) return;
    pthread_mutex_lock(&S.mu);
    int changed = 0;
    for (int c = 0; c < S.R->nchunks + S.R->ck_nch; c++) {
        float cx, cz, r;
        route_chunk_centre(S.R, c, &cx, &cz, &r);
        float d = hypotf(cx - x, cz - z) - r;
        int w = d < radius && (c < S.R->nchunks || S.R->creek_stream);   /* build C: the creek way only while it's wanted */
        S.dist[c] = d;
        if (w != S.want[c]) { S.want[c] = (unsigned char)w; changed = 1; }
        if (!w && S.state[c] == CS_READY) { route_geom_free(&S.ready[c]); S.state[c] = CS_NONE; }
        if (!w && S.state[c] == CS_TAKEN) S.state[c] = CS_NONE; /* the caller drops it (route_stream_wanted) */
    }
    if (changed) pthread_cond_signal(&S.cv);
    pthread_mutex_unlock(&S.mu);
}

int route_stream_take(RtGeom *out)
{
    if (!S.running) return 0;
    pthread_mutex_lock(&S.mu);
    int pick = -1; float best = 1e30f;
    for (int c = 0; c < S.R->nchunks + S.R->ck_nch; c++) if (S.state[c] == CS_READY && S.dist[c] < best) { best = S.dist[c]; pick = c; }
#ifdef __EMSCRIPTEN__
    if (pick < 0) {   /* nothing ready: build the nearest wanted chunk now, one a frame */
        for (int c = 0; c < S.R->nchunks + S.R->ck_nch; c++) if (S.want[c] && S.state[c] == CS_NONE && S.dist[c] < best) { best = S.dist[c]; pick = c; }
        if (pick >= 0) {
            route_build_chunk(S.R, pick, &S.ready[pick]);
            S.st.built++; S.st.ms_sum += S.ready[pick].build_ms; if (S.ready[pick].build_ms > S.st.ms_max) S.st.ms_max = S.ready[pick].build_ms;
        }
    }
#endif
    if (pick >= 0) { *out = S.ready[pick]; S.state[pick] = CS_TAKEN; }
    pthread_mutex_unlock(&S.mu);
    return pick >= 0;
}

void route_stream_invalidate(int index)
{
    if (!S.running || index < 0 || index >= RT_MAX_CHUNKS) return;
    pthread_mutex_lock(&S.mu);
    if (S.state[index] == CS_BUILDING) S.redo[index] = 1;
    else { if (S.state[index] == CS_READY) route_geom_free(&S.ready[index]); S.state[index] = CS_NONE; }
    pthread_cond_signal(&S.cv);
    pthread_mutex_unlock(&S.mu);
}

int route_stream_wanted(int index)
{
    pthread_mutex_lock(&S.mu);
    int w = S.want[index];
    pthread_mutex_unlock(&S.mu);
    return w;
}

int route_stream_pending(void)
{
    if (!S.running) return 0;
    pthread_mutex_lock(&S.mu);
    int n = 0;
    for (int c = 0; c < S.R->nchunks + S.R->ck_nch; c++) if (S.want[c] && S.state[c] != CS_TAKEN) n++;
    pthread_mutex_unlock(&S.mu);
    return n;
}

RtStreamStats route_stream_stats(void)
{
    pthread_mutex_lock(&S.mu);
    RtStreamStats s = S.st;
    pthread_mutex_unlock(&S.mu);
    return s;
}
