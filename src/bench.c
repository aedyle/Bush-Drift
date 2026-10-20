/* --bench bookkeeping: one record per frame, kept in memory and only printed or written
 * out after the run, so the measuring does not disturb what it measures. Also samples the
 * handheld's processor and graphics clock speeds, and what the battery gauge says. */
#include "bench.h"
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/resource.h>
#include <time.h>
#include <unistd.h>

static FrameRec *rec;
static int nrec, cap;
static int fd_cpu = -1, fd_gpu = -1;
static long last_ctx, last_flt;
static double t_start;
static char governor[32] = "?", gpu_governor[32] = "?";
static long batt_tte0, batt_cc0, batt_mv0, temp_cpu0, temp_gpu0;
static long tte_samples[600];
static int ntte;
static double last_tte_t, last_rss_t;

static double now_s(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec / 1e9;
}

static long read_long(const char *path)
{
    char b[64];
    int fd = open(path, O_RDONLY);
    if (fd < 0) return -1;
    ssize_t n = read(fd, b, sizeof(b) - 1);
    close(fd);
    if (n <= 0) return -1;
    b[n] = 0;
    return atol(b);
}

static void read_word(const char *path, char *out, size_t cap_)
{
    FILE *f = fopen(path, "r");
    if (!f) return;
    if (fscanf(f, "%31s", out) != 1) snprintf(out, cap_, "?");
    fclose(f);
}

static long fd_long(int fd)
{
    char b[32];
    if (fd < 0) return 0;
    ssize_t n = pread(fd, b, sizeof(b) - 1, 0);
    if (n <= 0) return 0;
    b[n] = 0;
    return atol(b);
}

#define BATT "/sys/class/power_supply/axp2202-battery/"

void bench_begin(int max_frames)
{
    cap = max_frames + 8;
    rec = calloc((size_t)cap, sizeof(FrameRec));
    if (rec) { /* touched now, page by page, so the memory column shows the game's growth, not this log's (a plain memset after calloc is optimised away) */
        volatile char *p = (volatile char *)rec;
        for (size_t i = 0; i < (size_t)cap * sizeof(FrameRec); i += 4096) p[i] = 0;
    }
    nrec = 0;
    fd_cpu = open("/sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq", O_RDONLY);
    fd_gpu = open("/sys/class/devfreq/gpu/cur_freq", O_RDONLY);
    read_word("/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor", governor, sizeof(governor));
    read_word("/sys/class/devfreq/gpu/governor", gpu_governor, sizeof(gpu_governor));
    batt_tte0 = read_long(BATT "time_to_empty_now");
    batt_cc0 = read_long(BATT "charge_counter");
    batt_mv0 = read_long(BATT "voltage_now");
    temp_cpu0 = read_long("/sys/class/thermal/thermal_zone0/temp");
    temp_gpu0 = read_long("/sys/class/thermal/thermal_zone1/temp");
    struct rusage ru;
    getrusage(RUSAGE_SELF, &ru);
    last_ctx = ru.ru_nivcsw;
    last_flt = ru.ru_minflt + ru.ru_majflt;
    t_start = last_tte_t = now_s();
    ntte = 0;
}

void bench_frame(FrameRec *r)
{
    if (!rec || nrec >= cap) return;
    struct rusage ru;
    getrusage(RUSAGE_SELF, &ru);
    r->ctxsw = (unsigned short)(ru.ru_nivcsw - last_ctx);
    r->faults = (unsigned short)(ru.ru_minflt + ru.ru_majflt - last_flt);
    last_ctx = ru.ru_nivcsw;
    last_flt = ru.ru_minflt + ru.ru_majflt;
    r->cpu_mhz = (unsigned short)(fd_long(fd_cpu) / 1000);
    r->gpu_mhz = (unsigned short)(fd_long(fd_gpu) / 1000000);
    static unsigned rss;
    double t = now_s();
    if (!rss || t - last_rss_t >= 1.0) { /* resident pages, from /proc/self/statm, once a second */
        last_rss_t = t;
        long pages = 0, res = 0;
        FILE *f = fopen("/proc/self/statm", "r");
        if (f) { if (fscanf(f, "%ld %ld", &pages, &res) != 2) res = 0; fclose(f); }
        rss = (unsigned)(res * (sysconf(_SC_PAGESIZE) / 1024));
    }
    r->rss_kb = rss;
    rec[nrec++] = *r;
    if (t - last_tte_t >= 1.0 && ntte < 600) { /* once a second, the battery gauge's own estimate */
        last_tte_t = t;
        tte_samples[ntte++] = read_long(BATT "time_to_empty_now");
    }
}

static int cmp_float(const void *a, const void *b)
{
    float x = *(const float *)a, y = *(const float *)b;
    return x < y ? -1 : x > y;
}
static int cmp_long(const void *a, const void *b)
{
    long x = *(const long *)a, y = *(const long *)b;
    return x < y ? -1 : x > y;
}

void bench_end(const char *csv_path, int skip)
{
    if (!rec || nrec <= skip + 10) return;
    int n = nrec - skip;
    FrameRec *r = rec + skip;
    float *d = malloc(sizeof(float) * (size_t)n);
    double sum[7] = { 0 };
    int over167 = 0, over177 = 0, over25 = 0;
    for (int i = 0; i < n; i++) {
        d[i] = r[i].dt;
        if (r[i].dt > 16.7f) over167++;
        if (r[i].dt > 17.7f) over177++;
        if (r[i].dt > 25.0f) over25++;
        sum[0] += r[i].dt; sum[1] += r[i].upd; sum[2] += r[i].marks_up; sum[3] += r[i].draw; sum[4] += r[i].text; sum[5] += r[i].swap; sum[6] += r[i].audio;
    }
    qsort(d, (size_t)n, sizeof(float), cmp_float);
    printf("BENCH frames %d  mean %.2f ms (%.2f fps)  median %.2f  95th %.2f  99th %.2f  worst %.2f\n", n, sum[0] / n, 1000.0 * n / sum[0],
           d[n / 2], d[(int)(n * 0.95)], d[(int)(n * 0.99)], d[n - 1]);
    printf("BENCH frames over 16.7 ms: %d (%.1f%%)   over 17.7 ms: %d (%.1f%%)   over 25 ms: %d\n", over167, 100.0 * over167 / n, over177, 100.0 * over177 / n, over25);
    printf("BENCH ms per frame: update %.2f  mark upload %.2f  draw submission %.2f  text %.2f  swap %.2f  (sound generator %.2f, on its own thread)\n",
           sum[1] / n, sum[2] / n, sum[3] / n, sum[4] / n, sum[5] / n, sum[6] / n);
    /* processor and graphics clock speeds seen, as shares of frames */
    unsigned short seen[16];
    int count[16], ns = 0;
    for (int pass = 0; pass < 2; pass++) {
        ns = 0;
        for (int i = 0; i < n; i++) {
            unsigned short v = pass ? r[i].gpu_mhz : r[i].cpu_mhz;
            int k = 0;
            while (k < ns && seen[k] != v) k++;
            if (k == ns) { if (ns == 16) continue; seen[ns] = v; count[ns] = 0; ns++; }
            count[k]++;
        }
        printf("BENCH %s clock (%s):", pass ? "graphics" : "processor", pass ? gpu_governor : governor);
        for (int k = 0; k < ns; k++) if (seen[k]) printf("  %u MHz %.0f%%", seen[k], 100.0 * count[k] / n);
        printf("\n");
    }
    long tte1 = read_long(BATT "time_to_empty_now"), cc1 = read_long(BATT "charge_counter"), mv1 = read_long(BATT "voltage_now");
    if (batt_cc0 > 0) {
        long med = -1;
        if (ntte > 4) { /* second half of the run, once the gauge has caught up */
            int h = ntte / 2;
            qsort(tte_samples + h, (size_t)(ntte - h), sizeof(long), cmp_long);
            med = tte_samples[h + (ntte - h) / 2];
        }
        printf("BENCH battery: %.2f V -> %.2f V, gauge %ld -> %ld mAh; time-to-empty estimate %ld s -> %ld s", batt_mv0 / 1e6, mv1 / 1e6, batt_cc0 / 1000, cc1 / 1000, batt_tte0, tte1);
        if (med > 0) printf(" (middle value late in the run %ld s, which implies about %.0f mA)", med, (double)cc1 / 1000.0 * 3600.0 / (double)med);
        printf("\n");
    }
    printf("BENCH temperature: processor %.1f -> %.1f C, graphics %.1f -> %.1f C\n", temp_cpu0 / 1000.0, read_long("/sys/class/thermal/thermal_zone0/temp") / 1000.0,
           temp_gpu0 / 1000.0, read_long("/sys/class/thermal/thermal_zone1/temp") / 1000.0);
    if (csv_path) {
        FILE *f = fopen(csv_path, "w");
        if (f) {
            fprintf(f, "frame,dt,update,mark_upload,draw,text,swap,audio,marks,steps,cpu_mhz,gpu_mhz,ctxsw,faults,dist,plan_ms,chunks,rss_kb\n");
            for (int i = 0; i < n; i++)
                fprintf(f, "%d,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%u,%u,%u,%u,%u,%u,%.1f,%.3f,%u,%u\n", i + skip, r[i].dt, r[i].upd, r[i].marks_up, r[i].draw, r[i].text, r[i].swap,
                        r[i].audio, r[i].marks, r[i].steps, r[i].cpu_mhz, r[i].gpu_mhz, r[i].ctxsw, r[i].faults, r[i].dist, r[i].plan_ms, r[i].chunks, r[i].rss_kb);
            fclose(f);
        }
    }
    free(d);
}
