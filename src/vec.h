/* Small vector and quaternion helpers. Axes:
 * x = right, y = up, z = forward. The algebra is the ordinary kind; only the
 * renderer has to mirror z. */
#ifndef VEC_H
#define VEC_H
#include <math.h>

typedef struct { float x, y, z; } V3;
typedef struct { float x, y, z, w; } Q;

#define RAD2DEGF 57.29578f
#define DEG2RADF 0.017453292f

static inline V3 v3(float x, float y, float z) { V3 v = { x, y, z }; return v; }
static inline V3 vadd(V3 a, V3 b) { return v3(a.x + b.x, a.y + b.y, a.z + b.z); }
static inline V3 vsub(V3 a, V3 b) { return v3(a.x - b.x, a.y - b.y, a.z - b.z); }
static inline V3 vscale(V3 a, float s) { return v3(a.x * s, a.y * s, a.z * s); }
static inline float vdot(V3 a, V3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
static inline V3 vcross(V3 a, V3 b) { return v3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x); }
static inline float vlen(V3 a) { return sqrtf(vdot(a, a)); }
static inline V3 vnorm(V3 a) { float l = vlen(a); return l > 1e-9f ? vscale(a, 1.0f / l) : v3(0, 0, 0); }
static inline V3 vlerp(V3 a, V3 b, float t) { return vadd(a, vscale(vsub(b, a), t)); }

static inline Q qident(void) { Q q = { 0, 0, 0, 1 }; return q; }
static inline Q qmul(Q a, Q b)
{
    Q r = { a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
            a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
            a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
            a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z };
    return r;
}
static inline Q qnorm(Q q)
{
    float l = sqrtf(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    if (l < 1e-9f) return qident();
    Q r = { q.x / l, q.y / l, q.z / l, q.w / l };
    return r;
}
static inline Q qconj(Q q) { Q r = { -q.x, -q.y, -q.z, q.w }; return r; }
static inline V3 qrot(Q q, V3 v)
{
    V3 u = v3(q.x, q.y, q.z);
    V3 t = vscale(vcross(u, v), 2.0f);
    return vadd(vadd(v, vscale(t, q.w)), vcross(u, t));
}
static inline Q qaxis(V3 axis, float rad)
{
    float s = sinf(rad * 0.5f);
    Q q = { axis.x * s, axis.y * s, axis.z * s, cosf(rad * 0.5f) };
    return q;
}
static inline Q qnlerp(Q a, Q b, float t)
{
    float d = a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
    float s = d < 0 ? -1.0f : 1.0f;
    Q r = { a.x + (b.x * s - a.x) * t, a.y + (b.y * s - a.y) * t, a.z + (b.z * s - a.z) * t, a.w + (b.w * s - a.w) * t };
    return qnorm(r);
}
static inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
static inline float clamp01(float v) { return clampf(v, 0.0f, 1.0f); }
static inline float signf(float v) { return v >= 0.0f ? 1.0f : -1.0f; }
static inline float maxf(float a, float b) { return a > b ? a : b; }
static inline float minf(float a, float b) { return a < b ? a : b; }
#endif
