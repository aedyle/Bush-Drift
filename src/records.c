#include "records.h"
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

void records_set_date(char dst[12], const char *src)
{
    int y, m, d;
    if (src && sscanf(src, "%4d-%2d-%2d", &y, &m, &d) == 3 && y > 2000 && y <= 9999 && m >= 1 && m <= 12 && d >= 1 && d <= 31) snprintf(dst, 12, "%04d-%02d-%02d", y, m, d);
    else strcpy(dst, "-");
}

/* The whole file goes to "<name>.tmp" and is pushed out to the card before it takes the real
 * name, so a crash or power cut leaves either the old records or the new, never a part of one. */
int records_write_whole(const char *path, const char *text)
{
    char tmp[300];
    snprintf(tmp, sizeof(tmp), "%s.tmp", path);
    FILE *f = fopen(tmp, "w");
    if (!f) return 0;
    int ok = fputs(text, f) >= 0;
    ok = fflush(f) == 0 && ok;
    ok = fsync(fileno(f)) == 0 && ok;
    ok = fclose(f) == 0 && ok;
    if (!ok) { remove(tmp); return 0; }
    if (rename(tmp, path) != 0) return 0;
    char dir[300];
    snprintf(dir, sizeof(dir), "%s", path);
    char *slash = strrchr(dir, '/');
    if (slash) *slash = 0; else strcpy(dir, ".");
    int fd = open(dir, O_RDONLY);
    if (fd >= 0) { fsync(fd); close(fd); }
    return 1;
}

void records_today(char out[12])
{
    time_t now = time(NULL);
    struct tm tmv;
    if (localtime_r(&now, &tmv) && tmv.tm_year + 1900 >= 2026) strftime(out, 12, "%Y-%m-%d", &tmv);
    else strcpy(out, "-");
}

const char *records_date_text(const char *iso)
{
    static char buf[4][16];
    static int turn;
    static const char *mon[12] = { "Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec" };
    int y, m, d;
    if (!iso || sscanf(iso, "%4d-%2d-%2d", &y, &m, &d) != 3 || m < 1 || m > 12) return "--";
    char *o = buf[turn++ & 3];
    snprintf(o, 16, "%d %s %d", d, mon[m - 1], y);
    return o;
}
