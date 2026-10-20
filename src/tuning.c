#include "tuning.h"
#include "tuning_defaults.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

Tuning T;

static const char *names[] = {
#define X(n) #n,
    TUNE_SCALARS
#undef X
};
static float *ptrs[] = {
#define X(n) &T.n,
    TUNE_SCALARS
#undef X
};
#define NSCALAR ((int)(sizeof(names) / sizeof(names[0])))
static unsigned char seen[sizeof(names) / sizeof(names[0])];
static char missing[2048];


int tuning_count(void) { return NSCALAR; }
const char *tuning_name(int i) { return names[i]; }
float *tuning_ptr(int i) { return ptrs[i]; }

static char *trim(char *s)
{
    while (*s == ' ' || *s == '\t') s++;
    char *e = s + strlen(s);
    while (e > s && (e[-1] == ' ' || e[-1] == '\t' || e[-1] == '\n' || e[-1] == '\r')) *--e = 0;
    return s;
}

/* One "key = value" line (changed in place). Returns 1 if the key was known. */
static int tuning_line(char *line)
{
    char *hash = strchr(line, '#');
    if (hash) *hash = 0;
    char *eq = strchr(line, '=');
    if (!eq) return 0;
    *eq = 0;
    char *key = trim(line), *val = trim(eq + 1);
    for (int i = 0; i < NSCALAR; i++) {
        if (strcmp(key, names[i]) == 0) {
            *ptrs[i] = strtof(val, NULL);
            seen[i] = 1;
            return 1;
        }
    }
    fprintf(stderr, "tuning: unknown key '%s'\n", key);
    return 0;
}

void tuning_defaults(void)
{
    char line[1024];
    memset(seen, 0, sizeof(seen));
    for (const char *p = tuning_default_text; *p; ) {
        const char *e = strchr(p, '\n');
        size_t n = e ? (size_t)(e - p) : strlen(p);
        if (n >= sizeof(line)) n = sizeof(line) - 1;
        memcpy(line, p, n);
        line[n] = 0;
        tuning_line(line);
        p += e ? n + 1 : n;
    }
    missing[0] = 0;
    for (int i = 0; i < NSCALAR; i++) {
        if (!seen[i] && strlen(missing) + strlen(names[i]) + 3 < sizeof(missing)) {
            if (missing[0]) strcat(missing, ", ");
            strcat(missing, names[i]);
        }
    }
#ifdef HANDHELD
    T.pad_swap_ab = 0; /* the handheld's own buttons arrive the right way round */
#endif
}

int tuning_load(const char *path)
{
    tuning_defaults();
    FILE *f = fopen(path, "r");
    if (!f) return -1;
    char line[1024];
    int count = 0;
    while (fgets(line, sizeof(line), f)) count += tuning_line(line);
    fclose(f);
    return count;
}

/* Lines of "key = value" text applied on top of whatever is in force (the handling presets). */
int tuning_apply_text(const char *text)
{
    char line[1024];
    int count = 0;
    for (const char *p = text; *p; ) {
        const char *e = strchr(p, '\n');
        size_t n = e ? (size_t)(e - p) : strlen(p);
        if (n >= sizeof(line)) n = sizeof(line) - 1;
        memcpy(line, p, n);
        line[n] = 0;
        count += tuning_line(line);
        p += e ? n + 1 : n;
    }
    return count;
}

const char *tuning_missing(void) { return missing; }

int tuning_save(const char *path)
{
    FILE *f = fopen(path, "w");
    if (!f) return -1;
    fprintf(f, "# Saved from the game (F6). key = value.\n");
    for (int i = 0; i < NSCALAR; i++) fprintf(f, "%s = %g\n", names[i], *ptrs[i]);
    fclose(f);
    return 0;
}
