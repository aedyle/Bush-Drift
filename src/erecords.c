#include "erecords.h"
#include "records.h"
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

ERecords EREC;

static char path_[240];
static pthread_t save_thread;
static int save_running;

static int read_file(const char *path, ERecords *out)
{
    FILE *f = fopen(path, "r");
    if (!f) return 0;
    ERecords r;
    memset(&r, 0, sizeof(r));
    char line[200], date[32];
    int head = 0, end = 0;
    while (fgets(line, sizeof(line), f)) {
        int a, b;
        float d;
        unsigned seed;
        if (!head) { if (strncmp(line, "bush-drift records 1", 20) != 0) break;   /* build B: was 32, which never matched (the records never loaded) */ head = 1; continue; }
        int test = strstr(line, " test") != NULL;
        if (sscanf(line, "entry %d %d %f %x %31s", &a, &b, &d, &seed, date) == 5) {
            if (r.n < EREC_MAX && a > 0) {
                ERecEntry *e = &r.top[r.n++];
                e->score = a; e->mult = b > 0 ? b : 0; e->dist = d > 0 ? d : 0; e->seed = seed; e->test = test;
                records_set_date(e->date, date);
            }
        } else if (sscanf(line, "furthest %f %x %31s", &d, &seed, date) == 3) {
            r.furthest.value = d; r.furthest.seed = seed; r.furthest.test = test; records_set_date(r.furthest.date, date);
        } else if (sscanf(line, "bestcombo %f %x %31s", &d, &seed, date) == 3) {
            r.combo.value = d; r.combo.seed = seed; r.combo.test = test; records_set_date(r.combo.date, date);
        } else if (sscanf(line, "bestmult %f %x %31s", &d, &seed, date) == 3) {
            r.mult.value = d; r.mult.seed = seed; r.mult.test = test; records_set_date(r.mult.date, date);
        } else if (!strncmp(line, "end", 3)) { end = 1; break; }
    }
    fclose(f);
    if (!head || !end) return 0;
    for (int i = 1; i < r.n; i++) /* keep the list in order whatever the file says */
        for (int j = i; j > 0 && r.top[j].score > r.top[j - 1].score; j--) { ERecEntry t = r.top[j]; r.top[j] = r.top[j - 1]; r.top[j - 1] = t; }
    *out = r;
    return 1;
}

static char *text(void)
{
    char *s = malloc(2048);
    if (!s) return NULL;
    int k = snprintf(s, 2048, "bush-drift records 1\n");
    for (int i = 0; i < EREC.n; i++) {
        const ERecEntry *e = &EREC.top[i];
        k += snprintf(s + k, 2048 - (size_t)k, "entry %d %d %.0f %08X %s%s\n", e->score, e->mult, e->dist, e->seed, e->date[0] ? e->date : "-", e->test ? " test" : "");
    }
    const ERecBest *b[3] = { &EREC.furthest, &EREC.combo, &EREC.mult };
    const char *name[3] = { "furthest", "bestcombo", "bestmult" };
    for (int i = 0; i < 3; i++)
        if (b[i]->value > 0) k += snprintf(s + k, 2048 - (size_t)k, "%s %.0f %08X %s%s\n", name[i], b[i]->value, b[i]->seed, b[i]->date[0] ? b[i]->date : "-", b[i]->test ? " test" : "");
    snprintf(s + k, 2048 - (size_t)k, "end\n");
    return s;
}

void erecords_load(const char *path)
{
    char other[300];
    snprintf(path_, sizeof(path_), "%s", path);
    memset(&EREC, 0, sizeof(EREC));
    if (read_file(path, &EREC)) return;
    snprintf(other, sizeof(other), "%s.bak", path);
    if (read_file(other, &EREC)) { fprintf(stderr, "records: %s unreadable, second copy used\n", path); return; }
    snprintf(other, sizeof(other), "%s.tmp", path);
    if (read_file(other, &EREC)) fprintf(stderr, "records: %s unreadable, finished save used\n", path);
}

int erecords_add_run(int score, int mult, float dist, uint32_t seed, const char *date)
{
    if (score <= 0) return 0;
    int at = EREC.n;
    while (at > 0 && score > EREC.top[at - 1].score) at--; /* an equal score ranks below the earlier one */
    if (at >= EREC_MAX) return 0;
    if (EREC.n < EREC_MAX) EREC.n++;
    for (int i = EREC.n - 1; i > at; i--) EREC.top[i] = EREC.top[i - 1];
    ERecEntry *e = &EREC.top[at];
    memset(e, 0, sizeof(*e));
    e->score = score; e->mult = mult; e->dist = dist; e->seed = seed;
    records_set_date(e->date, date);
    return at + 1;
}

int erecords_best(ERecBest *b, float value, uint32_t seed, const char *date)
{
    if (!(value > b->value)) return 0;
    b->value = value; b->seed = seed; b->test = 0;
    records_set_date(b->date, date);
    return 1;
}

static void *save_main(void *arg)
{
    char *t = arg, bak[300];
    snprintf(bak, sizeof(bak), "%s.bak", path_);
    if (!records_write_whole(path_, t)) fprintf(stderr, "records: could not save %s\n", path_);
    records_write_whole(bak, t);
    free(t);
    return NULL;
}

void erecords_finish(void)
{
    if (save_running) { pthread_join(save_thread, NULL); save_running = 0; }
}

void erecords_save(void)
{
    if (!path_[0]) return;
    erecords_finish();
    char *t = text();
    if (!t) return;
    if (pthread_create(&save_thread, NULL, save_main, t) == 0) save_running = 1;
    else save_main(t);
}

static int check(int ok, const char *what) { printf("  %s: %s\n", ok ? "ok  " : "FAIL", what); return ok; }

int erecords_selftest(const char *dir)
{
    char path[300], bak[320];
    snprintf(path, sizeof(path), "%s/erecords-test.txt", dir);
    snprintf(bak, sizeof(bak), "%s.bak", path);
    remove(path); remove(bak);
    int ok = 1;
    erecords_load(path);
    ok &= check(EREC.n == 0 && EREC.furthest.value == 0, "no file: empty records");
    ok &= check(erecords_add_run(5000, 4, 812, 0xABCD1234u, "2026-10-14") == 1, "first run ranks 1st");
    ok &= check(erecords_add_run(9000, 7, 1450, 0x0000BEEFu, "2026-10-15") == 1, "a higher score ranks 1st");
    ok &= check(erecords_add_run(5000, 2, 300, 1, "2026-10-15") == 3, "an equal score ranks below the earlier one");
    for (int i = 0; i < 8; i++) erecords_add_run(100 + i, 1, 50, 2, "2026-10-15");
    ok &= check(EREC.n == EREC_MAX && erecords_add_run(10, 1, 10, 3, "2026-10-15") == 0, "ten kept, a lower one does not rank");
    ok &= check(erecords_best(&EREC.furthest, 1450, 0xBEEF, "2026-10-15") == 1 && erecords_best(&EREC.furthest, 900, 1, "2026-10-15") == 0, "furthest only moves when beaten");
    ok &= check(erecords_best(&EREC.combo, 4200, 0xBEEF, "2026-10-15") == 1, "best combo");
    ok &= check(erecords_best(&EREC.mult, 7, 0xBEEF, "2026-10-15") == 1, "highest multiplier");
    EREC.top[1].test = 1; EREC.combo.test = 1; /* a row and a best carried in from a test entry */
    erecords_save();
    erecords_finish();
    ERecords want = EREC;
    erecords_load(path);
    ok &= check(EREC.n == want.n && EREC.top[0].score == 9000 && EREC.top[0].seed == 0x0000BEEFu && EREC.top[0].dist > 1449 && !strcmp(EREC.top[0].date, "2026-10-15")
                && EREC.furthest.value > 1449 && EREC.furthest.seed == 0xBEEF && EREC.combo.value > 4199 && EREC.mult.value > 6.9f, "saved and read back the same, seeds included");
    ok &= check(EREC.top[1].test == 1 && EREC.top[0].test == 0 && EREC.combo.test == 1 && EREC.furthest.test == 0, "test marks kept");
    ok &= check(erecords_best(&EREC.combo, 5000, 3, "2026-10-16") == 1 && EREC.combo.test == 0, "a best beaten in the game loses its test mark");
    FILE *f = fopen(path, "w"); fprintf(f, "bush-drift records 1\nentry 12"); fclose(f);
    erecords_load(path);
    ok &= check(EREC.n == want.n && EREC.top[0].score == 9000, "a damaged main file falls back to the second copy");
    remove(path); remove(bak);
    printf(ok ? "erecordstest finished\n" : "erecordstest FAILED\n");
    return ok ? 0 : 1;
}
