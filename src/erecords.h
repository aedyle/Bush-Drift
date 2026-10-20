/* Endless records: a top ten by score (with distance, best multiplier, date and the road's seed),
 * plus the furthest distance, the best single combo and the highest multiplier, each with its seed
 * and date. Same file handling as the old track records (records.c): replaced whole, a second copy
 * beside it, saved on another thread. No drawing in here. */
#ifndef ERECORDS_H
#define ERECORDS_H
#include <stdint.h>
#define EREC_MAX 10
/* test = 1: carried in from a test entry's own records (16 Oct), shown as "test" after the date; the file
 * writes it as the word "test" at the end of the line */
typedef struct { int score, mult; float dist; uint32_t seed; char date[12]; int test; } ERecEntry;
typedef struct { float value; uint32_t seed; char date[12]; int test; } ERecBest;
typedef struct {
    int n;
    ERecEntry top[EREC_MAX];
    ERecBest furthest, combo, mult; /* value 0 = none yet */
} ERecords;
extern ERecords EREC;
void erecords_load(const char *path);
/* A finished run. Returns its place in the top ten (1 = best), or 0. */
int erecords_add_run(int score, int mult, float dist, uint32_t seed, const char *date);
/* Each returns 1 if it is a new best. */
int erecords_best(ERecBest *b, float value, uint32_t seed, const char *date);
void erecords_save(void);
void erecords_finish(void);
int erecords_selftest(const char *dir);
#endif
