/* Shared by the records file (erecords.c): writing a file whole, and dates. No drawing in here. */
#ifndef RECORDS_H
#define RECORDS_H

/* Today as YYYY-MM-DD, or "-" if the clock has plainly never been set. */
void records_today(char out[12]);
/* "2026-10-11" -> "11 Oct 2026"; anything else -> "--". */
const char *records_date_text(const char *iso);
/* Write a file whole (under another name, pushed to the card, then renamed), and a checked date. */
int records_write_whole(const char *path, const char *text);
void records_set_date(char dst[12], const char *src);
#endif
