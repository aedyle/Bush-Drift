#ifndef BENCH_H
#define BENCH_H

/* One frame of a --bench run. Times in milliseconds. */
typedef struct {
    float dt;        /* whole frame, as the game saw it */
    float upd;       /* input, handling, rules, sound control */
    float marks_up;  /* sending new tyre marks to the graphics chip */
    float draw;      /* submitting the 3D scene */
    float text;      /* 2D text and flushing it */
    float swap;      /* handing the frame to the screen (includes waiting for it, and reading the buttons) */
    float audio;     /* time the sound generator ran during this frame (its own thread) */
    unsigned short marks, steps, cpu_mhz, gpu_mhz, ctxsw, faults;
    float dist;      /* metres reached on the run under way */
    float plan_ms;   /* planner-thread time for chunks finished during this frame (its own thread) */
    unsigned short chunks; /* chunks the planner finished during this frame */
    unsigned rss_kb; /* resident memory, read once a second */
} FrameRec;

void bench_begin(int max_frames);
void bench_frame(FrameRec *r);                 /* fills in the system columns and stores the record */
void bench_end(const char *csv_path, int skip); /* prints the BENCH lines; skip = start-up frames to leave out */
#endif
