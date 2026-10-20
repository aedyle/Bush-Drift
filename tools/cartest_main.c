/* Standalone handling test: the built-in car tuning (tuning-car.txt), optionally a tuning file on
 * top (cartest file.txt), then car_selftest(). */
#include "car.h"
#include "tuning.h"
#include <stdio.h>

int main(int argc, char **argv)
{
    if (argc > 1) {
        if (tuning_load(argv[1]) < 0) fprintf(stderr, "cartest: cannot read %s, using built-in values\n", argv[1]);
    } else {
        tuning_defaults();
    }
    return car_selftest();
}
