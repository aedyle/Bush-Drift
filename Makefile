CC ?= cc
CFLAGS ?= -O2 -g -Wall -Wextra -std=c99 -D_DEFAULT_SOURCE -DNO_SAMPLES
SRC = src/main.c src/car.c src/tuning.c src/bench.c src/records.c src/route.c src/erecords.c src/world.c
HDR = src/car.h src/tuning.h src/vec.h src/bench.h src/records.h src/route.h src/erecords.h src/world.h build/tuning_defaults.h

# Bush Drift. NO_SAMPLES: generated sound only.
bush-drift: $(SRC) $(HDR)
	$(CC) $(CFLAGS) -Ibuild -o $@ $(SRC) $$(pkg-config --cflags --libs raylib) -lm -lpthread

# The tuning values, copied into the program so it runs without a tuning file: the game's (tuning-bush.txt)
# and the ute's handling (tuning-car.txt).
build/tuning_defaults.h: tuning-bush.txt tuning-car.txt tools/gen_defaults.py
	mkdir -p build
	python3 -I tools/gen_defaults.py tuning-bush.txt tuning-car.txt $@

# The road and the ground under it, from OpenStreetMap, the NSW 5 m DEM and NSW imagery (scripts/README.md).
# Downloads are cached in data/raw; outputs go to data/built.
route:
	scripts/build_all.sh

# Standalone handling test: src/car.c with the car's own tuning only (tuning-car.txt). The defaults
# header goes to build/car/ under the name src/tuning.c includes, so build/tuning_defaults.h is untouched.
build/tuning_defaults_car.h: tuning-car.txt tools/gen_defaults.py
	mkdir -p build/car
	python3 -I tools/gen_defaults.py tuning-car.txt $@

build/car/tuning_defaults.h: build/tuning_defaults_car.h
	mkdir -p build/car
	cp $< $@

cartest: src/car.c src/car.h src/tuning.c src/tuning.h src/vec.h tools/cartest_main.c build/car/tuning_defaults.h
	cc -O2 -std=c99 -D_DEFAULT_SOURCE -Wall -Wextra -Ibuild/car -Isrc -o $@ src/car.c src/tuning.c tools/cartest_main.c -lm

run: bush-drift
	./bush-drift


clean:
	rm -f bush-drift cartest

.PHONY: run clean route
