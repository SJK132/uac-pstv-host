# Host-side tests for src/mixer.c. No VitaSDK needed -- tests/stubs provides
# the handful of kernel declarations mixer.c touches, and the harness supplies
# ksceKernelCopyFromUserProc as a memcpy.
#
#   make          build and run the correctness suite
#   make race     deterministic detector for the drain/consumer race
#   make bench    fill() hot-path A/B against the pre-optimisation algorithm
#   make all      all three

CC      ?= gcc
CFLAGS  ?= -O2 -std=gnu99 -Wall -Wextra
INCLUDE  = -Istubs -I../src

.PHONY: test race bench all clean
test: test_mixer
	./test_mixer

race: test_race
	./test_race

bench: bench_fill
	./bench_fill

all: test race bench

test_mixer: test_mixer.c ../src/mixer.c
	$(CC) $(CFLAGS) $(INCLUDE) -o $@ $^

test_race: test_race.c ../src/mixer.c
	$(CC) $(CFLAGS) -DUAC_MIXER_TEST $(INCLUDE) -o $@ $^

bench_fill: bench.c
	$(CC) $(CFLAGS) -o $@ $<

clean:
	rm -f test_mixer test_race bench_fill
