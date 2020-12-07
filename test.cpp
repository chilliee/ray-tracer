#include "ray_tracer.hpp"
#include "CLI/CLI.hpp"

#include <cmath>
#include <ostream>
#include <cstdlib>
#include <ctime>
#include <string>
#include <iostream>
#include <iomanip>
#include <fstream>

static __inline__ unsigned long long rdtsc(void) {
  unsigned hi, lo;
  __asm__ __volatile__ ("rdtsc" : "=a"(lo), "=d"(hi));
  return ( (unsigned long long)lo)|( ((unsigned long long)hi)<<32 );
}

int test() {
    unsigned long long t0, t1, t2;
    float LO = -1.;
    float HI = 1.;
    srand (static_cast <unsigned> (time(0)));
    float x1 = LO + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(HI-LO)));
    float x2 = LO + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(HI-LO)));
    float x3 = LO + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(HI-LO)));
    float y1 = LO + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(HI-LO)));
    float y2 = LO + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(HI-LO)));
    float y3 = LO + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(HI-LO)));
    Vec3f x = Vec3f(x1, x2, x3);
    Vec3f y = Vec3f(y1, y2, y3);

    t0 = rdtsc();

    dot(x, y);

    t1 = rdtsc();

    cross(x, y);

    t2 = rdtsc();

    // print out result below
}