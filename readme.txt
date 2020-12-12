External repo: CLI: https://github.com/CLIUtils/CLI11 command line interface

Assimp: https://github.com/assimp/assimp module loader

Steps to run the code:

mkdir build

cd build

cmake ..

make

./ray-tracer -s ../media/bunny.dae

Note: when running bvh algorithm, the algorithm is too fast, so in order to show the improvement of openmp parallelization, we use sampling to make the task bigger. The command we ran for the results of parallelization is like this:

./ray-tracer -s ../media/dragon.dae -x 1920 -y 1080 -n 16

Specifics in running the code to generate results:

Check main.cpp, there are three lines of codes, two of which are commented out.

//BVHTrace(scene, framebuffer, width, height, light_per_pixel);

//simpleTrace(scene, framebuffer, width, height);

simdTrace(scene, framebuffer, width, height);

These are respectively Bounding volume tracing, naive baseline tracing, and naive baseline tracing with simd. Uncomment the function you want to run and make(only leave one uncommented at one time).

To get the results of time versus threads in the parallelization part, firstly uncomment the BVHTrace function in main.cpp only, then change the num_threads() value in line 196 of ray_tracer.cpp to the value you like