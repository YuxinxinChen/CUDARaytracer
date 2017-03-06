Implement ray tracing on CUDA.

To compile the code, run nvcc -o raytrace raytrace.cu. That is it.

# Instructions:

raytrace.cu is the file only use CUDA. To compile and run raytrace.cu, run nvcc -o raytrace raytrace.cu, and then ./raytrace.

CUDAGLraytrace.cu is the file use CUDA to compute the ray trace data and use OpenGL to render the picture. To compile and run the code:

```bash
 mkdir build
 cd build
 cmake .
 make
 ./cudaraytracer
```
