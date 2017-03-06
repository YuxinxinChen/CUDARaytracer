#include <cstdlib>
#include <cstdio>
#include <cmath>
#include <fstream>
#include <vector>
#include <iostream>
#include <cassert>

#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <cuda_runtime.h>
#include <cuda_gl_interop.h>

#define size 6
#define MAX_RAY_DEPTH 3
const unsigned int window_width = 512;
const unsigned int window_height = 512;

struct vertexColor
{
        float x,y;
        float r,g,b;
};
struct Sphere{

	float3 color;
	float radius;
	float3 center;
	float3 emissionColor;
	float transparency;
	float reflection;

	__device__ bool hit(float3 origin, float3 direct, float *t0, float *t1) {
		float3 l = make_float3(center.x - origin.x, center.y-origin.y, center.z-origin.z);
		float tca = l.x*direct.x + l.y*direct.y+ l.z*direct.z;
		if(tca <0) return false;
		float d2 = l.x*l.x + l.y*l.y + l.z*l.z - tca*tca;
		if(d2 > radius*radius) return false;
		float thc = sqrtf(radius*radius - d2);
		*t0 = tca -thc;
                *t1 = tca +thc;
		return true;
	}
};

__constant__ Sphere s[size];

__device__ void normalize(float3 &f)
{
	float temp = sqrtf(f.x*f.x + f.y*f.y + f.z*f.z);
	f.x = f.x/temp; 
	f.y = f.y/temp;
	f.z = f.z/temp;
}

__device__ float mix(float a, float b, float mix)
{
    return b * mix + a * (1 - mix);
}

__device__ float3 trace(float3 origin, float3 direct, int depth)
{
    float dist = INFINITY;
    int intersect_object = -1;
    for(unsigned i=0; i<size; i++){
	float t0 = INFINITY, t1 = INFINITY; 
	if(s[i].hit(origin, direct, &t0, &t1))
	{
	    if(t0<0) t0 = t1;
	    if(t0< dist){
 		dist = t0;
		intersect_object = i;
	    }
	}
     }

    if(intersect_object == -1){
  	float3 result = make_float3(2, 2, 2);
	return result;
    }
    float3 surfaceColor = make_float3(0,0,0);
    float3 phit = make_float3(origin.x + direct.x*dist,origin.y + direct.y*dist,origin.z + direct.z*dist);
    float3 nhit = make_float3(phit.x-s[intersect_object].center.x, phit.y-s[intersect_object].center.y, phit.z-s[intersect_object].center.z);
    normalize(nhit);

    float bias = 1e-4;
    bool inside = false;
    if( (direct.x*nhit.x+direct.y*nhit.y+direct.z*nhit.z) > 0){
	nhit.x = (-1)*nhit.x;
	nhit.y = (-1)*nhit.y;
	nhit.z = (-1)*nhit.z;
	inside = true;
    }
 
    if((s[intersect_object].transparency > 0 || s[intersect_object].reflection > 0 ) && depth < MAX_RAY_DEPTH)
    {
	float tmp = direct.x*nhit.x + direct.y*nhit.y + direct.z*nhit.z;
	float facingratio = (-1)*tmp;
	float fresneleffect = mix(pow(1-facingratio, 3), 1, 0.1);
	float3 refldir = make_float3(direct.x - 2*tmp*nhit.x, direct.y - 2*tmp*nhit.y, direct.z - 2*tmp*nhit.z);
	normalize(refldir);
	float3 newReflRay = make_float3(phit.x + nhit.x*bias, phit.y + nhit.y*bias, phit.z + nhit.z*bias);
	float3 reflection = trace(newReflRay, refldir, depth+1);
	float3 refraction = make_float3(0,0,0);
	if(s[intersect_object].transparency){
	    float ior = 1.1, eta;
	    if(inside) eta = ior; else eta = 1/ior;
	    float cosi = (-1)*(nhit.x*direct.x + nhit.y*direct.y + nhit.z*direct.z);
	    float k = 1 -eta*eta*(1-cosi*cosi);
	    float3 refrdir = make_float3(direct.x*eta+nhit.x*(eta*cosi-sqrt(k)), direct.y*eta+nhit.y*(eta*cosi-sqrt(k)), direct.z*eta+nhit.z*(eta*cosi-sqrt(k)));
	    normalize(refrdir);
	    float3 newRefrRay = make_float3(phit.x-bias*nhit.x, phit.y-bias*nhit.y, phit.z-bias*nhit.z);
	    refraction = trace(newRefrRay, refrdir, depth+1);
	}

	surfaceColor.x = (reflection.x * fresneleffect + refraction.x * (1 - fresneleffect) * s[intersect_object].transparency )*s[intersect_object].color.x;
	surfaceColor.y = (reflection.y * fresneleffect + refraction.y * (1 - fresneleffect) * s[intersect_object].transparency )*s[intersect_object].color.y;
	surfaceColor.z = (reflection.z * fresneleffect + refraction.z * (1 - fresneleffect) * s[intersect_object].transparency )*s[intersect_object].color.z;
     } 
    else{
    // diffuse 
    	for(unsigned i=0; i<size; i++){
	    if(s[i].emissionColor.x>0){
	    	float3 transmission = make_float3(1,1,1);
	    	float3 lightDirection = make_float3(s[i].center.x-phit.x, s[i].center.y-phit.y, s[i].center.z-phit.z);
	    	normalize(lightDirection);
	    	for(unsigned j=0; j< size; j++){
		    if(i!=j){
		    	float t0, t1;
		    	float3 newRay = make_float3(phit.x+bias*nhit.x, phit.y+bias*nhit.y, phit.z+bias*nhit.z);
		    	if(s[j].hit(newRay, lightDirection, &t0, &t1)){
			    transmission.x = 0; transmission.y=0; transmission.z=0;
			    break;
		    	}
                    }  
                }
	        float tmp = nhit.x*lightDirection.x+nhit.y*lightDirection.y+nhit.z*lightDirection.z;
	        if(tmp<0) tmp = 0;

	       surfaceColor.x = surfaceColor.x+ s[intersect_object].color.x*transmission.x*tmp*s[i].emissionColor.x;
	       surfaceColor.y = surfaceColor.y+ s[intersect_object].color.y*transmission.y*tmp*s[i].emissionColor.y;
	       surfaceColor.z = surfaceColor.z+ s[intersect_object].color.z*transmission.z*tmp*s[i].emissionColor.z;
	  }
       }
    }
 

   float3 result2 = make_float3(surfaceColor.x+s[intersect_object].emissionColor.x, 
					 surfaceColor.y+s[intersect_object].emissionColor.y,
					 surfaceColor.z+s[intersect_object].emissionColor.z);
   return result2;
}

__global__ void tracer_kernel(vertexColor *pos)
{
  int x = blockIdx.x*blockDim.x + threadIdx.x;
  int y = blockIdx.y*blockDim.y + threadIdx.y;
   if(x < window_width && y < window_height)
  {
    //----------compute ray----------------/
    float window_ratio = window_width / float(window_height);
    float fov = 90;
    float angle = tan(fov * 0.5 * M_PI / 180.0);
    float u = (2.0 * (x+0.5)/(float)window_width -1) * angle * window_ratio;
    float v = (1 - 2.0 * (y+0.5) / (float) window_height) * angle;
   
    float3 origin = make_float3(0,0,0);
    float3 direct = make_float3(u,v,-1);
    normalize(direct);
    //----------compute ray--------------//

    float3 pixel = trace(origin, direct, 0);
    pos[x+window_width*y].x = u;
    pos[x+window_width*y].y = v;
    pos[x+window_width*y].r = pixel.x;
    pos[x+window_width*y].g = pixel.y;
    pos[x+window_width*y].b = pixel.z;
   
  }
}

void init_kernel()
{
  Sphere *temp_s = (Sphere*)malloc(sizeof(Sphere)*size);
  temp_s[0].center = make_float3(0, 0, -20);
  temp_s[0].radius = 4;
  temp_s[0].color = make_float3(1.00, 0.32, 0.36);
  temp_s[0].emissionColor = make_float3(0,0,0);
  temp_s[0].transparency = 0.5;
  temp_s[0].reflection = 1;

  temp_s[1].center = make_float3(0, 20, -30);
  temp_s[1].radius = 3;
  temp_s[1].color = make_float3(0, 0, 0);
  temp_s[1].emissionColor = make_float3(3, 3, 3);
  temp_s[1].transparency = 0;
  temp_s[1].reflection = 0;

  temp_s[2].center = make_float3(0, -10004, -20);
  temp_s[2].radius = 10000;
  temp_s[2].color = make_float3(0.2, 0.2, 0.2);
  temp_s[2].emissionColor = make_float3(0, 0, 0);
  temp_s[2].transparency = 0;
  temp_s[2].reflection = 0;

  temp_s[3].center = make_float3(5, -1, -15);
  temp_s[3].radius = 2;
  temp_s[3].color = make_float3(0.9, 0.76, 0.46);
  temp_s[3].emissionColor = make_float3(0, 0, 0);
  temp_s[3].transparency = 0;
  temp_s[3].reflection = 1;

  temp_s[4].center = make_float3(5, 0, -25);
  temp_s[4].radius = 3;
  temp_s[4].color = make_float3(0.65, 0.77, 0.97);
  temp_s[4].emissionColor = make_float3(0, 0, 0);
  temp_s[4].transparency = 0;
  temp_s[4].reflection = 1;

  temp_s[5].center = make_float3(-5.5, -0, -15);
  temp_s[5].radius = 3;
  temp_s[5].color = make_float3(0.9, 0.9, 0.9);
  temp_s[5].emissionColor = make_float3(0, 0, 0);
  temp_s[5].transparency = 0;
  temp_s[5].reflection = 1;
  for(int i=0; i<size; i++){
	printf("x,y,z: %f, %f, %f, radius: %f\n", temp_s[i].center.x, temp_s[i].center.y,temp_s[i].center.z,temp_s[i].radius);
  }
  size_t sz = size*sizeof(Sphere);
  cudaMemcpyToSymbol(s, temp_s, sz, size_t(0), cudaMemcpyHostToDevice);
 
  free(temp_s);
}


static void error_callback(int error, const char* description)
{
    fprintf(stderr, "Error: %s\n", description);
}

static void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
        glfwSetWindowShouldClose(window, GLFW_TRUE);
}


GLuint vbo;
struct cudaGraphicsResource *cuda_vbo_resource;
void *d_vbo_buffer = NULL;

void launch_kernel(vertexColor *pos)
{
    dim3 block(8, 8, 1);
    dim3 grid(window_width / block.x, window_height / block.y, 1);
    tracer_kernel<<< grid, block>>>(pos);
}

void runCuda(struct cudaGraphicsResource **vbo_resource)
{
        vertexColor *dptr;
        cudaGraphicsMapResources(1, vbo_resource, 0);
        size_t num_bytes;
        cudaGraphicsResourceGetMappedPointer((void **)&dptr, &num_bytes,
                                                         *vbo_resource);
        launch_kernel(dptr);

        cudaGraphicsUnmapResources(1, vbo_resource, 0);
}

void createVBO(GLuint *vbo, struct cudaGraphicsResource **vbo_res, unsigned int vbo_res_flags)
{
        //Create vertex buffer object
        glGenBuffers(1, vbo);
        glBindBuffer(GL_ARRAY_BUFFER, *vbo);
        //Initialize VBO
        glBufferData(GL_ARRAY_BUFFER, window_width*window_height*sizeof(vertexColor), 0, GL_DYNAMIC_DRAW);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        //Register VBO with CUDA
        cudaGraphicsGLRegisterBuffer(vbo_res, *vbo, vbo_res_flags);
}

int main(void)
{   
    //------ InitGL---------------//
    GLFWwindow* window;

    glfwSetErrorCallback(error_callback);

    if (!glfwInit())
        exit(EXIT_FAILURE);

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);

    window = glfwCreateWindow(window_width, window_height, "Simple example", NULL, NULL);
    if (!window)
    {   
        glfwTerminate();
        exit(EXIT_FAILURE);
    }
    glfwSetKeyCallback(window, key_callback);

    glfwMakeContextCurrent(window);
    glewInit();
    glfwSwapInterval(1);

    glClearColor(0.0, 0.0, 0.0, 1.0);
    glDisable(GL_DEPTH_TEST);

    glViewport(0,0,window_width, window_height);
    //----------InitGL--------------//

    cudaGLSetGLDevice(0);

    createVBO(&vbo, &cuda_vbo_resource, cudaGraphicsMapFlagsWriteDiscard);

    init_kernel();
    runCuda(&cuda_vbo_resource);

    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glVertexPointer(2, GL_FLOAT, sizeof(float)*5, 0);
    glColorPointer(3, GL_FLOAT,sizeof(float)*5, (GLvoid*)(sizeof(float)*2));
    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_COLOR_ARRAY);

    while (!glfwWindowShouldClose(window))
    {
        glClear(GL_COLOR_BUFFER_BIT);

        glDrawArrays(GL_POINTS, 0, window_width * window_height);

        glfwSwapBuffers(window);
        glfwPollEvents();
    }
    glfwDestroyWindow(window);

    glfwTerminate();
    exit(EXIT_SUCCESS);
}

