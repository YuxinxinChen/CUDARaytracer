#include <cstdlib>
#include <cstdio>
#include <cmath>
#include <fstream>
#include <vector>
#include <iostream>
#include <cassert>
#include "3DBVH.h"
#include <queue>

#if defined __linux__ || defined __APPLE__
// "Compiled for Linux
#else
// Windows doesn't define these values by default, Linux does
#define M_PI 3.141592653589793
#define INFINITY 1e8
#endif

#define MAX_RAY_DEPTH 5

float mix(const float &a, const float &b, const float &mix)
{
    return b * mix + a * (1 - mix);
}


bool BBoxIntersectTest(Vec3f bbox_min, Vec3f bbox_max, Vec3f origin, Vec3f direct)
{
        Vec3f invdir = Vec3f(1.0/direct.x, 1.0/direct.y, 1.0/direct.z);
        float tmin, tmax, tymin, tymax, tzmin, tzmax;
        if(invdir.x>0){
                tmin = (bbox_min.x - origin.x)*invdir.x;
                tmax = (bbox_max.x - origin.x)*invdir.x;
        }
        else{
                tmin = (bbox_max.x - origin.x)*invdir.x;
                tmax = (bbox_min.x - origin.x)*invdir.x;
        }
        if(invdir.y>0){
                tymin = (bbox_min.y - origin.y)*invdir.y;
                tymax = (bbox_max.y - origin.y)*invdir.y;
        }else{
                tymin = (bbox_max.y - origin.y)*invdir.y;
                tymax = (bbox_min.y - origin.y)*invdir.y;
        }

        if((tmin > tymax) || (tymin > tmax))
                return false;
        if(tymin > tmin)
                tmin = tymin;
        if(tymax < tmax)
                tmax = tymax;

        if(invdir.z>0){
                tzmin = (bbox_min.z - origin.z)*invdir.z;
                tzmax = (bbox_max.z - origin.z)*invdir.z;
        }else{
                tzmin = (bbox_max.z - origin.z)*invdir.z;
                tzmax = (bbox_min.z - origin.z)*invdir.z;
        }

        if((tmin > tzmax) || (tzmin > tmax))
                return false;
        if(tzmin > tmin)
                tmin = tzmin;
        if(tzmax < tmax)
                tmax = tzmax;

        return true;
}

int BVHIntersectTest(Node *root, Vec3f origin, Vec3f direct, const std::vector<Sphere> &spheres, const Sphere **sphere, float &tnear)
{
        Node * stack[64];
        Node ** stackPtr = stack;
        *stackPtr++ = NULL;
        Node *node  = root;
        std::queue<int> list;

        do{
                Node *childL = node->childA;
                Node *childR = node->childB;
                bool overlapL = BBoxIntersectTest(childL->bbox_min, childL->bbox_max, origin, direct);
                bool overlapR = BBoxIntersectTest(childR->bbox_min, childR->bbox_max, origin, direct);
                if(overlapL && childL->isLeaf)
                        list.push(childL->objectID);
                if(overlapR && childR->isLeaf)
                        list.push(childR->objectID);

                bool traverseL = (overlapL && !childL->isLeaf);
                bool traverseR = (overlapR && !childR->isLeaf);

                if(!traverseL && !traverseR)
                        node = *--stackPtr;
                else{
                        node = (traverseL)?childL:childR;
                        if(traverseL && traverseR)
                                *stackPtr++ = childR;
                }
          }
        while(node!=NULL);

        int sz = list.size();
	int intersectIdx = -1;
        while(!list.empty())
        {
                int idx = list.front();
                list.pop();
                float t0 = INFINITY, t1 = INFINITY;
                if(spheres[idx].intersect(origin, direct, t0, t1)){
                        if(t0 < 0) t0 = t1;
                        if(t0 < tnear){
                                tnear = t0;
                                *sphere = &spheres[idx];
				intersectIdx = idx;
                        }
                }
        }
	return intersectIdx;
}

Vec3f trace(
    const Vec3f &rayorig,
    const Vec3f &raydir,
    const std::vector<Sphere> &spheres,
    const int &depth,
	  Node *root)
{
    float tnear = INFINITY;
    const Sphere* sphere = NULL;
    int intersectIdx = BVHIntersectTest(root, rayorig,raydir,spheres, &sphere, tnear);
    if (!sphere) return Vec3f(2);
    Vec3f surfaceColor = 0; // color of the ray/surfaceof the object intersected by the ray
    Vec3f phit = rayorig + raydir * tnear; // point of intersection
    Vec3f nhit = phit - sphere->center; // normal at the intersection point
    nhit.normalize(); // normalize normal direction
    // If the normal and the view direction are not opposite to each other
    // reverse the normal direction. That also means we are inside the sphere so set
    // the inside bool to true. Finally reverse the sign of IdotN which we want
    // positive.
    float bias = 1e-4; // add some bias to the point from which we will be tracing
    bool inside = false;
    if (raydir.dot(nhit) > 0) nhit = -nhit, inside = true;
    if ((sphere->transparency > 0 || sphere->reflection > 0) && depth < MAX_RAY_DEPTH) {
        float facingratio = -raydir.dot(nhit);
        // change the mix value to tweak the effect
        float fresneleffect = mix(pow(1 - facingratio, 3), 1, 0.1);
        // compute reflection direction (not need to normalize because all vectors
        // are already normalized)
        Vec3f refldir = raydir - nhit * 2 * raydir.dot(nhit);
        refldir.normalize();
        Vec3f reflection = trace(phit + nhit * bias, refldir, spheres, depth + 1, root);
        Vec3f refraction = 0;
        // if the sphere is also transparent compute refraction ray (transmission)
        if (sphere->transparency) {
            float ior = 1.1, eta = (inside) ? ior : 1 / ior; // are we inside or outside the surface?
            float cosi = -nhit.dot(raydir);
            float k = 1 - eta * eta * (1 - cosi * cosi);
            Vec3f refrdir = raydir * eta + nhit * (eta *  cosi - sqrt(k));
            refrdir.normalize();
            refraction = trace(phit - nhit * bias, refrdir, spheres, depth + 1, root);
        }
        // the result is a mix of reflection and refraction (if the sphere is transparent)
        surfaceColor = (
            reflection * fresneleffect +
            refraction * (1 - fresneleffect) * sphere->transparency) * sphere->surfaceColor;
    }
    else {
        // it's a diffuse object, no need to raytrace any further
        for (unsigned i = 0; i < spheres.size(); ++i) {
            if (spheres[i].emissionColor.x > 0) {
                // this is a light
                Vec3f transmission = 1;
                Vec3f lightDirection = spheres[i].center - phit;
                lightDirection.normalize();
		float tmp1 = -1; const Sphere *tmp_sphere=NULL;
		int tmp2 = BVHIntersectTest(root, phit + nhit * bias, lightDirection, spheres, &tmp_sphere, tmp1);
		if(tmp1 >=0 && tmp2!=i)
			transmission = 0;
     
                surfaceColor += sphere->surfaceColor * transmission *
                std::max(float(0), nhit.dot(lightDirection)) * spheres[i].emissionColor;
            }
        }
    }
    
    return surfaceColor + sphere->emissionColor;
}

//[comment]
// Main rendering function. We compute a camera ray for each pixel of the image
// trace it and return a color. If the ray hits a sphere, we return the color of the
// sphere at the intersection point, else we return the background color.
//[/comment]
void render(const std::vector<Sphere> &spheres)
{
    Node *root;
    int sz = spheres.size();
    std::vector<Leaf3D> leafs3D;
    int sortedMortonCode[sz];
    int sortedObjectIDs[sz];
    for(int i=0; i<sz; i++)
        {
                leafs3D.push_back(Leaf3D(spheres[i], i));
        }
    updateLeaf3D(leafs3D);
    for(int i=0; i<sz; i++)
        {
                sortedMortonCode[i] = leafs3D[i].position;
                sortedObjectIDs[i] = i;
        }
    radixsort(sortedMortonCode, sortedObjectIDs, sz); 
    root = generateHierarchy(sortedMortonCode, sortedObjectIDs, sz, leafs3D);
    unsigned width = 512, height = 512;
    Vec3f *image = new Vec3f[width * height], *pixel = image;
    float invWidth = 1 / float(width), invHeight = 1 / float(height);
    float fov = 90, aspectratio = width / float(height);
    float angle = tan(M_PI * 0.5 * fov / 180.);
    // Trace rays
    for (unsigned y = 0; y < height; ++y) {
        for (unsigned x = 0; x < width; ++x, ++pixel) {
            float xx = (2 * ((x + 0.5) * invWidth) - 1) * angle * aspectratio;
            float yy = (1 - 2 * ((y + 0.5) * invHeight)) * angle;
            Vec3f raydir(xx, yy, -1);
            raydir.normalize();
            *pixel = trace(Vec3f(0), raydir, spheres, 0, root);
        }
    }
    // Save result to a PPM image (keep these flags if you compile under Windows)
    std::ofstream ofs("./untitled.ppm", std::ios::out | std::ios::binary);
    ofs << "P6\n" << width << " " << height << "\n255\n";
    for (unsigned i = 0; i < width * height; ++i) {
        ofs << (unsigned char)(std::min(float(1), image[i].x) * 255) <<
               (unsigned char)(std::min(float(1), image[i].y) * 255) <<
               (unsigned char)(std::min(float(1), image[i].z) * 255);
    }
    ofs.close();
    delete [] image;
}

//[comment]
// In the main function, we will create the scene which is composed of 5 spheres
// and 1 light (which is also a sphere). Then, once the scene description is complete
// we render that scene, by calling the render() function.
//[/comment]
int main(int argc, char **argv)
{
    srand48(13);
    std::vector<Sphere> spheres;
    // position, radius, surface color, reflectivity, transparency, emission color
//    spheres.push_back(Sphere(Vec3f( 0.0, -10004, -20), 10000, Vec3f(0.20, 0.20, 0.20), 0, 0));
    spheres.push_back(Sphere(Vec3f( 0.0,      0, -20),     4, Vec3f(1.00, 0.32, 0.36), 1, 0.5));
    spheres.push_back(Sphere(Vec3f( 5.0,     -1, -15),     2, Vec3f(0.90, 0.76, 0.46), 0, 0.0));
    spheres.push_back(Sphere(Vec3f( 5.0,      0, -25),     3, Vec3f(0.65, 0.77, 0.97), 1, 0.0));
    spheres.push_back(Sphere(Vec3f(-5.5,      0, -15),     3, Vec3f(0.90, 0.90, 0.90), 1, 0.0));
    // light
    spheres.push_back(Sphere(Vec3f( 0.0,     20, -30),     3, Vec3f(0.00, 0.00, 0.00), 0, 0.0, Vec3f(3)));
    render(spheres);

    return 0;
}
