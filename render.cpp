#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <iostream>
#include <float.h>
#include "SETTINGS.h"
#include "skeleton.h"
#include "displaySkeleton.h"
#include "motion.h"

using namespace std;

// #define IGNORE_GLOSSY_REFLECTION
#define N_SAMPLES (3) // actually sqrt(number of samples). 0 = don't do distributed ray tracing

/********* Settings *********/

#define EPSILON (0.05)
#define PHONG_EXP (10.0)
#define OREN_NAYAR_ROUGHNESS (0.2)
#define RAYCOLOR_MAX_RECURSE (10)

int windowWidth = 640;
int windowHeight = 480;
VEC3 bg_color = VEC3(1,1,1);

/********* Stick figure classes, primitives (global vars) *********/

// Stick-man classes
DisplaySkeleton displayer;    
Skeleton* skeleton;
Motion* motion;

enum SurfaceType {
  Phong, OrenNayar, Reflect, Wood
};

// scene geometry
struct Sphere {
  VEC3 color;
  SurfaceType type;
  VEC3 center;
  float radius;
};
vector<Sphere> spheres;

struct Triangle {
  VEC3 color; SurfaceType type;
  VEC3 a; VEC3 b; VEC3 c;
  VEC2 ta; VEC2 tb; VEC2 tc;
};
vector<Triangle> triangles;

struct Cylinder {
  VEC3 color; SurfaceType type;
  MATRIX3 rotation;
  VEC3 center;
  float length;
  float radius;
};
vector<Cylinder> cylinders;


struct Light {
  VEC3 point; 
  VEC3 color;
  VEC3 spotlight_dir; // direction that spotlight is pointing
  bool is_spotlight; // true if it is a spotlight
  float size; // soft shadows parameter
};
vector<Light> lights;

struct Ray {
  VEC3 org; // origin
  VEC3 dir; // direction
};

// Struct that describes an intersection between a ray and the scene.
struct Intersection {
  VEC3 point; // intersection point
  VEC3 normal; // normal vector to primitive at intersection point
  VEC3 primitive_color;
  SurfaceType type;
};

/**************************************************/

void rayColor(const Ray& ray, VEC3& pixelColor, int recurse_depth);

inline double rand_double() {
  return (double) random()/(double)RAND_MAX;
}

/********* Wood texture using Perlin noise*********/

float noise2(float vec[2]);
void init_perlin();

inline float perlin_noise(const VEC2& p) {
  float vec[2] = {p[0], p[1]};
  return noise2(vec);
}

inline float smoothstep(const float x, const float x0, const float x1) {
  float t = (x - x0) / (x1 - x0);
  t = max(min(t,1.0f), 0.f);
  t = t * t * (3.0 - 2.0 * t);
  return t;
}

inline float wood_texture(const VEC2& p) {
  // stretch to imitate wood grain
  VEC2 pos = VEC2(10.0 * p[0], 3.0 * p[1]);
  
  // rotate noisily
  float angle = perlin_noise(pos) * 0.9;
  MATRIX2 rotation;
  rotation <<
    cos(angle), -sin(angle),
    sin(angle),  cos(angle);
  pos = rotation * pos * 10.0;

  // look up in texture: a sine wave
  float t = 0.5 * sin(M_PI * pos[0]) + 0.5;
  // make the bright parts wider
  t = smoothstep(t, 0.0, 0.75);
  return t;
}


//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
void writePPM(const string& filename, int& xRes, int& yRes, const float* values)
{
  int totalCells = xRes * yRes;
  unsigned char* pixels = new unsigned char[3 * totalCells];
  for (int i = 0; i < 3 * totalCells; i++)
    pixels[i] = values[i];

  FILE *fp;
  fp = fopen(filename.c_str(), "wb");
  if (fp == NULL)
  {
    cout << " Could not open file \"" << filename.c_str() << "\" for writing." << endl;
    cout << " Make sure you're not trying to write from a weird location or with a " << endl;
    cout << " strange filename. Bailing ... " << endl;
    exit(0);
  }

  fprintf(fp, "P6\n%d %d\n255\n", xRes, yRes);
  fwrite(pixels, 1, totalCells * 3, fp);
  fclose(fp);
  delete[] pixels;
}


/********* Ray-scene intersection *********/

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
bool raySphereIntersect(const Sphere& sphere,
                        const Ray& ray, double& t)
{
  const VEC3& rayPos = ray.org;
  const VEC3& rayDir = ray.dir;

  const VEC3 op = sphere.center - rayPos;
  const double eps = 1e-8;
  const double b = op.dot(rayDir);
  double det = b * b - op.dot(op) + sphere.radius * sphere.radius;

  // determinant check
  if (det < 0) 
    return false; 
  
  det = sqrt(det);
  t = b - det;
  if (t <= eps)
  {
    t = b + det;
    if (t <= eps)
      t = -1;
  }

  if (t < 0) return false;  
  return true;
}

bool rayTriangleIntersect(const Triangle& triangle, const Ray& ray, double& t, VEC2& texcoord) {
  
  const VEC3& rayPos = ray.org;
  const VEC3& rayDir = ray.dir;
  const VEC3& v1 = triangle.a;
  const VEC3& v2 = triangle.b;
  const VEC3& v3 = triangle.c; 

  MATRIX3 mat_A;
  mat_A <<
    v1[0]-v2[0], v1[0]-v3[0], rayDir[0],
    v1[1]-v2[1], v1[1]-v3[1], rayDir[1],
    v1[2]-v2[2], v1[2]-v3[2], rayDir[2];
  double denom = mat_A.determinant();
  // check if denom == 0?

  MATRIX3 mat_t;
  mat_t <<
    v1[0]-v2[0], v1[0]-v3[0], v1[0]-rayPos[0],
    v1[1]-v2[1], v1[1]-v3[1], v1[1]-rayPos[1],
    v1[2]-v2[2], v1[2]-v3[2], v1[2]-rayPos[2];
  t = mat_t.determinant() / denom;
  // printf("denom = %f\n", denom);
  // printf("t = %f\n", t);
  
  if (t < 0) {
    return false;
  }

  MATRIX3 mat_beta;
  mat_beta <<
    v1[0]-rayPos[0], v1[0]-v3[0], rayDir[0],
    v1[1]-rayPos[1], v1[1]-v3[1], rayDir[1],
    v1[2]-rayPos[2], v1[2]-v3[2], rayDir[2];
  double beta = mat_beta.determinant() / denom;
  MATRIX3 mat_gamma;
  mat_gamma <<
    v1[0]-v2[0], v1[0]-rayPos[0], rayDir[0],
    v1[1]-v2[1], v1[1]-rayPos[1], rayDir[1],
    v1[2]-v2[2], v1[2]-rayPos[2], rayDir[2];
  double gamma = mat_gamma.determinant() / denom;

  double alpha = 1 - beta - gamma;
  if (alpha > 0 && beta > 0 && gamma > 0) {
    texcoord = alpha * triangle.ta + beta * triangle.tb + gamma * triangle.tc;
    return true;
  }
  return false;
}

bool rayCylinderIntersect(const Cylinder& cylinder, const Ray& ray, double& t, VEC3& point, VEC3& normal) {
  
  const VEC3& rayPos = ray.org;
  const VEC3& rayDir = ray.dir;

  // In cylinder frame, cylinder axis is coexistient with positive z axis

  VEC3 o_cyl = cylinder.rotation.transpose() * (rayPos - cylinder.center);
  VEC3 d_cyl = cylinder.rotation.transpose() * rayDir;

  double a = d_cyl[0] * d_cyl[0] + d_cyl[1] * d_cyl[1];
  double b = 2 * (o_cyl[0] * d_cyl[0] + o_cyl[1] * d_cyl[1]);
  double c = o_cyl[0] * o_cyl[0] + o_cyl[1] * o_cyl[1] - cylinder.radius * cylinder.radius;
  double determinant = b * b - 4 * a * c;
  
  if (determinant < 0) {
    return false;
  }

  double sqrt_determinant = sqrt(determinant);
  t = (-b - sqrt_determinant) / 2 / a;
  if (t < 0) {
    t = (-b + sqrt_determinant) / 2 / a;
  }

  if (t < 0) {
    // cylinder is behind ray
    return false;
  }

  double h = (o_cyl + t * d_cyl)[2];
  if (h < 0 || h > cylinder.length) {
    // not in height bounds
    return false;
  }

  VEC3 p_cyl = o_cyl + t * d_cyl;
  VEC3 n_cyl = p_cyl;
  n_cyl[2] = 0;
  point = cylinder.rotation * p_cyl + cylinder.center;
  normal = cylinder.rotation * n_cyl;
  normal.normalize();
  return true;
}

// ray (in): ray to intersect with scene
// returns: true if intersection with scene, false o/w
// intersection (out): struct describing intersection with primitive
bool intersectScene(const Ray& ray, Intersection& intersection) {

  int hitID = -1;
  Intersection intersectionFound;
  double t;
  double tMinFound = DBL_MAX;
  for (int y = 0; y < spheres.size(); y++)
  {
    if (raySphereIntersect(spheres[y], ray, t))
    { 
      // is the closest so far?
      if (t < tMinFound)
      {
        tMinFound = t;
        hitID = y;
        intersection.primitive_color = spheres[hitID].color;
        intersection.point = ray.org + t * ray.dir;
        intersection.normal = (intersection.point - spheres[y].center).normalized();
        intersection.type = spheres[y].type;
      }
    }
  }

  for (int z = 0; z < triangles.size(); z++) {
    VEC2 texcoord;
    if (rayTriangleIntersect(triangles[z], ray, t, texcoord)) {
      if (t < tMinFound) {
        tMinFound = t;
        intersection = intersectionFound;
        hitID = z;

        Triangle& triangle = triangles[z];
        intersection.point = ray.org + t * ray.dir;
        intersection.normal = (triangle.b - triangle.a).cross(triangle.c - triangle.a); // IS THIS THE RIGHT ORDER??
        intersection.normal.normalize();
        intersection.type = triangle.type;

        if (intersection.type == Wood) {
          float value = wood_texture(texcoord);
          VEC3 color1 = VEC3(189, 140, 106) / 255.0;
          VEC3 color2 = VEC3(77, 48, 24) / 255.0;
          intersection.primitive_color = value * color1 + (1-value) * color2;
        }
        else {
          intersection.primitive_color = triangle.color;
        }
      }
    }
  }
  
  for (int z = 0; z < cylinders.size(); z++) {
    VEC3 point, normal;
    if (rayCylinderIntersect(cylinders[z], ray, t, point, normal)) {
      if (t < tMinFound) {
        tMinFound = t;
        intersection = intersectionFound;
        hitID = z;
        intersection.primitive_color = cylinders[z].color;
        intersection.point = point;
        intersection.normal = normal;
        intersection.type = cylinders[z].type;
      }
    }
  }

  return (hitID != -1);
}

/********* Ray color *********/

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////

// Constrains color to be in interval [0.0, 1.0]
inline void clampColor(VEC3& color) {
  color[0] = max(min(color[0], 1.0), 0.0);
  color[1] = max(min(color[1], 1.0), 0.0);
  color[2] = max(min(color[2], 1.0), 0.0);
}

inline void offsetRay(Ray& ray) {
  ray.org = ray.org + EPSILON * ray.dir;
}

inline VEC3 reflect(const VEC3& n, const VEC3& l) {
  return -l + 2 * l.dot(n) * n;
}

inline VEC3 elementProduct(const VEC3& u, const VEC3& v) {
  return VEC3(u[0] * v[0], u[1] * v[1], u[2] * v[2]);
}

inline void orthogonalize(VEC3& r, VEC3& u, VEC3& v) {
  r.normalize();
  u = VEC3(1,0,0);
  u = u - r.dot(u) * r;
  u.normalize();
  v = r.cross(u);
}

// Draw n^2 stratified samples from the square [-0.5, 0.5] x [-0.5, 0.5].
// If n=0, then put just (0,0) in the samples list for completeness.
void jitterSamples(const int n, vector<VEC2>& samples) {
  if (n == 0) {
    samples.push_back(VEC2(0,0));
    return;
  }

  for (int i = 0; i < n; i++) {
    for (int j = 0; j < n; j++) {
      samples.push_back(VEC2(
        -0.5 + ((double)i + rand_double())/(double)n,
        -0.5 + ((double)j + rand_double())/(double)n
      ));
    }
  }
}

// NAMES of rays:
// n: outward facing normal
// l: direction to light (pointing away from intersection pt)
// e: direction from pt to eye
// r: light reflection direction (away from pt)

VEC3 shadePhong(const VEC3& light_color, const VEC3& primitive_color, const VEC3& n, const VEC3& l, const VEC3& e) {
  VEC3 r = reflect(n, l);
  return elementProduct(light_color, primitive_color) * (max(0.0, (double)n.dot(l)) + pow(max(0.0, (double)r.dot(e)), PHONG_EXP));
}

VEC3 shadeOrenNayar(const VEC3& light_color, const VEC3& primitive_color, const VEC3& n, const VEC3& l, const VEC3& e) {
  float a = 1.0 - (0.5 * OREN_NAYAR_ROUGHNESS) / (OREN_NAYAR_ROUGHNESS + 0.33);
  float b = (0.45 * OREN_NAYAR_ROUGHNESS) / (OREN_NAYAR_ROUGHNESS + 0.09);

  VEC3 col = elementProduct(light_color, primitive_color);
  float irradiance = max(0.0, l.dot(n));

  float edotn = e.dot(n);
  float ldotn = l.dot(n);
  float angleViewNormal = acos(edotn);
  float angleLightNormal = acos(ldotn);
  
  VEC3 v_pn = (e - n * edotn).normalized();
  VEC3 l_pn = (l - n * ldotn).normalized();
  float angleDiff = max(0.0, v_pn.dot(l_pn));

  float alpha = max(angleViewNormal, angleLightNormal);
  float beta = min(angleViewNormal, angleLightNormal);

  return col * irradiance * (a + b * angleDiff * sin(alpha) * tan(beta));
}

void reflectColor(const Ray& reflect_ray, const VEC3& normal, VEC3& color, int recurse_depth) {
  int n = N_SAMPLES; // sqrt(number of samples)
  float width = 0.025; // fuzziness paramater (half the side length of target square one unit away from intersection)

  vector<VEC2> samples;
  jitterSamples(n, samples);

  // coordinate basis for target square
  VEC3 r = reflect_ray.dir;
  VEC3 u, v;
  orthogonalize(r, u, v);

  color = VEC3(0,0,0);
  for (int i = 0; i < samples.size(); i++) {
    VEC3 rand_dir = r + samples[i][0] * width * u + samples[i][1] * width * v;
    
    if (normal.dot(rand_dir) <= 0) {
      // random direction is below reflecting surface
      // printf("reflect direction below surface: n=(%.3f, %.3f, %.3f), rand_dir=(%.3f, %.3f, %.3f)\n", normal[0], normal[1], normal[2], rand_dir[0], rand_dir[1], rand_dir[2]);
      continue;
    }

    Ray rand_ray = {reflect_ray.org, rand_dir};
    offsetRay(rand_ray);
    VEC3 rand_color;
    rayColor(rand_ray, rand_color, recurse_depth + 1);
    color += rand_color / (float)samples.size();
  }
}

void rayColor(const Ray& ray, VEC3& pixelColor, int recurse_depth) {
  if (recurse_depth >= RAYCOLOR_MAX_RECURSE) {
    pixelColor = bg_color;
    return;
  }

  Intersection intersection;
  if (!intersectScene(ray, intersection)) {
    pixelColor = bg_color;
    return;
  }

  // debug: color according to direction of normal
  // pixelColor = (intersection.normal + VEC3(1,1,1))/2;
  // return;
  
  VEC3 e = -ray.dir;
  e.normalize();
  VEC3& n = intersection.normal;

  // If primitive is mirror, then reflect
  if ( intersection.type == Reflect ){
    VEC3 reflect_dir = reflect(n, e);
    Ray reflect_ray = {intersection.point, reflect_dir};

    #ifdef IGNORE_GLOSSY_REFLECTION
      offsetRay(reflect_ray);
      rayColor(reflect_ray, pixelColor, recurse_depth + 1);
    #else
      reflectColor(reflect_ray, n, pixelColor, recurse_depth);
    #endif

    return;
  }

  // Primitive is solid, go and do normal coloring

  pixelColor = VEC3(0,0,0);
  for (int i = 0; i < lights.size(); i++) {
    Light light = lights[i];

    // coordinate basis for light
    VEC3 r = light.point - intersection.point;
    VEC3 u, v;
    orthogonalize(r, u, v);

    // printf("Basis: r=(%.4f, %.4f, %.4f) u=(%.4f, %.4f, %.4f) v=(%.4f, %.4f, %.4f)\n",
    //   r[0], r[1], r[2], u[0], u[1], u[2], v[0], v[1], v[2]);

    int n_samples = N_SAMPLES;
    if (light.size == 0) {
      n_samples = 0; // don't attempt soft shadows if the light is actually a point light
    }

    vector<VEC2> samples;
    jitterSamples(n_samples, samples);
    for (int s = 0; s < samples.size(); s++) {
      VEC3 rand_light = light.point + light.size * (samples[s][0] * u + samples[s][1] * v);
      VEC3 l = rand_light - intersection.point;
      l.normalize();

      // Need to test that light actually hits the outside of primitive
      // if (n.dot(l) < 0) {
      //   continue;
      // }

      // Shadow test
      // Offset the shadow ray by a bit so it doesn't hit the intersection point we just found
      Ray shadow_ray = {intersection.point, l};
      offsetRay(shadow_ray);
      
      float spotlight_angle = (float) 7 / 180 * M_PI;
      // spotlight test
      if (lights[i].is_spotlight) {
        if (lights[i].spotlight_dir.dot(l) > -cos(spotlight_angle)) {
          continue;
        }
      }

      Intersection shadow_ray_intersection;
      if (intersectScene(shadow_ray, shadow_ray_intersection)) {
        VEC3 pt_to_primitive = intersection.point - shadow_ray_intersection.point;
        VEC3 pt_to_light = intersection.point - light.point;
        if (pt_to_primitive.squaredNorm() < pt_to_light.squaredNorm()) {
          continue;
        }
      } // end shadow test

      if (intersection.type == Phong || intersection.type == Wood) {
        pixelColor += shadePhong(light.color, intersection.primitive_color, n, l, e) / samples.size();
      }
      else if (intersection.type == OrenNayar) {
        pixelColor += shadeOrenNayar(light.color, intersection.primitive_color, n, l, e) / samples.size();
      }
    
    } // end loop over distributed samples on a light
  } // end loop over lights
  
  clampColor(pixelColor);

}


//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
void renderImage(float& time, int& xRes, int& yRes, const string& filename) 
{
  // allocate the final image
  const int totalCells = xRes * yRes;
  float* ppmOut = new float[3 * totalCells];

  // move camera position
  // VEC3 eye = VEC3(4, 0.5, 12) * (1-time) + VEC3(1, 0.5, 8) * time; 
  // VEC3 eye = VEC3(0, 10, 0);
  VEC3 eye1 = VEC3(-2, 3, 8);
  VEC3 eye2 = VEC3(-6, 1.75, 7.5);
  VEC3 eye = (1.0 - time) * eye1 + time * eye2;

  // look at waist
  // vector<VEC4>& translations = displayer.translations();
  // VEC3 lookingAt = translations[1].head<3>();
  VEC3 lookingAt1 = VEC3(0.9, 1.2, -1.9);
  VEC3 lookingAt2 = VEC3(0.0, 1.2, 2.0);
  VEC3 lookingAt = (1.0 - time) * lookingAt1 + time * lookingAt2;
  // printf("looking at: (%.5f, %.5f, %.5f)\n", lookingAt[0], lookingAt[1], lookingAt[2]);

  VEC3 up = VEC3(0,1,0);

  // compute image plane
  const float halfY = (lookingAt - eye).norm() * tan(45.0f / 360.0f * M_PI);
  const float halfX = halfY * 4.0f / 3.0f;

  const VEC3 cameraZ = (lookingAt - eye).normalized();
  const VEC3 cameraX = up.cross(cameraZ).normalized();
  const VEC3 cameraY = cameraZ.cross(cameraX).normalized();

  for (int y = 0; y < yRes; y++) 
    for (int x = 0; x < xRes; x++) 
    {
      // generate the ray, making x-axis go left to right
      const float ratioX = 1.0f - ((xRes - 1) - x) / float(xRes) * 2.0f;
      const float ratioY = 1.0f - y / float(yRes) * 2.0f;
      const VEC3 rayHitImage = lookingAt + 
                               ratioX * halfX * cameraX +
                               ratioY * halfY * cameraY;
      const VEC3 rayDir = (rayHitImage - eye).normalized();

      // get the color
      Ray ray = {eye, rayDir};
      VEC3 color;
      rayColor(ray, color, 0);

      // // VEC2 texcoord = VEC2((float)x / xRes - 0.5, (float)y / yRes - 0.5) * 2.0;
      // VEC2 texcoord = VEC2((float)x / xRes, (float)y / yRes);
      // float value = wood_texture(texcoord);
      // // float value = 0.5 + 0.5*perlin_noise(texcoord);
      // VEC3 color1 = VEC3(189, 140, 106) / 255.0;
      // VEC3 color2 = VEC3(77, 48, 24) / 255.0;
      // VEC3 color = value * color1 + (1-value) * color2;

      // set, in final image
      ppmOut[3 * (y * xRes + x)] = color[0] * 255.0f;
      ppmOut[3 * (y * xRes + x) + 1] = color[1] * 255.0f;
      ppmOut[3 * (y * xRes + x) + 2] = color[2] * 255.0f;
    }
  writePPM(filename, xRes, yRes, ppmOut);

  delete[] ppmOut;
}

//////////////////////////////////////////////////////////////////////////////////
// Load up a new motion captured frame
//////////////////////////////////////////////////////////////////////////////////
void setSkeletonsToSpecifiedFrame(int frameIndex)
{
  if (frameIndex < 0)
  {
    printf("Error in SetSkeletonsToSpecifiedFrame: frameIndex %d is illegal.\n", frameIndex);
    exit(0);
  }
  if (displayer.GetSkeletonMotion(0) != NULL)
  {
    int postureID;
    if (frameIndex >= displayer.GetSkeletonMotion(0)->GetNumFrames())
    {
      cout << " We hit the last frame! You might want to pick a different sequence. " << endl;
      postureID = displayer.GetSkeletonMotion(0)->GetNumFrames() - 1;
    }
    else 
      postureID = frameIndex;
    displayer.GetSkeleton(0)->setPosture(* (displayer.GetSkeletonMotion(0)->GetPosture(postureID)));
  }
}


/********* Build scene *********/

//////////////////////////////////////////////////////////////////////////////////
// Build a list of spheres in the scene
//////////////////////////////////////////////////////////////////////////////////

void buildQuad(VEC3 sw, VEC3 se, VEC3 nw, VEC3 ne, VEC3 color, SurfaceType type) {
  triangles.push_back((struct Triangle){color, type, sw, se, ne, VEC2(0,1), VEC2(1,1), VEC2(1,0)});
  triangles.push_back((struct Triangle){color, type, sw, ne, nw, VEC2(0,1), VEC2(1,0), VEC2(0,0)});
}

void buildScene()
{
  spheres.clear();
  triangles.clear();
  cylinders.clear();
  lights.clear();

  // init skeleton

  displayer.ComputeBonePositions(DisplaySkeleton::BONES_AND_LOCAL_FRAMES);
  // retrieve all the bones of the skeleton
  vector<MATRIX4>& rotations = displayer.rotations();
  vector<MATRIX4>& scalings  = displayer.scalings();
  vector<VEC4>& translations = displayer.translations();
  vector<float>& lengths     = displayer.lengths();

  // floor
  float floor_size = 3.1; // extent of floor (towards walls)
  float floor_size_back = 10; // extent of floor (towards camera)
  float height = 0;
  VEC3 color = VEC3(255, 162, 48) / 255.0;
  VEC3 sw(-floor_size_back, height, floor_size_back);
  VEC3 se(floor_size, height, floor_size_back);
  VEC3 nw(-floor_size_back, height, -floor_size);
  VEC3 ne(floor_size, height, -floor_size);
  buildQuad(sw, se, nw, ne, color, Wood);

  // ceiling
  height = 4.5;
  color = VEC3(255, 213, 112) / 255.0;
  sw = VEC3(-floor_size_back, height, floor_size_back);
  se = VEC3(floor_size, height, floor_size_back);
  nw = VEC3(-floor_size_back, height, -floor_size);
  ne = VEC3(floor_size, height, -floor_size);
  buildQuad(se, sw, ne, nw, color, OrenNayar); // reverse order

  // back wall
  height = 6;
  float depth = -3;
  float half_width = 8;
  color = VEC3(217, 217, 217) / 255.0;
  sw = VEC3(-half_width, 0, depth);
  se = VEC3(half_width, 0, depth);
  nw = VEC3(-half_width, height, depth);
  ne = VEC3(half_width, height, depth);
  buildQuad(sw, se, nw, ne, color, OrenNayar);

  // back wall pillars
  MATRIX3 rotate_vertical;
  rotate_vertical << 
    1, 0, 0,
    0, 0, 1,
    0, -1, 0;
  VEC3 center = VEC3(-0.333333, -1, -3);
  float radius = 0.3;
  color = VEC3(255, 213, 112) / 255.0;
  cylinders.push_back((struct Cylinder){color, OrenNayar, rotate_vertical, center, height + 1, radius});

  center = VEC3(-3.666666666, -1, -3);
  cylinders.push_back((struct Cylinder){color, OrenNayar, rotate_vertical, center, height + 1, radius});

  // left wall
  depth = 3;
  color = VEC3(217, 217, 217) / 255.0;
  sw = VEC3(depth, 0, half_width);
  se = VEC3(depth, 0, -half_width);
  nw = VEC3(depth, height, half_width);
  ne = VEC3(depth, height, -half_width);
  buildQuad(se, sw, ne, nw, color, OrenNayar); // reverse order

  // left wall mirror
  height = 2.5;
  depth = 2.995;
  float mirror_margin = 0.15;
  float mirror_east = -3 + mirror_margin;
  color = VEC3(51,204,255)/255.0;
  sw = VEC3(depth, mirror_margin, half_width);
  se = VEC3(depth, mirror_margin, mirror_east);
  nw = VEC3(depth, height, half_width);
  ne = VEC3(depth, height, mirror_east);
  buildQuad(se, sw, ne, nw, color, Reflect); // reverse order

  // spheres on left wall, back to front
  center = VEC3(3,3.125,-1.5);
  float sphere_radius = 0.15;
  color = VEC3(1,0,0);
  for (int i = 0; i < 5; i++) {
    Sphere sphere = {color, OrenNayar, center, radius};
    spheres.push_back(sphere);
    center += VEC3(0,0,1.5);
  }

  // right wall
  depth = -7;
  height = 6;
  color = VEC3(217, 217, 217) / 255.0;
  sw = VEC3(depth, 0, half_width);
  se = VEC3(depth, 0, -half_width);
  nw = VEC3(depth, height, half_width);
  ne = VEC3(depth, height, -half_width);
  buildQuad(sw, se, nw, ne, color, OrenNayar);

  // door
  depth = -6.995;
  height = 2;
  color = VEC3(0,0,1);
  float door_east = -2;
  float door_west = -1;
  sw = VEC3(depth, 0, door_west);
  se = VEC3(depth, 0, door_east);
  nw = VEC3(depth, height, door_west);
  ne = VEC3(depth, height, door_east);
  buildQuad(sw, se, nw, ne, color, Phong);

  // lights
  VEC3 spotlight_pos = VEC3(0.0, 2.4, 10.0);
  VEC3 spotlight_dir = (translations[1].head<3>() - spotlight_pos).normalized();
  lights.push_back((struct Light){
    spotlight_pos, // position
    VEC3(255, 255, 102) / 255.0, // color
    spotlight_dir,
    true,
    0.2
  });

  // red light
  lights.push_back((struct Light){
    VEC3(-1.0,3.5,5.0), // position
    VEC3(96, 48, 48) / 255.0, // color
    VEC3(0,0,0), // direction
    false, // not a spotlight
    0.1 // size
  });

  // green light
  lights.push_back((struct Light){
    VEC3(-2.3, 3.0, 5.0),
    VEC3(48, 96, 96) / 255.0,
    VEC3(0,0,0),
    false,
    0.1
  });

  // skeleton

  // VEC4& center = translations[1];
  // printf("(%0.3f, %0.3f, %0.3f)\n", center[0], center[1], center[2]);

  // skip the first bone, 
  // it's just the origin
  int totalBones = rotations.size();
  for (int x = 1; x < totalBones; x++)
  {
    MATRIX4& rotation = rotations[x];
    MATRIX4& scaling = scalings[x];
    VEC4& translation = translations[x];

    // printf("BONE %d\nrotation:\n", x);
    // cout << rotation << endl;
    // cout << "scaling" << endl;
    // cout << scaling << endl;
    // cout << "translation" << endl;
    // cout << translation << endl;
    // printf("length: %f\n\n", lengths[x]);

    // rotation, center, length, radius
    color = VEC3(96, 0, 191) / 255.0;
    Cylinder c = {color, Phong, rotation.topLeftCorner<3,3>(), translation.head<3>(), lengths[x], 0.01};
    cylinders.push_back(c);

    /*

    // get the endpoints of the cylinder
    VEC4 leftVertex(0,0,0,1);
    VEC4 rightVertex(0,0,lengths[x],1);

    leftVertex = rotation * scaling * leftVertex + translation;
    rightVertex = rotation * scaling * rightVertex + translation;

    // get the direction vector
    VEC3 direction = (rightVertex - leftVertex).head<3>();
    const float magnitude = direction.norm();
    direction *= 1.0 / magnitude;

    // how many spheres?
    const float sphereRadius = 0.05;
    const int totalSpheres = magnitude / (2.0 * sphereRadius);
    const float rayIncrement = magnitude / (float)totalSpheres;

    // store the spheres
    sphereCenters.push_back(leftVertex.head<3>());
    sphereRadii.push_back(0.05);
    sphereColors.push_back(VEC3(1,0,0));
    
    sphereCenters.push_back(rightVertex.head<3>());
    sphereRadii.push_back(0.05);
    sphereColors.push_back(VEC3(1,0,0));
    for (int y = 0; y < totalSpheres; y++)
    {
      VEC3 center = ((float)y + 0.5) * rayIncrement * direction + leftVertex.head<3>();
      sphereCenters.push_back(center);
      sphereRadii.push_back(0.05);
      sphereColors.push_back(VEC3(1,0,0));
    } 
    */
  }
}

/********* MAIN *********/

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
int main(int argc, char** argv)
{
  srandom(987654321);
  init_perlin(); // init here so same perlin noise regardless of run
  
  int frame_start = 0;
  int frame_end = 300;
  int frame_skip = 1;
  bool demo = false;

  if (argc == 4) {
    frame_start = stoi(argv[1]);
    frame_end = stoi(argv[2]);
    frame_skip = stoi(argv[3]);
    if ( frame_start < 0 || frame_end > 300 || frame_end < frame_start ) {
      printf("Bad start and end frame numbers\n");
      exit(1);
    }
  }
  if (argc == 2) {
    if (strcmp(argv[1], "demo") == 0) {
      demo = true;
      frame_start = 80;
      frame_end = 81;
      frame_skip = 1;
    }
  }

  printf("Rendering range(%d,%d,%d)\n", frame_start, frame_end, frame_skip);

  string skeletonFilename("131.asf");
  string motionFilename("131_08.amc");
  
  // load up skeleton stuff
  skeleton = new Skeleton(skeletonFilename.c_str(), MOCAP_SCALE);
  skeleton->setBasePosture();
  displayer.LoadSkeleton(skeleton);

  // load up the motion
  motion = new Motion(motionFilename.c_str(), MOCAP_SCALE, skeleton);
  displayer.LoadMotion(motion);
  skeleton->setPosture(*(displayer.GetSkeletonMotion(0)->GetPosture(0)));

  double samples_per_frame = 784.0/300;
  for (int i = frame_start; i < frame_end; i += frame_skip)
  {
    int x = i * samples_per_frame;
    int frame_num = i;
    float time = (float)i / 300;

    setSkeletonsToSpecifiedFrame(x);
    buildScene();

    if (demo) {
      renderImage(time, windowWidth, windowHeight, "./DEMO.ppm");
      printf("Rendered sample %d as frame %d, t=%.04f\n", x, frame_num, time);
      printf("DEMO: Rendered as DEMO.ppm\n");
    }
    else {
      char buffer[256];
      sprintf(buffer, "./frames/frame.%04i.ppm", frame_num);
      renderImage(time, windowWidth, windowHeight, buffer);
      printf("Rendered sample %d as frame %d, t=%.04f\n", x, frame_num, time);
    }
  }

  return 0;
}
