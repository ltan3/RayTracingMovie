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

// Stick-man classes
DisplaySkeleton displayer;    
Skeleton* skeleton;
Motion* motion;

int windowWidth = 640;
int windowHeight = 480;

// scene geometry
vector<VEC3> sphereCenters;
vector<float> sphereRadii;
vector<VEC3> sphereColors;

struct Triangle {
  VEC3 a; VEC3 b; VEC3 c;
};
vector<Triangle> triangles;
vector<VEC3> triangleColors;

struct Cylinder {
  MATRIX3 rotation;
  VEC3 center;
  float length;
  float radius;
};
vector<Cylinder> cylinders;
vector<VEC3> cylinderColors;

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

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
bool raySphereIntersect(const VEC3& center, 
                        const float radius, 
                        const VEC3& rayPos, 
                        const VEC3& rayDir,
                        float& t)
{
  const VEC3 op = center - rayPos;
  const float eps = 1e-8;
  const float b = op.dot(rayDir);
  float det = b * b - op.dot(op) + radius * radius;

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

bool rayTriangleIntersect(const Triangle& triangle, const VEC3& rayPos, const VEC3&rayDir, float& t) {
  VEC3 v1 = triangle.a;
  VEC3 v2 = triangle.b;
  VEC3 v3 = triangle.c; 

  MATRIX3 mat_A;
  mat_A <<
    v1[0]-v2[0], v1[0]-v3[0], rayDir[0],
    v1[1]-v2[1], v1[1]-v3[1], rayDir[1],
    v1[2]-v2[2], v1[2]-v3[2], rayDir[2];
  float denom = mat_A.determinant();
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
  float beta = mat_beta.determinant() / denom;
  MATRIX3 mat_gamma;
  mat_gamma <<
    v1[0]-v2[0], v1[0]-rayPos[0], rayDir[0],
    v1[1]-v2[1], v1[1]-rayPos[1], rayDir[1],
    v1[2]-v2[2], v1[2]-rayPos[2], rayDir[2];
  float gamma = mat_gamma.determinant() / denom;

  float alpha = 1 - beta - gamma;
  return (alpha > 0 && beta > 0 && gamma > 0);
}

bool rayCylinderIntersect(const Cylinder& cylinder, const VEC3& rayPos, const VEC3&rayDir, float& t) {
  VEC3 o_cyl = cylinder.rotation.transpose() * (rayPos - cylinder.center);
  VEC3 d_cyl = cylinder.rotation.transpose() * rayDir;

  float a = d_cyl[0] * d_cyl[0] + d_cyl[1] * d_cyl[1];
  float b = 2 * (o_cyl[0] * d_cyl[0] + o_cyl[1] * d_cyl[1]);
  float c = o_cyl[0] * o_cyl[0] + o_cyl[1] * o_cyl[1] - cylinder.radius * cylinder.radius;
  float determinant = b * b - 4 * a * c;
  
  if (determinant < 0) {
    return false;
  }

  float sqrt_determinant = sqrt(determinant);
  t = (-b - sqrt_determinant) / 2 / a;
  if (t < 0) {
    t = (-b + sqrt_determinant) / 2 / a;
  }

  if (t < 0) {
    // cylinder is behind ray
    return false;
  }

  float h = (o_cyl + t * d_cyl)[2];
  if (h < 0 || h > cylinder.length) {
    // not in height bounds
    return false;
  }

  return true;
}

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
void rayColor(const VEC3& rayPos, const VEC3& rayDir, VEC3& pixelColor) 
{
  pixelColor = VEC3(1,1,1);

  // look for intersections
  int hitID = -1;
  float tMinFound = FLT_MAX;
  for (int y = 0; y < sphereCenters.size(); y++)
  {
    float tMin = FLT_MAX;
    if (raySphereIntersect(sphereCenters[y], sphereRadii[y], rayPos, rayDir, tMin))
    { 
      // is the closest so far?
      if (tMin < tMinFound)
      {
        tMinFound = tMin;
        hitID = y;
        pixelColor = sphereColors[hitID];
      }
    }
  }

  for (int z = 0; z < triangles.size(); z++) {
    float tMin = FLT_MAX;
    if (rayTriangleIntersect(triangles[z], rayPos, rayDir, tMin)) {
      if (tMin < tMinFound) {
        tMinFound = tMin;
        hitID = z;
        pixelColor = triangleColors[z];
      }
    }
  }
  
  for (int z = 0; z < cylinders.size(); z++) {
    float tMin = FLT_MAX;
    if (rayCylinderIntersect(cylinders[z], rayPos, rayDir, tMin)) {
      if (tMin < tMinFound) {
        tMinFound = tMin;
        hitID = z;
        pixelColor = cylinderColors[z];
      }
    }
  }

  // No intersection, return white
  if (hitID == -1)
    return;
}

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
float clamp(float value)
{
  if (value < 0.0)      return 0.0;
  else if (value > 1.0) return 1.0;
  return value;
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
  VEC3 eye = VEC3(-6, 2.5, 5.2);

  // look at waist
  // vector<VEC4>& translations = displayer.translations();
  // VEC3 lookingAt = translations[1].head<3>();
  VEC3 lookingAt = VEC3(0,0.75,0);

  VEC3 up = VEC3(0,1,0);
  // VEC3 up = VEC3(0, -1, 0);

  // VEC3 eye = VEC3(0, -3, 0);
  // VEC3 lookingAt = VEC3(0,0,0);
  // VEC3 up = VEC3(0,0,1);

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
      VEC3 color;
      rayColor(eye, rayDir, color);

      // set, in final image
      ppmOut[3 * (y * xRes + x)] = clamp(color[0]) * 255.0f;
      ppmOut[3 * (y * xRes + x) + 1] = clamp(color[1]) * 255.0f;
      ppmOut[3 * (y * xRes + x) + 2] = clamp(color[2]) * 255.0f;
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

//////////////////////////////////////////////////////////////////////////////////
// Build a list of spheres in the scene
//////////////////////////////////////////////////////////////////////////////////

void buildQuad(VEC3 sw, VEC3 se, VEC3 nw, VEC3 ne, VEC3 color) {
  triangles.push_back({sw, se, ne});
  triangleColors.push_back(color);
  triangles.push_back({sw, ne, nw});
  triangleColors.push_back(color);
  // triangleColors.push_back(VEC3(0,1,0));
}

void buildScene()
{
  sphereCenters.clear();
  sphereRadii.clear();
  sphereColors.clear();
  triangles.clear();
  triangleColors.clear();
  cylinders.clear();
  cylinderColors.clear();

  // floor
  float floor_size = 3.1; // extent of floor (towards walls)
  float floor_size_back = 10; // extent of floor (towards camera)
  float height = 0;
  VEC3 color = VEC3(0xb7, 0x6f, 0x20) / 255.0;
  VEC3 sw(-floor_size_back, height, floor_size_back);
  VEC3 se(floor_size, height, floor_size_back);
  VEC3 nw(-floor_size_back, height, -floor_size);
  VEC3 ne(floor_size, height, -floor_size);
  buildQuad(sw, se, nw, ne, color);

  // ceiling
  height = 3.75;
  color = VEC3(85, 68, 43) / 255.0;
  sw = VEC3(-floor_size_back, height, floor_size_back);
  se = VEC3(floor_size, height, floor_size_back);
  nw = VEC3(-floor_size_back, height, -floor_size);
  ne = VEC3(floor_size, height, -floor_size);
  buildQuad(sw, se, nw, ne, color);

  // back wall
  height = 4;
  float depth = -3;
  float half_width = 8;
  color = VEC3(0,0,1);
  sw = VEC3(-half_width, 0, depth);
  se = VEC3(half_width, 0, depth);
  nw = VEC3(-half_width, height, depth);
  ne = VEC3(half_width, height, depth);
  buildQuad(sw, se, nw, ne, color);

  // back wall pillar
  MATRIX3 rotate_vertical;
  rotate_vertical << 
    1, 0, 0,
    0, 0, 1,
    0, -1, 0;
  VEC3 center = VEC3(-2, -1, -3);
  float radius = 0.3;
  cylinders.push_back({rotate_vertical, center, 5, radius});
  cylinderColors.push_back(VEC3(0,1,0));

  // left wall
  depth = 2;
  color = VEC3(1,0,1);
  sw = VEC3(depth, 0, half_width);
  se = VEC3(depth, 0, -half_width);
  nw = VEC3(depth, height, half_width);
  ne = VEC3(depth, height, -half_width);
  buildQuad(sw, se, nw, ne, color);

  // left wall mirror
  height = 2.5;
  depth = 1.99;
  color = VEC3(51,204,255)/255.0;
  sw = VEC3(depth, 0, half_width);
  se = VEC3(depth, 0, -half_width);
  nw = VEC3(depth, height, half_width);
  ne = VEC3(depth, height, -half_width);
  buildQuad(sw, se, nw, ne, color);

  // spheres on left wall, back to front
  center = VEC3(2,3.125,-3);
  float sphere_radius = 0.15;
  color = VEC3(1,0,0);
  for (int i = 0; i < 5; i++) {
    center += VEC3(0,0,1.5);
    sphereCenters.push_back(center);
    sphereRadii.push_back(sphere_radius);
    sphereColors.push_back(color);
  }

  // skeleton

  displayer.ComputeBonePositions(DisplaySkeleton::BONES_AND_LOCAL_FRAMES);
  // retrieve all the bones of the skeleton
  vector<MATRIX4>& rotations = displayer.rotations();
  vector<MATRIX4>& scalings  = displayer.scalings();
  vector<VEC4>& translations = displayer.translations();
  vector<float>& lengths     = displayer.lengths();


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
    Cylinder c = {rotation.topLeftCorner<3,3>(), translation.head<3>(), lengths[x], 0.01};
    cylinders.push_back(c);
    cylinderColors.push_back(VEC3(1,0,0));

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

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
int main(int argc, char** argv)
{
  // sphereCenters.clear();
  // sphereRadii.clear();
  // sphereColors.clear();
  // triangles.clear();
  // triangleColors.clear();
  // cylinders.clear();
  // cylinderColors.clear();

  // MATRIX3 rotate;
  // rotate << 
  //   1, 0, 0,
  //   0, 1, 0,
  //   0, 0, 1;
  // Cylinder c = {rotate, VEC3(0,0,0), 50000, 0.5};

  // float t;
  // VEC3 eye_dir = -Geye.normalized();
  // rayCylinderIntersect(c, Geye, eye_dir, t);
  // return 0;

  ////  debug end

  int frame_start = 0;
  int frame_end = 300;
  int frame_skip = 1;
  if (argc == 4) {
    frame_start = stoi(argv[1]);
    frame_end = stoi(argv[2]);
    frame_skip = stoi(argv[3]);
    if ( frame_start < 0 || frame_end > 300 || frame_end < frame_start ) {
      printf("Bad start and end frame numbers\n");
      exit(1);
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
    int frame_num = i - frame_start;
    float time = (float)i / 300;

    setSkeletonsToSpecifiedFrame(x);
    buildScene();

    char buffer[256];
    sprintf(buffer, "./frames/frame.%04i.ppm", frame_num);
    renderImage(time, windowWidth, windowHeight, buffer);
    printf("Rendered sample %d as frame %d, t=%.04f\n", x, frame_num, time);
  }

  return 0;
}
