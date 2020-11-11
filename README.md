
Lance Tan

CPSC 578 Computer Graphics: Final Project

5/2/2020

# Video

https://youtu.be/HCxaVzXnY3g

# Compiling and running program:

- To compile the program, type `make`. This generates the final executable, called "render".

- To generate a demo frame, type `./render demo`. This creates the image "DEMO.ppm" (This might take up to ~5 seconds.)

- To render the entire movie, type `./render`. The frames are generated as "frames/frame.XXXX.ppm", where XXXX is the frame number. WARNING: This takes several hours to complete.

- To render a subsection of the movie, type `./render (start) (end) (skip)`. This renders every _skip_ frame from _start_ (inclusive) thru _end_ (exclusive).

- If you can't open PPM files, you can convert them to another format using [Mogrify](https://imagemagick.org/script/mogrify.php).

# Effects

 - Glossy reflection: 

   The mirror on the left side of the scene is glossy. When a ray hits the mirror, instead of reflecting the ray perfectly, N different rays are generated at a small offset from the perfect reflection angle, and the colors of these distributed rays are averaged together. The parameter N is the square of the value of `#define N_SAMPLES`. Reducing this number makes rendering run faster.

- Soft shadows:

  The spotlight focused on the stick figure is also a soft shadow. To create this effect, I assigned a direction vector to the light. Then, during distributed ray tracing, I sampled shadow rays from the light the normal way, and added a rule where if the dot product between the sampled shadow ray and the spotlight's direction is > -cos(7 degrees), then treat the ray as if it was shadowed out. This has the effect of a soft shadow on everything not in a 7 degrees wide cone emanating from the light source.

 - Perlin noise texture: 

   To generate the wood floor texture at texcoord (u, v), I generated 2D Perlin noise at (u,v), treated the sample as an angle, and generated a random 2D rotation matrix using this angle. I then multiplied (u,v) by this rotation matrix, then applied a sinusoidal function to the result's x coordinate. The result is a stripey/swirly pattern with a clear direction, like wood grain.

   I borrowed this approach from the following source (which used value noise instead of Perlin noise):
   https://thebookofshaders.com/11/
   https://thebookofshaders.com/edit.php#11/wood.frag

   The code for generating Perlin noise (perlinNoise.cpp) is from Ken Perlin's website:
   https://mrl.nyu.edu/~perlin/doc/oscar.html

 - Shading models:

   The scene's ceiling, three walls, the pillars on the back wall and the red spheres on the left wall use Oren-Nayar shading, which is intended to model rough surfaces like clay or plaster. The other surfaces use Phong shading, which models smooth surfaces like plastic.


# Comments and reflection

Oren-Nayar shading was easy to get working--I followed the GLSL code posted on Canvas. Ray tracing and object intersection was also easy--it was straightforward to adapt my code from HW4.

Cylinder intersection was hard to get working, especially for the stick figure, which also provides a scaling matrix for each bone that I found was safe to ignore. Perlin noise was also hard to get working--I was careless and didn't read Ken Perlin's code before using it, and then ran into bugs when trying to initialize the noise generator in a reproducible way.

If I were to do this again I would pay more attention to the layout of my scene early on--I had initially tried to set my movie on a deserted island, before realizing that there aren't enough features on a deserted island to display all the ray tracing features that were required. I would have spent more time drawing/visualizing/making diagrams of the scene instead of attempting to place objects via trial and error.

# Credits

Motion capture data is from the [CMU Motion Capture Database](http://mocap.cs.cmu.edu/).

