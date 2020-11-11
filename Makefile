# calls:
CC         = g++
# CFLAGS     = -c -Wall -O3
CFLAGS     = -c -w -O3 # -w means no warnings
# CFLAGS = -c -w -g3
LDFLAGS    = 
# EXECUTABLE = previz render
EXECUTABLE = render

OBJECTS    = $(SOURCES:.cpp=.o)

all: $(EXECUTABLE)
	rm -f frames/*.ppm frames/movie.ppm DEMO.ppm
	
previz: previz.o skeleton.o motion.o displaySkeleton.o
	$(CC) $^ $(LDFLAGS) -o $@

render: render.o skeleton.o motion.o displaySkeleton.o perlinNoise.o
	$(CC) $^ $(LDFLAGS) -o $@

.cpp.o:
	$(CC) $(CFLAGS) $< -o $@

clean:
	rm -f *.o previz

