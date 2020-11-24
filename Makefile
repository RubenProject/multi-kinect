CC = g++ -g -Wall -Wextra

ROOT_DIR:=$(shell dirname $(realpath $(firstword $(MAKEFILE_LIST))))

CFLAGS = -INiTE2_Include -IOpenNI2_Include -I/usr/local/include/opencv4
LIBS = -L$(ROOT_DIR) -lNiTE2 -lOpenNI2 -lglfw -lGL -lfreenect2 -lopencv_core -lopencv_highgui -lopencv_imgproc


OBJS = flextGL.o viewer.o Protonect.o colorlistener.o \
	   depthlistener.o devicemanager.o frame.o body.o \
	   bodymanager.o bodylistener.o

Protonect: $(OBJS)
	$(CC) $(CFLAGS) $(OBJS) -o Protonect $(LIBS)

colorlistener.o: colorlistener.cpp
	$(CC) -c $(CFLAGS) colorlistener.cpp $(LIBS)

depthlistener.o: depthlistener.cpp
	$(CC) -c $(CFLAGS) depthlistener.cpp $(LIBS)

bodylistener.o: bodylistener.cpp
	$(CC) -c $(CFLAGS) bodylistener.cpp $(LIBS)

viewer.o: viewer.cpp
	$(CC) -c $(CFLAGS) viewer.cpp $(LIBS)

flextGL.o: flextGL.cpp
	$(CC) -c $(CFLAGS) flextGL.cpp $(LIBS)

devicemanager.o: devicemanager.cpp
	$(CC) -c $(CFLAGS) devicemanager.cpp $(LIBS)

frame.o: frame.cpp
	$(CC) -c $(CFLAGS) frame.cpp $(LIBS)

Protonect.o: Protonect.cpp
	 $(CC) -c $(CFLAGS) Protonect.cpp $(LIBS)

bodymanager.o: bodymanager.cpp
	 $(CC) -c $(CFLAGS) bodymanager.cpp $(LIBS)

body.o: body.cpp
	 $(CC) -c $(CFLAGS) body.cpp $(LIBS)


clean:
	rm *.o Protonect

