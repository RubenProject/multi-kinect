CC = g++ -g -Wall -Wextra

ROOT_DIR:=$(shell dirname $(realpath $(firstword $(MAKEFILE_LIST))))

CFLAGS = -INiTE2_Include -IOpenNI2_Include -I/usr/local/include/opencv4
LIBS = -L$(ROOT_DIR) -lNiTE2 -lOpenNI2 -lglfw -lGL -lfreenect2 -lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_calib3d


OBJS = flextGL.o viewer.o main.o colorlistener.o \
	   depthlistener.o kinectapp.o frame.o body.o \
	   bodymanager.o bodylistener.o recordmanager.o \

KinectApp: $(OBJS)
	$(CC) $(CFLAGS) $(OBJS) -o KinectApp $(LIBS)

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

kinectapp.o: kinectapp.cpp
	$(CC) -c $(CFLAGS) kinectapp.cpp $(LIBS)

frame.o: frame.cpp
	$(CC) -c $(CFLAGS) frame.cpp $(LIBS)

main.o: main.cpp
	 $(CC) -c $(CFLAGS) main.cpp $(LIBS)

bodymanager.o: bodymanager.cpp
	 $(CC) -c $(CFLAGS) bodymanager.cpp $(LIBS)

body.o: body.cpp
	 $(CC) -c $(CFLAGS) body.cpp $(LIBS)

recordmanager.o: recordmanager.cpp
	 $(CC) -c $(CFLAGS) recordmanager.cpp $(LIBS)

skeleton.o: skeleton.cpp
	 $(CC) -c $(CFLAGS) skeleton.cpp $(LIBS)

clean:
	rm *.o KinectApp

