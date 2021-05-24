CC = g++ -g -Wall -Wextra -std=c++2a

ROOT_DIR:=$(shell dirname $(realpath $(firstword $(MAKEFILE_LIST))))

CFLAGS = -INiTE2_Include -IOpenNI2_Include -I/usr/local/include/opencv4
LIBS = -L$(ROOT_DIR) -lNiTE2 -lOpenNI2 -lglfw -lGL -lfreenect2 -lopencv_core \
		-lopencv_highgui -lopencv_imgproc -lopencv_calib3d -lopencv_imgcodecs\
		-lyaml-cpp


OBJS = flextGL.o viewer.o main.o framelistener.o \
	   kinectapp.o frame.o body.o camera.o \
	   bodymanager.o bodylistener.o recordmanager.o \
	   common.o chessboard.o logger.o

.PHONY: all run KinectApp

KinectApp: $(OBJS)
	$(CC) $(CFLAGS) $(OBJS) -o KinectApp $(LIBS)

framelistener.o: framelistener.cpp
	$(CC) -c $(CFLAGS) framelistener.cpp $(LIBS)

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

common.o: common.cpp
	 $(CC) -c $(CFLAGS) common.cpp $(LIBS)

chessboard.o: chessboard.cpp
	 $(CC) -c $(CFLAGS) chessboard.cpp $(LIBS)

logger.o: logger.cpp
	 $(CC) -c $(CFLAGS) logger.cpp $(LIBS)

camera.o: camera.cpp
	 $(CC) -c $(CFLAGS) camera.cpp $(LIBS)

all: KinectApp

run: KinectApp
	sudo ./Reset/usbreset.sh
	./KinectApp

clean:
	rm *.o KinectApp

