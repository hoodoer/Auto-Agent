CC = g++


# Options passed to the compiler. -w suppresses warnings
#OPTIONS = -w


# Various include directories from JSBSim's codebase
#INCLUDES = \
#	-I FlightGear/src/ \
#	-I FlightGear/src/Network \
#	-I FlightGear/src/FDM


# Source code written for this project
SOURCES = \
	autoAgent.c++ \
	fgFdmReceiver.c++ \
	fgCtrlsTransmitter.c++



all:
	${CC} ${OPTIONS} ${INCLUDES} ${SOURCES} -o autoAgent
