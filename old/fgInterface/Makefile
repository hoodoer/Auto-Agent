CC = g++


# Options passed to the compiler. -w suppresses warnings
#OPTIONS = -w


# Various include directories from JSBSim's codebase
INCLUDES = \
	-I FlightGear/src/ \
	-I FlightGear/src/Network



# These are objects from the JSBSim flight
# dynamic system. It's the flight model system.
OBJECTS-JSB = 

# Source code written for this project
SOURCES = \
	fdmDump.c++ 



all:
	${CC} ${OPTIONS} ${INCLUDES} ${OBJECTS-JSB} ${SOURCES} -o fdmDump
