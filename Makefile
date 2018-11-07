include /usr/local/include/gvp/gvpMakeInclude


CC = g++



# Options passed to the compiler. -w suppresses warnings
#OPTIONS = -w


# Various include directories from JSBSim's codebase
INCLUDES = \
	-I ./                         \
	-I /usr/local/include/gvp1.0/ \
	-I utils/                     \
	-I fsm/                       \
	-I neural/                    \
	-I ${INCLUDE}                 \
	-I ${TAO_INC}                 \
	-I ${ACE_INC}                 \



# Various LIBS needed
# plib stuff for the joystick class
# Everything else for GVP interfaces
LIBS = \
	-lplibjs                    \
	-lplibul                    \
	-L ${LIBRARY}               \
	-lgvpApiClient              \
	-liorConnection             \
	-lgvpEntity                 \
	-lgvpPrototype              \
	-lgvpTimer                  \
	-lgvpMath                   \
	-lgvpIoDevice               \
	-lgvpLegend                 \
	-lDted                      \
	-lTAO_Utils                 \
	-lgsl                       \
	-lgslcblas                  \
	-lgvpLegend                 \
	-lltdl                      \
	-L${TAO_LIB}/PortableServer \
	-lTAO_PortableServer        \
	-L${TAO_LIB}                \
	-L${ACE_LIB}                \
	-lACE                       \
	-lpthread                   \
	-lreadline                  \
	-ltermcap                   


# Source code written for this project
SOURCES = \
	autoAgent.c++                \
	utils/fgFdmReceiver.c++      \
	utils/fgCtrlsTransmitter.c++ \
	utils/joystick.c++           \
	neural/neuralNet.cpp         \
	fsm/baseEntity.c++           \
	fsm/ucavStates.c++           \
	fsm/ucav.c++                 \
	pilot.c++                    \
	speech.c++


# Used for building the training system
# for the neural network.
TRAINERSOURCES = \
	utils/fgFdmReceiver.c++      \
	utils/fgCtrlsTransmitter.c++ \
	utils/joystick.c++           \
	neural/neuralNet.cpp         \
	aiTrainer.c++




all:
	${CC} ${OPTIONS} ${INCLUDES} ${SOURCES} ${LIBS} -o autoAgent


trainer:
	${CC} ${OPTIONS} ${INCLUDES} ${TRAINERSOURCES} ${LIBS} -o aiTrainer

