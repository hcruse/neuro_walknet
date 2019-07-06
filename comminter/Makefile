# The type of system this file is executed on (Mac/Linux/Windows)
UNAME := $(shell uname)

# Set the hector simulator main dir (which includes RobotSim and libraries)
HECTORDIR=../../hector/

COMMUNICATIONINTERFACEDIR=$(CURDIR)/$(HECTORDIR)/CommunicationInterface

OUTNAME=_CommunicationInterface.so

# List of source files
SRCCXX :=       Attribute.cpp\
                BfbClient.cpp\
                CommunicationInterface.cpp\
                CommunicationXmlParser.cpp\
                MathParser.cpp

# Path of the folder that contains all the sub-folders with the custom shared libraries (the ones written only for this project)
#CUSTOM_SHARED_LIB_DIR=../SharedLibraries
CUSTOM_SHARED_LIB_DIR=$(CURDIR)/$(HECTORDIR)/SharedLibraries

# Names of the shared libraries used in this project
CUSTOM_SHARED_LIBS= BfbMessage\
                    TcpConnection\
                    PugiXml

# Define the build directory
BUILDDIR=bin

# The c++ compiler that will be used to compile the source files.
CXX=g++

ifneq ($(UNAME), Darwin)
GCC_VER_GTE47 := $(shell echo `gcc -dumpversion | cut -f1-2 -d.` \>= 4.7 | sed -e 's/\./*100+/g' | bc )
ifeq ($(GCC_VER_GTE47),0)
        $(error The g++ version that is used as standard compiler is too old. At least version 4.7 must be used. You can check your version using the command "g++ --version". If you have a newer version installed but it is not your default compiler, you can change it using the script gcc-set-default-version from the hectorsim/Tools folder.)
endif
endif

# Compiler flags that influence the behaviour of the c++ compiler
# -std=c++11 tells the compiler to use the c++11 standard. This standard includes for example shared pointers in the standard library. 
# -Wall tells the compiler to show most of the warnings (-Wextra will show even more). 
# -Wno-unused-parameter additionally 
# -O2 defines the opimization level the compiler should use (other options; -O, -O0, -O1, -O2, -O3, -Os). 
# -g produces debugging information in the operating system's native format.
CXXFLAGS=-std=c++11 -DNDEBUG -fPIC -Wall -O2 -g -MD -MP

# Preprocessor flags
CPPFLAGS=$(addprefix -I${CUSTOM_SHARED_LIB_DIR}/, ${CUSTOM_SHARED_LIBS})
CPPFLAGS+= -I$(COMMUNICATIONINTERFACEDIR)

# The libraries that should be inOUTNAMEcluded
# -lm tells the compiler to include the math library. This enables the use of #include <math.h>
# -lboost... includes certain libraries from the boost collection
# -pthread includes a library used for multithreading
LDLIBS= $(addprefix -l, ${CUSTOM_SHARED_LIBS})  -lm -lboost_program_options -lboost_system -lboost_filesystem -lboost_regex -lboost_thread -pthread 

TEMP=-Wl,-rpath=${CUSTOM_SHARED_LIB_DIR}/
LDFLAGS=$(addprefix ${TEMP}, ${CUSTOM_SHARED_LIBS})\
        $(addprefix -L${CUSTOM_SHARED_LIB_DIR}/, ${CUSTOM_SHARED_LIBS})

# The type of system this file is executed on (Mac/Linux/Windows)
UNAME := $(shell uname)
# Under MAC OS X the values have to be changed slightly
ifeq ($(UNAME), Darwin)
#OUTNAME := $(OUTNAME:%.so=%.dylib)
    CXXFLAGS += -stdlib=libc++ -D_DARWIN_C_SOURCE 
    # The c++ compiler that will be used to compile the source files.
    LDFLAGS=$(${CUSTOM_SHARED_LIBS})\
        $(addprefix -L${CUSTOM_SHARED_LIB_DIR}/, ${CUSTOM_SHARED_LIBS})
    CXX=clang++
    # The libs for linking python3 can be found using python3-config
    # Unfortunately, these provide some wrong values when python3.2m is used as version
    # - in that case the lpython3.2 has to be used as lib not 3.2m
    LDLIBS += -stdlib=libc++ -L/Library/Frameworks/Python.framework/Versions/3.6/lib/python3.6/config-3.6m-darwin -lpython3.6 -ldl -framework CoreFoundation
endif


# This is the output name but with the trailing underscore and the file extension removed. 
# "_CommunicationInterface.so" -> "CommunicationInterface"
RAWOUTNAME=$(subst _,,$(subst .so,,$(OUTNAME)))
all: $(CUSTOM_SHARED_LIBS) $(BUILDDIR) $(OUTNAME)

$(BUILDDIR):
	mkdir $(BUILDDIR)

# Link the object files into one shared library
# This target depends on the object files of the individual *.cpp files. Therefore, in the first line (the dependency line), the list of *.cpp files is converted into a list of all *.o files. 
# "$^" is the names of all the dependencies separated by spaces, but with duplicate names removed. 
# "$@" is the file name of the target. 
$(OUTNAME): $(BUILDDIR)/$(RAWOUTNAME)_wrap.o $(SRCCXX:%.cpp=${BUILDDIR}/%.o)
	$(CXX) -o $@ $^  $(LDFLAGS) $(LDLIBS) -shared
ifeq ($(UNAME), Darwin)
#For MAC OS X the dylib relative load path has to be hand tuned
	install_name_tool -change libBfbMessage.dylib "@loader_path/$(HECTORDIR)/SharedLibraries/BfbMessage/libBfbMessage.dylib" _CommunicationInterface.so
	install_name_tool -change libTcpConnection.dylib "@loader_path/$(HECTORDIR)/SharedLibraries/TcpConnection/libTcpConnection.dylib" _CommunicationInterface.so
	install_name_tool -change libPugiXml.dylib "@loader_path/$(HECTORDIR)/SharedLibraries/PugiXml/libPugiXml.dylib" _CommunicationInterface.so
endif

# This is a macro that will build an object file based on the c++ source file. 
# "$<" is the name of the first dependency (the "%.cpp" in the first line). 
# "$@" is the name of the object file. 
$(BUILDDIR)/%.o : $(COMMUNICATIONINTERFACEDIR)/%.cpp
	$(CXX) $(CXXFLAGS) $(CPPFLAGS) -c $< -o $@

# THIERRY: better to use a pkg-config than an explicit path because depending on the system, the minor version of Python3 can differ. To make this command work, the generic package python3-dev must be installed.
$(BUILDDIR)/%_wrap.o: $(BUILDDIR)/%_wrap.cxx
ifeq ($(UNAME), Darwin)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS)  `/usr/local/bin/python3.6-config --cflags` -c $< -o $@ 
else
	$(CXX) $(CPPFLAGS) $(CXXFLAGS)  `pkg-config --cflags python3` -c $< -o $@ 
endif
$(BUILDDIR)/%_wrap.cxx: %.i
	swig -c++ -python $(pkg-config --cflags --libs python3) -I$(COMMUNICATIONINTERFACEDIR) -outdir ./ -o $@ $<

EFFCPP: CXXFLAGS+=-Weffc++
EFFCPP: all

WRAP: $(BUILDDIR)/$(RAWOUTNAME)_wrap.cxx

# All object files are deleted. 
clean:
	@rm -f $(BUILDDIR)/*.* $(RAWOUTNAME).py $(OUTNAME) 

#### SHARED LIBRARIES #########################################

cleanall: clean
	(cd $(CUSTOM_SHARED_LIB_DIR)/BfbMessage; make clean)
	(cd $(CUSTOM_SHARED_LIB_DIR)/TcpConnection; make clean)
	(cd $(CUSTOM_SHARED_LIB_DIR)/PugiXml; make clean)

BfbMessage:
	(cd $(CUSTOM_SHARED_LIB_DIR)/BfbMessage; make)
TcpConnection: 
	(cd $(CUSTOM_SHARED_LIB_DIR)/TcpConnection; make)
PugiXml: 
	(cd $(CUSTOM_SHARED_LIB_DIR)/PugiXml; make)
