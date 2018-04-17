CXXFLAGS =	-O2 -g -Wall -fmessage-length=0 -Wall -std=c++11


#include paths
INCLUDE_DIRS = -I/home/claudio/Projects/PreCICE/mbdyn/include -I/home/claudio/Projects/PreCICE/precice/src/ -I/home/claudio/Projects/PreCICE/boost/1_66_0_openmpi/include

# libraries
LIBS = -L/home/claudio/Projects/PreCICE/mbdyn/lib -lmbc -L/home/claudio/Projects/PreCICE/precice/build/last -lprecice -L/home/claudio/Projects/PreCICE/boost/1_66_0_openmpi/lib  -lboost_log -lboost_log_setup -lboost_thread -lboost_system -lboost_filesystem -lboost_program_options -lboost_regex

# project directories
SRC_DIR = ./src
OBJ_DIR = ./build
DEP_DIR = ./depend

# project sources
SRC_FILES = $(wildcard $(SRC_DIR)/*.cpp)
OBJ_FILES = $(patsubst $(SRC_DIR)/%.cpp,$(OBJ_DIR)/%.o,$(SRC_FILES))

# project dependencies
DEP_FILES:=$(patsubst $(SRC_DIR)/%.cpp,$(DEP_DIR)/%.d,$(SRC_FILES))



all: mbd_node_ts

# phony targets
.PHONY: clean

NO_DEPS := clean

# the program build just objects to be linked to the examples
mbd_node_ts: $(OBJ_FILES)
	@echo "Program files"
	@echo $(SRC_FILES)
	@echo $(OBJ_FILES)
	g++ $(CXXFLAGS) -o $@ $(OBJ_FILES) $(LIBS)


# build program objects
$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp $(DEP_DIR)/%.d
	g++ $(CXXFLAGS) $(INCLUDE_DIRS) -c -o $@ $<


# -MM dependencies excluding system deps
# -MT ovveride target dependency, otherwise it would be *.o
# -MF file to write the dependency to
$(DEP_DIR)/%.d: $(SRC_DIR)/%.cpp
	$(CXX) $(CXXFLAGS) -MM -MT $(OBJ_DIR)/$(patsubst %.cpp,%.o,$(notdir $<)) $< -MF $@
#	$(CXX) $(CXXFLAGS) -MM -MT $(patsubst $(SRC_DIR)/%,$(OBJ_DIR)/%,$(patsubst %.cpp,%.o,$<)) $< -MF $@
#	$(CXX) $(CXXFLAGS) -MM -MT '$(patsubst $(SRC_DIR)/%.cpp,$(OBJ_DIR)/%.o,$<)' $< -MF $@

# create depenency files
depend:	$(DEP_FILES)
	@echo "make dependencies"

#Don't create dependencies when we're cleaning, for instance
ifeq (0, $(words $(findstring $(MAKECMDGOALS), $(NO_DEPS))))
    #Chances are, these files don't exist.  GMake will create them and
    #clean up automatically afterwards
    -include $(DEP_FILES)
endif

# clean everything
clean:
	@echo "remove example objects"
	rm -f $(OBJS)
	@echo "remove project objects"
	rm -f $(OBJ_FILES)
	@echo "remove dependencies"
	rm -f $(DEP_FILES)
