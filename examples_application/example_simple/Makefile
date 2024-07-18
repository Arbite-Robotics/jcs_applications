##############################################################################################################
# Target
TARGET            = jcs_tool
TARGET_PATH       = ./

BASE_PATH         = ../../

# Path where jcs_host and SOEM libraries are installed
LIB_INSTALL_PATH  = ../../install/

UTILITIES_PATH    = $(BASE_PATH)utilities/
3RD_PARTY_PATH    = $(BASE_PATH)3rd_party/

# General external
INC_EXT           = -I/usr/local/include
COPTS             = -L/usr/local/lib -L/usr/lib

# JCS dev_host lib
# Point to jcs_host install location
JCS_DEV_HOST_PATH = $(LIB_INSTALL_PATH)jcs_host/
LIB_JCS           = -L$(JCS_DEV_HOST_PATH) -ljcs_host
INC_JCS           = -I$(JCS_DEV_HOST_PATH) -I$(JCS_DEV_HOST_PATH)/types

LIB_EXT           = $(LIB_JCS)
INC_EXT           += $(INC_JCS)

# RTAI and pthreads
LIB_EXT           += -lrt -pthread

# SOEM
SOEM_DIR          = $(LIB_INSTALL_PATH)SOEM/install/
SOEM_LIB_DIR      = $(SOEM_DIR)lib/
LIB_SOEM          = -L$(SOEM_LIB_DIR) -lsoem
INC_EXT           += -I$(SOEM_DIR)include/soem/
LIB_EXT           += $(LIB_SOEM)

# YAML
LIB_EXT           += -lyaml-cpp
INC_EXT           += -I/usr/include/yaml-cpp

##############################################################################################################
# COMPILER WARNINGS
CPPWARNINGS  = 
CXXWARNINGS  = 

# COMPILER FLAGS
CPPOPTS     = -std=c++11  $(JCS_CPPOPTS)
# No optimisation
# CPPOPTS       += -g -O0
# With optimisation
CPPOPTS     += -g -O2

CXXOPTS     =

CPPFLAGS    = $(CPPWARNINGS) -fstack-protector-all -Wstack-protector -c -MMD -MP -MF$(@:%.o=%.d) -MT$@ -o $@ $<
CXXFLAGS    = 

# Compiler
CXX     = g++
LD      = g++
MKDIR   = mkdir -p

##############################################################################################################
# Start off objects, includes etc
PROJ_INC = $(INC_EXT)
PROJ_OBS = 

# Function for compiling c++ source
# Arguments: (1)=includes
define CPPFUN
	@echo 'Compiling $<'
	@$(MKDIR) '$(@D)'
	$(CXX) $(CXXOPTS) $(CPPOPTS) $(PROJ_INC) $(CXXFLAGS) $(CPPFLAGS)

endef

##############################################################################################################
# INCLUDES
PROJ_INC += -I./
PROJ_INC += -I$(UTILITIES_PATH)cmd_input_parser/
PROJ_INC += -I$(UTILITIES_PATH)rt/

##############################################################################################################
# Project specific
PROJ_CPPOBJ  = build/example_simple.o
PROJ_CPPOBJ += build/jcs_user_external.o

##############################################################################################################
# External
EXT_CPPOBJ += build/rt/task_rt.o

##############################################################################################################
# Collect all the objects
PROJ_OBJS += $(PROJ_CPPOBJ)
PROJ_OBJS += $(EXT_CPPOBJ)

##############################################################################################################
$(TARGET): $(PROJ_OBJS)
	@echo 'Linking target $@'
	$(LD) $(COPTS) -o build/$(TARGET) $(PROJ_OBJS) $(LIB_EXT)

# Second expansion used in object path substitution
.SECONDEXPANSION:

$(PROJ_CPPOBJ): $$(patsubst build/%.o,%.cpp,$$@)
	$(call CPPFUN)

$(EXT_CPPOBJ): $$(patsubst build/%.o, $(UTILITIES_PATH)%.cpp, $$@)
	$(call CPPFUN)

##############################################################################################################
clean:
	rm -rf build


# Automatically detect .c file dependencies
DEPS := $(PROJ_OBJS)
-include $(DEPS:.o=.d)