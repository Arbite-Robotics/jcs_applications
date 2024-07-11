##############################################################################################################
# Target
TARGET            = jcs_tool
APPLICATION       = ../../
TARGET_PATH       = $(APPLICATION)jcs_applications/jcs_tool/
UTILITIES_PATH    = $(APPLICATION)jcs_applications/utilities/
3RD_PARTY_PATH    = $(APPLICATION)jcs_applications/3rd_party/
JCS_DEV_HOST_PATH = $(APPLICATION)jcs_host/

# General external
INC_EXT         = -I/usr/local/include
COPTS           = -L/usr/local/lib -L/usr/lib

# JCS dev_host lib
LIB_JCS         = -L$(JCS_DEV_HOST_PATH) -ljcs_host
INC_JCS         = -I$(JCS_DEV_HOST_PATH) -I$(JCS_DEV_HOST_PATH)/types

LIB_EXT         = $(LIB_JCS)
INC_EXT         += $(INC_JCS)

# RTAI and pthreads
LIB_EXT         += -lrt -pthread

# SOEM
SOEM_DIR        = ../../../3rd_party/SOEM/install/
SOEM_LIB_DIR    = $(SOEM_DIR)lib/
LIB_SOEM        = -L$(SOEM_LIB_DIR) -lsoem
INC_EXT         += -I$(SOEM_DIR)include/soem/
LIB_EXT         += $(LIB_SOEM)

# YAML
# LIB_EXT         += -lyaml-cpp
# LIB_EXT         += -l:libyaml-cpp.so.0.7
LIB_EXT         += `pkg-config --static --libs yaml-cpp`
INC_EXT         += -I/usr/include/yaml-cpp

##############################################################################################################
# COMPILER WARNINGS
CPPWARNINGS  = 
CXXWARNINGS  = 

# COMPILER FLAGS
# No optimisation
# CPPOPTS   = -g -O0
# With optimisation
CPPOPTS     = -g -O2

CXXOPTS     = -std=c++11

CPPFLAGS    = $(CPPWARNINGS) -fstack-protector-all -Wstack-protector -c -MMD -MP -MF$(@:%.o=%.d) -MT$@ -o $@ $<
CXXFLAGS    = 

# C compiler
COPTS 		= 
CFLAGS 		= -Wmissing-prototypes

# Compiler
CXX     = g++
CC      = gcc
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

define CFUN
	@echo 'Compiling $<'
	@$(MKDIR) '$(@D)'
	$(CC) $(COPTS) $(CPPOPTS) $(1) $(CFLAGS) $(CPPFLAGS)
endef

##############################################################################################################
# INCLUDES
PROJ_INC += -I./
PROJ_INC += -I$(UTILITIES_PATH)cmd_input_parser/
PROJ_INC += -I$(UTILITIES_PATH)rt/
PROJ_INC += -I$(UTILITIES_PATH)rotate/
PROJ_INC += -I$(UTILITIES_PATH)ramp/
PROJ_INC += -I$(UTILITIES_PATH)recorder/
PROJ_INC += -I$(UTILITIES_PATH)config/

##############################################################################################################
# Project specific
PROJ_CPPOBJ  = build/jcs_tool.o
PROJ_CPPOBJ += build/tool_manager.o
PROJ_CPPOBJ += build/jcs_tool_if.o
PROJ_CPPOBJ += build/jcs_user_external.o

##############################################################################################################
# Tools 
# Tool ID
include $(TARGET_PATH)tools/tool_id/tool_id.mk
PROJ_INC    += $(JCS_TOOL_ID_INC)
PROJ_CPPOBJ += $(JCS_TOOL_ID_SRC)
# Tool: Motor controller current test 
include $(TARGET_PATH)tools/tool_mc_current_test/tool_mc_current_test.mk
PROJ_INC    += $(JCS_TOOL_MC_I_TEST_INC)
PROJ_CPPOBJ += $(JCS_TOOL_MC_I_TEST_SRC)
# Tool: gui 
include $(TARGET_PATH)tools/tool_gui/tool_gui.mk
PROJ_INC    += $(JCS_TOOL_GUI_INC)
PROJ_CPPOBJ += $(JCS_TOOL_GUI_SRC)
DEV_HOST_CPPOBJ += $(DEV_HOST_SRC)
3RD_PARTY_CPPOBJ += $(3RD_PARTY_SRC)
LIB_EXT     += $(JCS_TOOL_GUI_LIBEXT)
CXXFLAGS    += $(JCS_TOOL_GUI_CXXFLAGS)

3RD_PARTY_COBJ += $(3RD_PARTY_CSRC)
CPPFLAGS       += $(3RD_PARTY_COPTS)
##############################################################################################################
# External
EXT_CPPOBJ += build/rt/task_rt.o
EXT_CPPOBJ += build/rotate/rotate.o
EXT_CPPOBJ += build/ramp/ramp.o
EXT_CPPOBJ += build/recorder/recorder.o
EXT_CPPOBJ += build/config/config.o

##############################################################################################################
# Collect all the objects
PROJ_OBJS += $(PROJ_CPPOBJ)
PROJ_OBJS += $(DEV_HOST_CPPOBJ)
PROJ_OBJS += $(EXT_CPPOBJ)
PROJ_OBJS += $(3RD_PARTY_CPPOBJ)
PROJ_OBJS += $(3RD_PARTY_COBJ)

##############################################################################################################
$(TARGET): $(PROJ_OBJS)
	@echo 'Linking target $@'
	$(LD) $(COPTS) -o build/$(TARGET) $(PROJ_OBJS) $(LIB_EXT)

# Second expansion used in object path substitution
.SECONDEXPANSION:

$(PROJ_CPPOBJ): $$(patsubst build/%.o,%.cpp,$$@)
	$(call CPPFUN)

$(DEV_HOST_CPPOBJ): $$(patsubst build/%.o, $(JCS_DEV_HOST_PATH)%.cpp, $$@)
	$(call CPPFUN)

$(EXT_CPPOBJ): $$(patsubst build/%.o, $(UTILITIES_PATH)%.cpp, $$@)
	$(call CPPFUN)

$(3RD_PARTY_CPPOBJ): $$(patsubst build/%.o, $(3RD_PARTY_PATH)%.cpp, $$@)
	$(call CPPFUN)

$(3RD_PARTY_COBJ): $$(patsubst build/%.o, $(3RD_PARTY_PATH)%.c, $$@)
	$(call CFUN)

##############################################################################################################
clean:
	rm -rf build


# Automatically detect .c file dependencies
DEPS := $(PROJ_OBJS)
-include $(DEPS:.o=.d)