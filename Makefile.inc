APPL_COBJS +=

APPL_CXXOBJS += cli_main.o cli_menu.o motion.o PID_v1.o token.o gcodeparser.o utils.o selectfile.o directory.o letters.o

SRCLANG := c++

ifdef CONFIG_EV3RT_APPLICATION

# Include libraries
include $(EV3RT_SDK_LIB_DIR)/libcpp-test/Makefile
include $(EV3RT_SDK_LIB_DIR)/lib2/Makefile

endif
