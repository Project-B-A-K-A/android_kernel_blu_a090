LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_MODULE	:= testmtv
#LOCAL_C_INCLUDES	:= $(LOCAL_PATH)/../src
#LOCAL_C_INCLUDES	+= $(LOCAL_PATH)/..
LOCAL_SRC_FILES	:= test.c test_freq_tbl.c test_isdbt.c
LOCAL_LDLIBS	:= -llog
STLPORT_FORCE_REBUILD := true

#include $(BUILD_SHARED_LIBRARY)
include $(BUILD_EXECUTABLE)
