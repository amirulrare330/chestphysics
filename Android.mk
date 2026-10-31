LOCAL_PATH := $(call my-dir)
include $(CLEAR_VARS)
LOCAL_MODULE    := ChestPhysics
LOCAL_SRC_FILES := ChestPhysics.cpp
LOCAL_C_INCLUDES := $(LOCAL_PATH)/aml_sdk
LOCAL_CPPFLAGS  := -O2 -std=c++17 -fno-exceptions -fno-rtti
LOCAL_LDLIBS    := -llog
include $(BUILD_SHARED_LIBRARY)
