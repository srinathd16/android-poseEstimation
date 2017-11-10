LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

OPENCV_ROOT:=/home/srinathd/Desktop/OpenCV-android-sdk
OPENCV_CAMERA_MODULES:=on
OPENCV_INSTALL_MODULES:=on
OPENCV_LIB_TYPE:=SHARED
include ${OPENCV_ROOT}/sdk/native/jni/OpenCV.mk

NDK_MODULE_PATH=/home/srinathd/Android/Sdk/ndk-bundle
LOCAL_ARM_NEON := true
LOCAL_SRC_FILES := native-lib.cpp

LOCAL_C_INCLUDES += /home/srinathd/Desktop/OpenCV-android-sdk/sdk/native/jni/include
LOCAL_LDLIBS     += -llog -ldl
LOCAL_CFLAGS    += -DOPENCV_OLDER_VISION
LOCAL_CPP_FEATURES += exceptions (Recommended)
LOCAL_CPPFLAGS += -fexceptions

LOCAL_MODULE := native-lib


include $(BUILD_SHARED_LIBRARY)