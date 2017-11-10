LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

OPENCV_ROOT:=/home/srinathd/Desktop/OpenCV-android-sdk
OPENCV_CAMERA_MODULES:=on
OPENCV_INSTALL_MODULES:=on
OPENCV_LIB_TYPE:=SHARED
include ${OPENCV_ROOT}/sdk/native/jni/OpenCV.mk

NDK_MODULE_PATH=/home/srinathd/Android/Sdk/ndk-bundle
LOCAL_ARM_NEON := true
LOCAL_SRC_FILES := /media/sda5/Courses/F17/MSA/PA2/eee598-poseestimation-sakalabattula-konda-dasari/app/src/main/cpp/native-lib.cpp
LOCAL_CPPFLAGS := -std=gnu++0x
LOCAL_CFLAGS += -O2
LOCAL_LDLIBS += -llog -ldl
LOCAL_MODULE := native


include $(BUILD_SHARED_LIBRARY)