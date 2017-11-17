#include <jni.h>
#include <string>

#include<iostream>
#include <android/log.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/features2d/features2d.hpp>

using namespace cv;
extern "C"

JNIEXPORT jstring JNICALL
Java_com_example_srinathd_eee598_1poseestimation_1sakalabattula_1konda_1dasari_Camera2BasicFragment_nativePoseEstimation(
        JNIEnv *env, jobject instance,
        jint jHeight, jint jWidth, jobject srcBuffer, jobject dstSurface, jstring path) {

    std::string hello = "Hello from C++";

    int printInt = 10;
//    __android_log_print(ANDROID_LOG_INFO, "Pixels", "pixels[]= %d", printInt);

    uint8_t *srcLumaPtr = reinterpret_cast<uint8_t *>(env->GetDirectBufferAddress(srcBuffer));

    Mat mYuv(jHeight + jHeight / 2, jWidth, CV_8UC1, srcLumaPtr);

    //__android_log_print(ANDROID_LOG_INFO, "YUV Mat", "Mat= %s", mYuv);

//    __android_log_print(ANDROID_LOG_INFO, "Pixels", "pixels[]= %d", printInt++);


    return env->NewStringUTF(hello.c_str());
}

/*
JNIEXPORT jstring JNICALL
Java_com_example_srinathd_eee598_1poseestimation_1sakalabattula_1konda_1dasari_Camera2BasicFragment_stringFromJNI(
        JNIEnv *env, jobject instance, jint yuvHeight, jint yuvWidth, jobject jTempImage) {

    // TODO
    std::string hello = "Hello from C++";

    int printInt = 10;
    __android_log_print(ANDROID_LOG_INFO, "Pixels", "pixels[]= %d", printInt);

    uint8_t *jTempImagePtr = reinterpret_cast<uint8_t *>(env->GetDirectBufferAddress(jTempImage));

    Mat mYuv(yuvHeight + yuvHeight / 2, yuvWidth, CV_8UC1, jTempImagePtr);

    //__android_log_print(ANDROID_LOG_INFO, "YUV Mat", "Mat= %s", mYuv);

    __android_log_print(ANDROID_LOG_INFO, "Pixels", "pixels[]= %d", printInt++);

    return env->NewStringUTF(hello.c_str());

}
*/
/*
JNIEXPORT jstring JNICALL
Java_com_example_srinathd_eee598_poseestimation_sakalabattula_konda_dasari_Camera2BasicFragment_stringFromJNI(
        JNIEnv *env,
        jobject /* this *///, jint yuvHeight, jint yuvWidth, jobject jTempImage) {
/*    std::string hello = "Hello from C++";

    int printInt = 10;
    __android_log_print(ANDROID_LOG_INFO, "Pixels", "pixels[]= %d", printInt);

    uint8_t *jTempImagePtr = reinterpret_cast<uint8_t *>(env->GetDirectBufferAddress(jTempImage));

    Mat mYuv(yuvHeight + yuvHeight / 2, yuvWidth, CV_8UC1, jTempImagePtr);

    //__android_log_print(ANDROID_LOG_INFO, "YUV Mat", "Mat= %s", mYuv);

    __android_log_print(ANDROID_LOG_INFO, "Pixels", "pixels[]= %d", printInt++);

    return env->NewStringUTF(hello.c_str());
}
*/