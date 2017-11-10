#include <jni.h>
#include <string>

#include<iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/features2d/features2d.hpp>


extern "C"
JNIEXPORT jstring JNICALL
Java_com_example_srinathd_eee598_1poseestimation_1sakalabattula_1konda_1dasari_Camera2BasicFragment_stringFromJNI(
        JNIEnv *env,
        jobject /* this */, jobject jTempImage) {
    std::string hello = "Hello from C++";

    uint8_t *jTempImagePtr = reinterpret_cast<uint8_t *>(env->GetDirectBufferAddress(jTempImage));

    Mat mYuv(srcHeight + srcHeight / 2, srcWidth, CV_8UC1, srcLumaPtr);



    return env->NewStringUTF(hello.c_str());
}
