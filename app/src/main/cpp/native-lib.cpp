
#include <jni.h>
#include <string>

#include<iostream>
#include <android/log.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/features2d/features2d.hpp>

#include <opencv2/calib3d.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/highgui.hpp>

using namespace cv;
using namespace std;

Mat global_descriptors_model;
vector<Point3f> global_list_points3d_model;

extern "C" {

JNIEXPORT jstring JNICALL
/*
Java_com_example_srinathd_eee598_1poseestimation_1sakalabattula_1konda_1dasari_Camera2BasicFragment_nativePoseEstimation(
        JNIEnv *env, jobject instance,
        jint jHeight, jint jWidth, jobject srcBuffer, jobject dstSurface, jstring path, jlong bmp_mat_addr) {
*/
Java_com_example_srinathd_eee598_1poseestimation_1sakalabattula_1konda_1dasari_Camera2BasicFragment_processReferenceImage(
        JNIEnv *env, jobject instance,
        jlong bmp_mat_addr) {

    std::string hello = "Hello from C++";

    int printInt = 10;
//    __android_log_print(ANDROID_LOG_INFO, "Pixels", "pixels[]= %d", printInt);

    //uint8_t *srcLumaPtr = reinterpret_cast<uint8_t *>(env->GetDirectBufferAddress(srcBuffer));

    //Mat mYuv(jHeight + jHeight / 2, jWidth, CV_8UC1, srcLumaPtr);

    //__android_log_print(ANDROID_LOG_INFO, "YUV Mat", "Mat= %s", mYuv);

    Mat *bitmapImage = (Mat *) bmp_mat_addr;
    Mat img_in = bitmapImage->clone();

    bool end_registration = false;



// Setup the points to register in the image
// In the order of the *.ply file and starting at 1
    int n = 8;
    int pts[] = {1, 2, 3, 4, 5, 6, 7, 8}; // 3 -> 4



    int numKeyPoints = 10000;



//computation of key points and descriptors for the ref Image
    vector<KeyPoint> keypoints_model;
    Mat descriptors_model;



    // BruteFroce matcher with Norm Hamming is the default matcher

    cv::Ptr<cv::FeatureDetector> detector_;
    detector_ = cv::ORB::create();
    detector_->detect(img_in, keypoints_model);

    cv::Ptr<cv::DescriptorExtractor> extractor_;
    extractor_ = cv::ORB::create();
    extractor_->compute(img_in, keypoints_model, global_descriptors_model);

    __android_log_print(ANDROID_LOG_INFO, "Key points size", "size= %d", keypoints_model.size());

    vector<Point3f> list_points3d_model;
    for (int i = 0; i < keypoints_model.size(); i++) {
        global_list_points3d_model.push_back(
                Point3d(keypoints_model[i].pt.x, keypoints_model[i].pt.y, 0.0f));
    }

    __android_log_print(ANDROID_LOG_INFO, "mYUV", "list_points = %d", global_list_points3d_model.size());

    return env->NewStringUTF(hello.c_str());
}


JNIEXPORT jstring JNICALL
Java_com_example_srinathd_eee598_1poseestimation_1sakalabattula_1konda_1dasari_Camera2BasicFragment_processCamera2Frames(
        JNIEnv *env, jclass type, jint jWidth, jint jHeight, jobject srcBuffer, jobject dst,
        jstring path_, jlong PMatrixAddr) {
    const char *path = env->GetStringUTFChars(path_, 0);

    std::string returnValue = "hello";
    // TODO

    env->ReleaseStringUTFChars(path_, path);
    __android_log_print(ANDROID_LOG_INFO, "mYUV", "rows");

    //Computation of key points and descriptors for the frame
    vector<KeyPoint> keypoints_frame;
    cv::Ptr<cv::FeatureDetector> detector_;
    detector_ = cv::ORB::create();

    uint8_t *srcLumaPtr = reinterpret_cast<uint8_t *>(env->GetDirectBufferAddress(srcBuffer));

    Mat mYuv(jHeight + jHeight / 2, jWidth, CV_8UC1, srcLumaPtr);

    detector_->detect(mYuv, keypoints_frame);

    Mat descriptors_frame;

    cv::Ptr<cv::DescriptorExtractor> extractor_;
    extractor_ = cv::ORB::create();
    extractor_->compute(mYuv, keypoints_frame, descriptors_frame);

// getting good matches between refImage and Input Frame
    cv::Ptr<cv::DescriptorMatcher> matcher_;
    matcher_ = cv::makePtr<cv::BFMatcher>((int)cv::NORM_HAMMING, false);

    std::vector<std::vector<cv::DMatch> > matches;
    vector<DMatch> good_matches;
    good_matches.clear();
    matcher_->knnMatch(descriptors_frame, global_descriptors_model, matches, 2);


    for ( std::vector<std::vector<cv::DMatch> >::iterator
                  matchIterator= matches.begin(); matchIterator!= matches.end(); ++matchIterator)
    {
        if (!matchIterator->empty()) good_matches.push_back((*matchIterator)[0]);
    }

    std::vector<cv::Point3f> list_points3d_model_match;    // container for the model 3D coordinates found in the scene
    std::vector<cv::Point2f> list_points2d_scene_match;    // container for the model 2D coordinates found in the scene
    for(unsigned int match_index = 0; match_index < good_matches.size(); ++match_index)
    {
        cv::Point3f point3d_model = global_list_points3d_model[ good_matches[match_index].trainIdx ];   // 3D point from model
        cv::Point2f point2d_scene = keypoints_frame[ good_matches[match_index].queryIdx ].pt;    // 2D point from the scene
        list_points3d_model_match.push_back(point3d_model);                                      // add 3D point
        list_points2d_scene_match.push_back(point2d_scene);                                      // add 2D point
    }


//Pose Estimation
    // Intrinsic camera parameters: UVC WEBCAM
    double f = 45; // focal length in mm
    double sx = 22.3, sy = 14.9;
    double width = 2592, height = 1944;
    double params[] = { width*f/sx,   // fx
                        height*f/sy,  // fy
                        width/2,      // cx
                        height/2};    // cy

    Mat _R_matrix;
    Mat _A_matrix;
    Mat _t_matrix;
    Mat _P_matrix;

    _A_matrix = cv::Mat::zeros(3, 3, CV_64FC1);   // intrinsic camera parameters
    _A_matrix.at<double>(0, 0) = params[0];       //      [ fx   0  cx ]
    _A_matrix.at<double>(1, 1) = params[1];       //      [  0  fy  cy ]
    _A_matrix.at<double>(0, 2) = params[2];       //      [  0   0   1 ]
    _A_matrix.at<double>(1, 2) = params[3];
    _A_matrix.at<double>(2, 2) = 1;
    _R_matrix = cv::Mat::zeros(3, 3, CV_64FC1);   // rotation matrix
    _t_matrix = cv::Mat::zeros(3, 1, CV_64FC1);   // translation matrix
    _P_matrix = cv::Mat::zeros(3, 4, CV_64FC1);   // rotation-translation matrix


    cv::Mat distCoeffs = cv::Mat::zeros(4, 1, CV_64FC1);  // vector of distortion coefficients
    cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64FC1);          // output rotation vector
    cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64FC1);    // output translation vector

    bool useExtrinsicGuess = false;   // if true the function uses the provided rvec and tvec values as
    // initial approximations of the rotation and translation vectors


    int pnpMethod = SOLVEPNP_ITERATIVE;
// RANSAC parameters
    int iterationsCount = 500;      // number of Ransac iterations.
    float reprojectionError = 2.0;  // maximum allowed distance to consider it an inlier.
    double confidence = 0.95;        // ransac successful confidence.
    Mat inliers;
    cv::solvePnPRansac( list_points3d_model_match, list_points2d_scene_match, _A_matrix, distCoeffs, rvec, tvec,
                        useExtrinsicGuess, iterationsCount, reprojectionError, confidence,
                        inliers, pnpMethod );

    Rodrigues(rvec,_R_matrix);      // converts Rotation Vector to Matrix
    _t_matrix = tvec;       // set translation matrix


    // Rotation-Translation Matrix Definition
    _P_matrix.at<double>(0,0) = _R_matrix.at<double>(0,0);
    _P_matrix.at<double>(0,1) = _R_matrix.at<double>(0,1);
    _P_matrix.at<double>(0,2) = _R_matrix.at<double>(0,2);
    _P_matrix.at<double>(1,0) = _R_matrix.at<double>(1,0);
    _P_matrix.at<double>(1,1) = _R_matrix.at<double>(1,1);
    _P_matrix.at<double>(1,2) = _R_matrix.at<double>(1,2);
    _P_matrix.at<double>(2,0) = _R_matrix.at<double>(2,0);
    _P_matrix.at<double>(2,1) = _R_matrix.at<double>(2,1);
    _P_matrix.at<double>(2,2) = _R_matrix.at<double>(2,2);
    _P_matrix.at<double>(0,3) = _t_matrix.at<double>(0);
    _P_matrix.at<double>(1,3) = _t_matrix.at<double>(1);
    _P_matrix.at<double>(2,3) = _t_matrix.at<double>(2);

    Mat& pMatrix =  *(Mat*) PMatrixAddr;
    pMatrix = _P_matrix;

    __android_log_print(ANDROID_LOG_INFO, "PFrame", "rows: %f", _P_matrix.at<double>(2,0));


    return env->NewStringUTF(returnValue.c_str());
    //return _P_matrix;
}
}