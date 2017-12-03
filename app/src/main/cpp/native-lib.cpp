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


#include "Mesh.h"
#include "Model.h"
#include "PnPProblem.h"
#include "RobustMatcher.h"
#include "ModelRegistration.h"
#include "Utils.h"




using namespace cv;
extern "C"
using namespace std;
JNIEXPORT jstring JNICALL
Java_com_example_srinathd_eee598_1poseestimation_1sakalabattula_1konda_1dasari_Camera2BasicFragment_nativePoseEstimation(
        JNIEnv *env, jobject instance,
        jint jHeight, jint jWidth, jobject srcBuffer, jobject dstSurface, jstring path, jlong bmp_mat_addr) {

    std::string hello = "Hello from C++";

    int printInt = 10;
//    __android_log_print(ANDROID_LOG_INFO, "Pixels", "pixels[]= %d", printInt);

    uint8_t *srcLumaPtr = reinterpret_cast<uint8_t *>(env->GetDirectBufferAddress(srcBuffer));

    Mat mYuv(jHeight + jHeight / 2, jWidth, CV_8UC1, srcLumaPtr);


    //__android_log_print(ANDROID_LOG_INFO, "YUV Mat", "Mat= %s", mYuv);

    Mat* bitmapImage = (Mat*)bmp_mat_addr;
    Mat img_in = bitmapImage->clone();

    __android_log_print(ANDROID_LOG_INFO, "mYUV", "rows = %d\tcolumns = %d", bitmapImage->rows, bitmapImage->cols);

    bool end_registration = false;

// Intrinsic camera parameters: UVC WEBCAM
    double f = 45; // focal length in mm
    double sx = 22.3, sy = 14.9;
    double width = 2592, height = 1944;
    double params_CANON[] = { width*f/sx,   // fx
                              height*f/sy,  // fy
                              width/2,      // cx
                              height/2};    // cy

// Setup the points to register in the image
// In the order of the *.ply file and starting at 1
    int n = 8;
    int pts[] = {1, 2, 3, 4, 5, 6, 7, 8}; // 3 -> 4

    ModelRegistration registration;
    Model model;
    Mesh mesh;
    PnPProblem pnp_registration(params_CANON);

    int numKeyPoints = 10000;

    //Instantiate robust matcher: detector, extractor, matcher
    RobustMatcher rmatcher;
    Ptr<FeatureDetector> detector = ORB::create(numKeyPoints);
    rmatcher.setFeatureDetector(detector);

    /**  GROUND TRUTH OF THE FIRST IMAGE  **/

    // Create & Open Window
    namedWindow("MODEL REGISTRATION", WINDOW_KEEPRATIO);





    if (!img_in.data) {
        cout << "Could not open or find the image" << endl;
        return NULL;
    }
Mat img_vis;
    // Set the number of points to register
    int num_registrations = n;
    registration.setNumMax(num_registrations);

    cout << "Click the box corners ..." << endl;
    cout << "Waiting ..." << endl;

    // Loop until all the points are registered
    while ( waitKey(30) < 0 )
    {
        // Refresh debug image
        img_vis = img_in.clone();

        // Current registered points
        vector<Point2f> list_points2d = registration.get_points2d();
        vector<Point3f> list_points3d = registration.get_points3d();

        // Draw current registered points
        //drawPoints(img_vis, list_points2d, list_points3d, red);

        // If the registration is not finished, draw which 3D point we have to register.
        // If the registration is finished, breaks the loop.
        if (!end_registration)
        {
            // Draw debug text
            int n_regist = registration.getNumRegist();
            int n_vertex = pts[n_regist];
            Point3f current_poin3d = mesh.getVertex(n_vertex-1);

           // drawQuestion(img_vis, current_poin3d, green);
           // drawCounter(img_vis, registration.getNumRegist(), registration.getNumMax(), red);
        }
        else
        {

            break;
        }

        // Show the image
        imshow("MODEL REGISTRATION", img_vis);
    }

    /** COMPUTE CAMERA POSE **/

    cout << "COMPUTING POSE ..." << endl;

    // The list of registered points
    vector<Point2f> list_points2d = registration.get_points2d();
    vector<Point3f> list_points3d = registration.get_points3d();

    // Estimate pose given the registered points
    bool is_correspondence = pnp_registration.estimatePose(list_points3d, list_points2d, SOLVEPNP_ITERATIVE);
    if ( is_correspondence )
    {
        cout << "Correspondence found" << endl;

        // Compute all the 2D points of the mesh to verify the algorithm and draw it
        vector<Point2f> list_points2d_mesh = pnp_registration.verify_points(&mesh);
       // draw2DPoints(img_vis, list_points2d_mesh, green);

    } else {
        cout << "Correspondence not found" << endl << endl;
    }

    // Show the image
    imshow("MODEL REGISTRATION", img_vis);

    // Show image until ESC pressed
    waitKey(0);


    /** COMPUTE 3D of the image Keypoints **/

    // Containers for keypoints and descriptors of the model
    vector<KeyPoint> keypoints_model;
    Mat descriptors;

    // Compute keypoints and descriptors
    rmatcher.computeKeyPoints(img_in, keypoints_model);
    rmatcher.computeDescriptors(img_in, keypoints_model, descriptors);

    // Check if keypoints are on the surface of the registration image and add to the model
    for (unsigned int i = 0; i < keypoints_model.size(); ++i) {
        Point2f point2d(keypoints_model[i].pt);
        Point3f point3d;
        bool on_surface = pnp_registration.backproject2DPoint(&mesh, point2d, point3d);
        if (on_surface)
        {
            model.add_correspondence(point2d, point3d);
            model.add_descriptor(descriptors.row(i));
            model.add_keypoint(keypoints_model[i]);
        }
        else
        {
            model.add_outlier(point2d);
        }
    }

    // save the model into a *.yaml file
    //model.save(write_path);






    return env->NewStringUTF(hello.c_str());
}
