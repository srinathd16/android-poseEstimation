/*
 * Copyright 2017 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *       http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package com.example.srinathd.eee598_poseestimation_sakalabattula_konda_dasari;

import android.Manifest;
import android.app.Activity;
import android.app.AlertDialog;
import android.app.Dialog;
import android.content.Context;
import android.content.DialogInterface;
import android.content.pm.PackageManager;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.ImageFormat;
import android.graphics.Matrix;
import android.graphics.Paint;
import android.graphics.PixelFormat;
import android.graphics.Point;
import android.graphics.PorterDuff;
import android.graphics.RectF;
import android.graphics.SurfaceTexture;
import android.graphics.YuvImage;
import android.hardware.camera2.CameraAccessException;
import android.hardware.camera2.CameraCaptureSession;
import android.hardware.camera2.CameraCharacteristics;
import android.hardware.camera2.CameraDevice;
import android.hardware.camera2.CameraManager;
import android.hardware.camera2.CaptureRequest;
import android.hardware.camera2.CaptureResult;
import android.hardware.camera2.TotalCaptureResult;
import android.hardware.camera2.params.StreamConfigurationMap;
import android.media.Image;
import android.media.ImageReader;
import android.os.Build;
import android.os.Bundle;
import android.os.Environment;
import android.os.Handler;
import android.os.HandlerThread;
import android.os.SystemClock;
import android.support.annotation.NonNull;
import android.support.annotation.RequiresApi;
import android.support.v4.app.ActivityCompat;
import android.support.v4.app.DialogFragment;
import android.support.v4.app.Fragment;
import android.support.v4.content.ContextCompat;
import android.util.Log;
import android.util.Size;
import android.view.LayoutInflater;
import android.view.MotionEvent;
import android.view.Surface;
import android.view.SurfaceHolder;
import android.view.SurfaceView;
import android.view.TextureView;
import android.view.View;
import android.view.ViewGroup;
import android.widget.ImageView;
import android.widget.LinearLayout;
import android.widget.TextView;
import android.widget.Toast;

import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.text.SimpleDateFormat;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.Date;
import java.util.Objects;
import java.util.concurrent.Semaphore;
import java.util.concurrent.TimeUnit;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point3;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import static org.opencv.core.CvType.CV_64FC1;

/**This file/class is called by Camera2BasicFragment.java
 * Includes the class Camera2BasicFragment, containing calls to functions that implement various operations on Camera to be used for the achieving Augmented Reality.
 */

@RequiresApi(api = Build.VERSION_CODES.LOLLIPOP)
public class Camera2BasicFragment extends Fragment
        implements ActivityCompat.OnRequestPermissionsResultCallback {

    Camera2BasicFragment globalContext;
	
    private static final int REQUEST_CAMERA_PERMISSION = 1;
    private static final String FRAGMENT_DIALOG = "dialog";
    /**
     * Tag for the {@link Log}.
     */
    private static final String TAG = "Camera2BasicFragment";

    /**
     * Camera state: Showing camera preview.
     */
    private static final int STATE_PREVIEW = 0;

    /**
     * Max preview width that is guaranteed by Camera2 API
     */
    private static final int MAX_PREVIEW_WIDTH = 1920;

    /**
     * Max preview height that is guaranteed by Camera2 API
     */
    private static final int MAX_PREVIEW_HEIGHT = 1080;

    /**
     * The listener can be used to notify when the surface texure associated with the texture view is available.
	 * onSurfaceTextureAvailable - Invoked when the surface texture is ready for use.
	 * onSurfaceTextureSizeChanged - Invoked when the size of the buffer is changed.
	 * onSurfaceTextureDestroyed - Called when the surface texture is about to be destroyed. If it is true there should be no further rendering, if false it should call release function.
	 * onSurfaceTextureUpdated - Invoked when the surface texture is updated.
     */
    private final TextureView.SurfaceTextureListener mSurfaceTextureListener
            = new TextureView.SurfaceTextureListener() {

        @Override
        public void onSurfaceTextureAvailable(SurfaceTexture texture, int width, int height) {
            openCamera(width, height);
        }

        @Override
        public void onSurfaceTextureSizeChanged(SurfaceTexture texture, int width, int height) {
            configureTransform(width, height);
        }

        @Override
        public boolean onSurfaceTextureDestroyed(SurfaceTexture texture) {
            return true;
        }

        @Override
        public void onSurfaceTextureUpdated(SurfaceTexture texture) {
        }

    };

    /**
     * ID of the current cameradevice.
     */
    private String mCameraId;

    /**
     * AutoFitTextureView for camera preview.
     */
    private AutoFitTextureView mTextureView;

    /**
     * CameraCaptureSession for camera preview.
     */
    private CameraCaptureSession mCaptureSession;

    /**
     * A reference to the opened CameraDevice.
     */
    private CameraDevice mCameraDevice;

    /**
     * The Size of camera preview.
     */
    private Size mPreviewSize;

    /**
     * CameraDevice.StateCallback is called when CameraDevice changes its state.
	 * Object for receiving update about the state of a camera device.
	 * onOpened - This method is called when the camera is opened.  We start camera preview here.
	 * onDisconnected - This method is called when the camera device is no longer available for use.
	 * onError - Called when camera device has an error.
     */

    private final CameraDevice.StateCallback mStateCallback = new CameraDevice.StateCallback() {

        @Override
        public void onOpened(@NonNull CameraDevice cameraDevice) {
            mCameraOpenCloseLock.release();
            mCameraDevice = cameraDevice;
            createCameraPreviewSession();
        }

        @Override
        public void onDisconnected(@NonNull CameraDevice cameraDevice) {
            mCameraOpenCloseLock.release();
            cameraDevice.close();
            mCameraDevice = null;
        }

        @Override
        public void onError(@NonNull CameraDevice cameraDevice, int error) {
            mCameraOpenCloseLock.release();
            cameraDevice.close();
            mCameraDevice = null;
            Activity activity = getActivity();
            if (null != activity) {
                activity.finish();
            }
        }

    };

    /**
     * An additional thread for running tasks that shouldn't block the UI.
     */
    private HandlerThread mBackgroundThread;

    /**
     * A Handler for running tasks in the background.
     */
    private Handler mBackgroundHandler;

    /**
     * An ImageReader that handles still image capture.
     */
    private ImageReader mImageReader;

    /**
     * This is the output file for our picture.
     */

    /**
     * This a callback object for the ImageReader. "onImageAvailable" will be called when a
     * still image is ready to be saved.
	 * onImageAvailable - Callback that is called when a new image is available from ImageReader.
     */
    private final ImageReader.OnImageAvailableListener mOnImageAvailableListener
            = new ImageReader.OnImageAvailableListener() {

        @Override
        public void onImageAvailable(ImageReader reader) {

            Image tempImage = reader.acquireNextImage();

            Mat mLines;
            if (tempImage.getFormat() != ImageFormat.YUV_420_888) {
                throw new IllegalArgumentException("src must have format YUV_420_888.");
            }
            Image.Plane[] planes = tempImage.getPlanes();
            // Spec guarantees that planes[0] is luma and has pixel stride of 1.
            // It also guarantees that planes[1] and planes[2] have the same row and
            // pixel stride.
            mLines = new Mat();

            String path = "/storage/emulated/0/Android/data/com.example.asuforia_augmented_reality/files/";

            Log.d("DIR", "external: " + Environment.getExternalStorageDirectory());

            long current_time = System.currentTimeMillis();

            Mat posePoints2D = new Mat();

            nativePoseEstimation(tempImage.getHeight(), tempImage.getWidth(), planes[0].getBuffer(), msurface, path, posePoints2D.getNativeObjAddr());

            long elapsed_time = (System.currentTimeMillis() - current_time);
            String elapsed_time_string = Objects.toString(elapsed_time);
            Log.d("processframes", "Time: "+elapsed_time_string);

            Log.d("posePoints2DMat", "rows: "+posePoints2D.rows()+" cols: "+posePoints2D.cols());

            double[] origin = posePoints2D.get(0,0);
            double[] x = posePoints2D.get(1,0);
            double[] y = posePoints2D.get(2,0);
            double[] z = posePoints2D.get(3,0);
            double[] diagonal_xy = posePoints2D.get(4,0);
            double[] x_3d = posePoints2D.get(5,0);
            double[] y_3d = posePoints2D.get(6,0);
            double[] diagonal_3d = posePoints2D.get(7,0);
            Log.d("Origin", "origin: "+origin[0]+ " " + origin[1]);
            Log.d("DoubleX", "x: "+x[0]+ " " + x[1]);

            //onPose onPose = new onPose(globalContext, origin, x, y, z, diagonal_xy, x_3d, y_3d, diagonal_3d);
			poseListener(globalContext, origin, x, y, z, diagonal_xy, x_3d, y_3d, diagonal_3d);
            tempImage.close();
        }

    };
	
	public void poseListener(Camera2BasicFragment context, double[] origin, double[] x, double[] y, double[] z, double[] diagonal_xy,
                             double[] x_3d, double[] y_3d, double[] diagonal_3d){
        onPose onpose = new onPose(context, origin, x, y, z, diagonal_xy, x_3d, y_3d, diagonal_3d);
    }


    /**
     * CaptureRequest.Builder for the camera preview
     */
    private CaptureRequest.Builder mPreviewRequestBuilder;

    /**
     * CaptureRequest generated by #mPreviewRequestBuilder
     */
    private CaptureRequest mPreviewRequest;

    private Surface msurface;
    /**
     * The current state of camera state for taking pictures.
     */
    private int mState = STATE_PREVIEW;

    /**
     * A Semaphore to prevent the app from exiting before closing the camera.
     */
    private Semaphore mCameraOpenCloseLock = new Semaphore(1);

    /**
     * Whether the current camera device supports Flash or not.
     */

    /**
     * Orientation of the camera sensor
     */
    private int mSensorOrientation;
	
	/**
	 * A CameraCaptureSession.CaptureCallback that handles events related to JPEG capture.
	 * onCaptureProgressed - Called when an image capture makes partial progress i.e. some results from the image capture are available.
	 * onCaptureCompleted - Called when the image capture is completed fully 
     */
	
    private CameraCaptureSession.CaptureCallback mCaptureCallback
            = new CameraCaptureSession.CaptureCallback() {

        private void process(CaptureResult result) {
            switch (mState) {
                case STATE_PREVIEW: {
                    break;
                }
            }
        }

        @Override
        public void onCaptureProgressed(@NonNull CameraCaptureSession session,
                                        @NonNull CaptureRequest request,
                                        @NonNull CaptureResult partialResult) {
            process(partialResult);
        }

        @Override
        public void onCaptureCompleted(@NonNull CameraCaptureSession session,
                                       @NonNull CaptureRequest request,
                                       @NonNull TotalCaptureResult result) {
            process(result);
        }

    };


    public static Camera2BasicFragment newInstance() {
        return new Camera2BasicFragment();
    }

    /**
	 * onCreateView - Creates and returns the view hierarchy of the image frame.
	 * Also converting the bitmap to Mat type for further processing
     */
    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container,
                             Bundle savedInstanceState) {

        BitmapFactory.Options opts = new BitmapFactory.Options();
        opts.inPreferredConfig = Bitmap.Config.ARGB_8888; // Each pixel is 4 bytes: Alpha, Red, Green, Blue
        Bitmap bmpIn = BitmapFactory.decodeResource(getResources(), R.drawable.sample, opts);
        Mat bmp_mat = new Mat();
        Bitmap bmp32 = bmpIn.copy(Bitmap.Config.ARGB_8888, true);
        Utils.bitmapToMat(bmp32, bmp_mat);
        long bmp_mat_addr = bmp_mat.getNativeObjAddr();

        Asuforia(bmp_mat_addr);

        globalContext = this;

        View view = inflater.inflate(R.layout.fragment_camera2_basic, container, false);

        LinearLayout surface = view.findViewById(R.id.surface);

        surface.addView(new onPose(this, view));

        return view;
    }
	
	/**
	 * onViewCreated - called after onCreateView has returned and also the saved state is restored in the view.
     */

    @Override
    public void onViewCreated(final View view, Bundle savedInstanceState) {
        mTextureView = (AutoFitTextureView) view.findViewById(R.id.texture);
    }

	/**
	 * onActivityCreated - called after onCreateView has returned and also the saved state is restored in the view.
     */
	
    @Override
    public void onActivityCreated(Bundle savedInstanceState) {
        super.onActivityCreated(savedInstanceState);
    }

	
	/**
	 * onResume - Called when the activity will start interacting with the user. 
	 */		
    @Override
    public void onResume() {
        super.onResume();
        startBackgroundThread();

        // When the screen is turned off and turned back on, the SurfaceTexture is already
        // available, and "onSurfaceTextureAvailable" will not be called. In that case, we can open
        // a camera and start preview from here (otherwise, we wait until the surface is ready in
        // the SurfaceTextureListener).
        if (mTextureView.isAvailable()) {
            openCamera(mTextureView.getWidth(), mTextureView.getHeight());
        } else {
            mTextureView.setSurfaceTextureListener(mSurfaceTextureListener);
        }
    }

	/**
	 * onPause - Called when the activity will start interacting with the user. 
	 */		
    @Override
    public void onPause() {
        closeCamera();
        stopBackgroundThread();
        super.onPause();
    }

	/**
	 * requestCameraPermission - whether we should show UI with rationale for requesting a permission
	 */	
    private void requestCameraPermission() {
        if (shouldShowRequestPermissionRationale(Manifest.permission.CAMERA)) {
            new ConfirmationDialog().show(getChildFragmentManager(), FRAGMENT_DIALOG);
        } else {
            requestPermissions(new String[]{Manifest.permission.CAMERA}, REQUEST_CAMERA_PERMISSION);
        }
    }

	/**
	 * onRequestPermissionsResult - Callback for the result from requesting permissions
	 */	
    @Override
    public void onRequestPermissionsResult(int requestCode, @NonNull String[] permissions,
                                           @NonNull int[] grantResults) {
        if (requestCode == REQUEST_CAMERA_PERMISSION) {
            if (grantResults.length != 1 || grantResults[0] != PackageManager.PERMISSION_GRANTED) {
                ErrorDialog.newInstance(getString(R.string.request_permission))
                        .show(getChildFragmentManager(), FRAGMENT_DIALOG);
            }
        } else {
            super.onRequestPermissionsResult(requestCode, permissions, grantResults);
        }
    }

    /**
     * Sets up member variables related to camera like height, width, rotation of surface etc.
     */
    @SuppressWarnings("SuspiciousNameCombination")
    private void setUpCameraOutputs(int width, int height) {
        Activity activity = getActivity();
        CameraManager manager = (CameraManager) activity.getSystemService(Context.CAMERA_SERVICE);
        try {
            for (String cameraId : manager.getCameraIdList()) {
                CameraCharacteristics characteristics
                        = manager.getCameraCharacteristics(cameraId);

                // We don't use a front facing camera in this sample.
                Integer facing = characteristics.get(CameraCharacteristics.LENS_FACING);
                if (facing != null && facing == CameraCharacteristics.LENS_FACING_FRONT) {
                    continue;
                }

                StreamConfigurationMap map = characteristics.get(
                        CameraCharacteristics.SCALER_STREAM_CONFIGURATION_MAP);
                if (map == null) {
                    continue;
                }

                // For still image captures, we use the largest available size.
                Size largest = Collections.max(
                        Arrays.asList(map.getOutputSizes(ImageFormat.YUV_420_888)),
                        new CompareSizesByArea());
                mImageReader = ImageReader.newInstance(largest.getWidth(), largest.getHeight(),
                        ImageFormat.YUV_420_888, /*maxImages*/5);
                mImageReader.setOnImageAvailableListener(
                        mOnImageAvailableListener, mBackgroundHandler);

                // Find out if we need to swap dimension to get the preview size relative to sensor
                // coordinate.
                int displayRotation = activity.getWindowManager().getDefaultDisplay().getRotation();
                //noinspection ConstantConditions
                mSensorOrientation = characteristics.get(CameraCharacteristics.SENSOR_ORIENTATION);
                boolean swappedDimensions = false;
                switch (displayRotation) {
                    case Surface.ROTATION_0:
                    case Surface.ROTATION_180:
                        if (mSensorOrientation == 90 || mSensorOrientation == 270) {
                            swappedDimensions = true;
                        }
                        break;
                    case Surface.ROTATION_90:
                    case Surface.ROTATION_270:
                        if (mSensorOrientation == 0 || mSensorOrientation == 180) {
                            swappedDimensions = true;
                        }
                        break;
                    default:
                        Log.e(TAG, "Display rotation is invalid: " + displayRotation);
                }

                Point displaySize = new Point();
                activity.getWindowManager().getDefaultDisplay().getSize(displaySize);
                int rotatedPreviewWidth = width;
                int rotatedPreviewHeight = height;
                int maxPreviewWidth = displaySize.x;
                int maxPreviewHeight = displaySize.y;

                if (swappedDimensions) {
                    rotatedPreviewWidth = height;
                    rotatedPreviewHeight = width;
                    maxPreviewWidth = displaySize.y;
                    maxPreviewHeight = displaySize.x;
                }

                if (maxPreviewWidth > MAX_PREVIEW_WIDTH) {
                    maxPreviewWidth = MAX_PREVIEW_WIDTH;
                }

                if (maxPreviewHeight > MAX_PREVIEW_HEIGHT) {
                    maxPreviewHeight = MAX_PREVIEW_HEIGHT;
                }

                mCameraId = cameraId;
                return;
            }
        } catch (CameraAccessException e) {
            e.printStackTrace();
        } catch (NullPointerException e) {
            // Currently an NPE is thrown when the Camera2API is used but not supported on the
            // device this code runs.
            ErrorDialog.newInstance(getString(R.string.camera_error))
                    .show(getChildFragmentManager(), FRAGMENT_DIALOG);
        }
    }

    /**
     * Opens the camera specified by Camera2BasicFragment by checking the permissions, setting up the camera height and width 
     */
    private void openCamera(int width, int height) {
        if (ContextCompat.checkSelfPermission(getActivity(), Manifest.permission.CAMERA)
                != PackageManager.PERMISSION_GRANTED) {
            requestCameraPermission();
            return;
        }
        setUpCameraOutputs(width, height);
        configureTransform(width, height);
        Activity activity = getActivity();
        CameraManager manager = (CameraManager) activity.getSystemService(Context.CAMERA_SERVICE);
        try {
            if (!mCameraOpenCloseLock.tryAcquire(2500, TimeUnit.MILLISECONDS)) {
                throw new RuntimeException("Time out waiting to lock camera opening.");
            }
            manager.openCamera(mCameraId, mStateCallback, mBackgroundHandler);
        } catch (CameraAccessException e) {
            e.printStackTrace();
        } catch (InterruptedException e) {
            throw new RuntimeException("Interrupted while trying to lock camera opening.", e);
        }
    }

    /**
     * Closes the current CameraDevice by locking the camera 
     */
    private void closeCamera() {
        try {
            mCameraOpenCloseLock.acquire();
            if (null != mCaptureSession) {
                mCaptureSession.close();
                mCaptureSession = null;
            }
            if (null != mCameraDevice) {
                mCameraDevice.close();
                mCameraDevice = null;
            }
            if (null != mImageReader) {
                mImageReader.close();
                mImageReader = null;
            }
        } catch (InterruptedException e) {
            throw new RuntimeException("Interrupted while trying to lock camera closing.", e);
        } finally {
            mCameraOpenCloseLock.release();
        }
    }

    /**
     * Starts a background thread and attaches its handler 
     */
    private void startBackgroundThread() {
        mBackgroundThread = new HandlerThread("CameraBackground");
        mBackgroundThread.start();
        mBackgroundHandler = new Handler(mBackgroundThread.getLooper());
    }

    /**
     * Stops the background thread and assigns its respective handler to null.
     */
    private void stopBackgroundThread() {
        mBackgroundThread.quitSafely();
        try {
            mBackgroundThread.join();
            mBackgroundThread = null;
            mBackgroundHandler = null;
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    /**
     * Creates a new CameraCaptureSession for camera preview.
     */
    private void createCameraPreviewSession() {
        try {
            SurfaceTexture texture = mTextureView.getSurfaceTexture();

            assert texture != null;

            Log.d("PoseEstimate", "createCameraPreviewSession");

            // We configure the size of default buffer to be the size of camera preview we want.
			// texture.setDefaultBufferSize(mPreviewSize.getWidth(), mPreviewSize.getHeight());
            texture.setDefaultBufferSize(MAX_PREVIEW_WIDTH, MAX_PREVIEW_HEIGHT);

            // This is the output Surface we need to start preview.
            msurface = new Surface(texture);

            // We set up a CaptureRequest.Builder with the output Surface.
            mPreviewRequestBuilder
                    = mCameraDevice.createCaptureRequest(CameraDevice.TEMPLATE_PREVIEW);
            mPreviewRequestBuilder.addTarget(msurface);

            mPreviewRequestBuilder.addTarget(mImageReader.getSurface());

            // Here, we create a CameraCaptureSession for camera preview.
            mCameraDevice.createCaptureSession(Arrays.asList(msurface, mImageReader.getSurface()),
                    new CameraCaptureSession.StateCallback() {

                        @Override
                        public void onConfigured(@NonNull CameraCaptureSession cameraCaptureSession) {
                            // The camera is already closed
                            if (null == mCameraDevice) {
                                return;
                            }

                            // When the session is ready, we start displaying the preview.
                            mCaptureSession = cameraCaptureSession;
                            try {
                                // Auto focus should be continuous for camera preview.
                                mPreviewRequestBuilder.set(CaptureRequest.CONTROL_AF_MODE,
                                        CaptureRequest.CONTROL_AF_MODE_CONTINUOUS_PICTURE);
                                // Flash is automatically enabled when necessary.
								// setAutoFlash(mPreviewRequestBuilder);

                                // Finally, we start displaying the camera preview.
                                mPreviewRequest = mPreviewRequestBuilder.build();
                                mCaptureSession.setRepeatingRequest(mPreviewRequest,
                                        mCaptureCallback, mBackgroundHandler);
                                Log.d("Preview", "setrepeating");
                            } catch (CameraAccessException e) {
                                e.printStackTrace();
                            }
                        }

                        @Override
                        public void onConfigureFailed(
                                @NonNull CameraCaptureSession cameraCaptureSession) {
                        }
                    }, null
            );
        } catch (CameraAccessException e) {
            e.printStackTrace();
        }
    }

    /**
     * Configures the necessary Matrix transformation to `mTextureView`.
     * This method should be called after the camera preview size is determined in
     * setUpCameraOutputs and also the size of `mTextureView` is fixed.
     */
    private void configureTransform(int viewWidth, int viewHeight) {
        Activity activity = getActivity();
        if (null == mTextureView || null == mPreviewSize || null == activity) {
            return;
        }
        int rotation = activity.getWindowManager().getDefaultDisplay().getRotation();
        Matrix matrix = new Matrix();
        RectF viewRect = new RectF(0, 0, viewWidth, viewHeight);
        RectF bufferRect = new RectF(0, 0, mPreviewSize.getHeight(), mPreviewSize.getWidth());
        float centerX = viewRect.centerX();
        float centerY = viewRect.centerY();
        if (Surface.ROTATION_90 == rotation || Surface.ROTATION_270 == rotation) {
            bufferRect.offset(centerX - bufferRect.centerX(), centerY - bufferRect.centerY());
            matrix.setRectToRect(viewRect, bufferRect, Matrix.ScaleToFit.FILL);
            float scale = Math.max(
                    (float) viewHeight / mPreviewSize.getHeight(),
                    (float) viewWidth / mPreviewSize.getWidth());
            matrix.postScale(scale, scale, centerX, centerY);
            matrix.postRotate(90 * (rotation - 2), centerX, centerY);
        } else if (Surface.ROTATION_180 == rotation) {
            matrix.postRotate(180, centerX, centerY);
        }
        mTextureView.setTransform(matrix);
    }

    /**
     * Compares two size's based on their areas.
     */
    static class CompareSizesByArea implements Comparator<Size> {

        @Override
        public int compare(Size lhs, Size rhs) {
            // We cast here to ensure the multiplications won't overflow
            return Long.signum((long) lhs.getWidth() * lhs.getHeight() -
                    (long) rhs.getWidth() * rhs.getHeight());
        }

    }

    /**
     * Shows an error message dialog by using ErrorDialog and onCreateDialog
     */
    public static class ErrorDialog extends DialogFragment {

        private static final String ARG_MESSAGE = "message";

        public static ErrorDialog newInstance(String message) {
            ErrorDialog dialog = new ErrorDialog();
            Bundle args = new Bundle();
            args.putString(ARG_MESSAGE, message);
            dialog.setArguments(args);
            return dialog;
        }

        @NonNull
        @Override
        public Dialog onCreateDialog(Bundle savedInstanceState) {
            final Activity activity = getActivity();
            return new AlertDialog.Builder(activity)
                    .setMessage(getArguments().getString(ARG_MESSAGE))
                    .setPositiveButton(android.R.string.ok, new DialogInterface.OnClickListener() {
                        @Override
                        public void onClick(DialogInterface dialogInterface, int i) {
                            activity.finish();
                        }
                    })
                    .create();
        }

    }

    /**
     * Shows OK/Cancel confirmation dialog about camera permission.
     */
    public static class ConfirmationDialog extends DialogFragment {

        @NonNull
        @Override
        public Dialog onCreateDialog(Bundle savedInstanceState) {
            final Fragment parent = getParentFragment();
            return new AlertDialog.Builder(getActivity())
                    .setMessage(R.string.request_permission)
                    .setPositiveButton(android.R.string.ok, new DialogInterface.OnClickListener() {
                        @Override
                        public void onClick(DialogInterface dialog, int which) {
                            parent.requestPermissions(new String[]{Manifest.permission.CAMERA},
                                    REQUEST_CAMERA_PERMISSION);
                        }
                    })
                    .setNegativeButton(android.R.string.cancel,
                            new DialogInterface.OnClickListener() {
                                @Override
                                public void onClick(DialogInterface dialog, int which) {
                                    Activity activity = parent.getActivity();
                                    if (activity != null) {
                                        activity.finish();
                                    }
                                }
                            })
                    .create();
        }
    }

    private static native String Asuforia(long bmp_addr);

    private static native String nativePoseEstimation(int srcWidth, int srcHeight, ByteBuffer srcBuf, Surface dst, String path, long tempAddr);
}

	/**
     * onPose - Creating a view class that acts lika built-in view with custom attributes and support from layout editor.
	 * Defining the constructor as per our requirements
	 * OnTouchEvent - used for displaying the cube on every touch made by the user	
     */

class onPose extends SurfaceView implements SurfaceHolder.Callback{

    private final Paint paint;
    private final SurfaceHolder mHolder;
    private final Context context;
    static double[] point1 = new double[2];
    static double[] point2 = new double[2];
    static double[] point3 = new double[2];
    static double[] point4 = new double[2];
    static double[] point5 = new double[2];
    static double[] point6 = new double[2];
    static double[] point7 = new double[2];
    static double[] point8 = new double[2];

    static View view;


    public onPose(Camera2BasicFragment context, View view_t) {
        super(context.getActivity().getBaseContext());
        view = view_t;
        Log.d("onPose", "onPose");
        mHolder = getHolder();
        setZOrderOnTop(true);
        mHolder.setFormat(PixelFormat.TRANSPARENT);
        PixelFormat.formatHasAlpha(PixelFormat.TRANSPARENT);

        this.context = context.getActivity().getBaseContext();
        paint = new Paint(Paint.ANTI_ALIAS_FLAG);
        paint.setColor(Color.RED);
        paint.setStyle(Paint.Style.STROKE);

        if(getHolder().getSurface().isValid()){
            Log.d("Surface", "true");
            Canvas canvas = mHolder.lockCanvas();
            if(canvas != null) {
                canvas.drawColor(Color.TRANSPARENT);
                Paint paint = new Paint();
                paint.setColor(Color.BLUE);
                paint.setStrokeWidth(10f);
                canvas.drawLine(1000, 1200, 500, 700, paint);
                mHolder.unlockCanvasAndPost(canvas);
            }
        }

    }

    public onPose(Camera2BasicFragment context, double[] origin, double[] x, double[] y, double[] z, double[] diagonal_xy,
                      double[] x_3d, double[] y_3d, double[] diagonal_3d) {
        super(context.getActivity().getBaseContext());
        point1 = origin;
        point2 = x;
        point3 = y;
        point4 = z;
        point5 = diagonal_xy;
        point6 = x_3d;
        point7 = y_3d;
        point8 = diagonal_3d;
        Log.d("onPose", "onPose");
        mHolder = getHolder();
        setZOrderOnTop(true);
        mHolder.setFormat(PixelFormat.TRANSPARENT);
        PixelFormat.formatHasAlpha(PixelFormat.TRANSPARENT);

        this.context = context.getActivity().getBaseContext();
        paint = new Paint(Paint.ANTI_ALIAS_FLAG);
        paint.setColor(Color.RED);
        Log.d("PaintColor", "color1 "+paint.getColor());
        paint.setStyle(Paint.Style.STROKE);
    }

    @Override
    public boolean onTouchEvent(MotionEvent event) {
        if (event.getAction() == MotionEvent.ACTION_DOWN) {
            invalidate();
            if (mHolder.getSurface().isValid()) {
                final Canvas canvas = mHolder.lockCanvas();
                Log.d("touch", "touchRecieved by camera");
                if (canvas != null) {
                    Log.d("touch", "touchRecieved CANVAS STILL Not Null");
                    canvas.drawColor(Color.TRANSPARENT, PorterDuff.Mode.CLEAR);
                    canvas.drawColor(Color.TRANSPARENT);
                    Paint paint = new Paint();
                    paint.setColor(Color.BLUE);
                    paint.setStrokeWidth(10f);

                    //Draw Cube
                    canvas.drawLine((float) point1[0], (float) point1[1], (float) point2[0], (float) point2[1], paint);
                    canvas.drawLine((float) point2[0], (float) point2[1], (float) point5[0], (float) point5[1], paint);
                    canvas.drawLine((float) point5[0], (float) point5[1], (float) point3[0], (float) point3[1], paint);
                    canvas.drawLine((float) point3[0], (float) point3[1], (float) point1[0], (float) point1[1], paint);
                    canvas.drawLine((float) point4[0], (float) point4[1], (float) point6[0], (float) point6[1], paint);
                    canvas.drawLine((float) point6[0], (float) point6[1], (float) point8[0], (float) point8[1], paint);
                    canvas.drawLine((float) point8[0], (float) point8[1], (float) point7[0], (float) point7[1], paint);
                    canvas.drawLine((float) point7[0], (float) point7[1], (float) point4[0], (float) point4[1], paint);
                    canvas.drawLine((float) point3[0], (float) point3[1], (float) point7[0], (float) point7[1], paint);
                    canvas.drawLine((float) point1[0], (float) point1[1], (float) point4[0], (float) point4[1], paint);
                    canvas.drawLine((float) point2[0], (float) point2[1], (float) point6[0], (float) point6[1], paint);
                    canvas.drawLine((float) point5[0], (float) point5[1], (float) point8[0], (float) point8[1], paint);

                    mHolder.unlockCanvasAndPost(canvas);

                }

            }
        }


        return false;
    }

    @Override
    public void surfaceCreated(SurfaceHolder surfaceHolder) {

    }

    @Override
    public void surfaceChanged(SurfaceHolder surfaceHolder, int i, int i1, int i2) {

    }

    @Override
    public void surfaceDestroyed(SurfaceHolder surfaceHolder) {

    }
}