using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System;
using System.Threading;
using UnityEngine.UI;
using UnityEngine.EventSystems;
using System.IO;
using System.Xml.Serialization;

#if UNITY_5_3 || UNITY_5_3_OR_NEWER
using UnityEngine.SceneManagement;
#endif
using OpenCVForUnity;

namespace HoloLensWithOpenCVForUnityExample
{
    /// <summary>
    /// HoloLens ArUco Example
    /// An example of marker based AR using OpenCVForUnity on Hololens.
    /// Referring to https://github.com/opencv/opencv_contrib/blob/master/modules/aruco/samples/detect_markers.cpp.
    /// </summary>
    [RequireComponent(typeof(HololensCameraStreamToMatHelper))]
    public class HoloLensArUcoExample : MonoBehaviour
    {
 

        [HeaderAttribute("Detection")]

        /// <summary>
        /// Determines if restores the camera parameters when the file exists.
        /// </summary>
        public bool useStoredCameraParameters = false;

        /// <summary>
        /// The toggle for switching to use the stored camera parameters.
        /// </summary>
        public Toggle useStoredCameraParametersToggle;

        /// <summary>
        /// Determines if enables the detection.
        /// </summary>
        public bool enableDetection = true;


        [HeaderAttribute("AR")]

        /// <summary>
        /// Determines if applied the pose estimation.
        /// </summary>
        public bool applyEstimationPose = true;

        /// <summary>
        /// The dictionary identifier.
        /// </summary>
        public int dictionaryId = Aruco.DICT_6X6_250;

        /// <summary>
        /// The length of the markers' side. Normally, unit is meters.
        /// </summary>
        public float markerLength = 0.111f;

        /// <summary>
        /// The AR cube.
        /// </summary>
        public GameObject arCube;

        /// <summary>
        /// The AR game object.
        /// </summary>
        public GameObject arGameObject;

        /// <summary>
        /// The AR camera.
        /// </summary>
        public Camera arCamera;

        [Space(10)]

        /// <summary>
        /// Determines if enable low pass filter.
        /// </summary>
        public bool enableLowPassFilter;

        /// <summary>
        /// The enable low pass filter toggle.
        /// </summary>
        public Toggle enableLowPassFilterToggle;

        /// <summary>
        /// The position low pass. (Value in meters)
        /// </summary>
        public float positionLowPass = 0.025f;

        /// <summary>
        /// The rotation low pass. (Value in degrees)
        /// </summary>
        public float rotationLowPass = 3f;

        /// <summary>
        /// The old pose data.
        /// </summary>
        PoseData oldPoseData;

        /// <summary>
        /// The cameraparam matrix.
        /// </summary>
        Mat camMatrix;

        /// <summary>
        /// The matrix that inverts the Y-axis.
        /// </summary>
        Matrix4x4 invertYM;

        /// <summary>
        /// The matrix that inverts the Z-axis.
        /// </summary>
        Matrix4x4 invertZM;

        /// <summary>
        /// The transformation matrix.
        /// </summary>
        Matrix4x4 transformationM;

        /// <summary>
        /// The transformation matrix for AR.
        /// </summary>
        Matrix4x4 ARM;

        /// <summary>
        /// The identifiers.
        /// </summary>
        Mat ids;
        Mat ids_D;

        /// <summary>
        /// The corners.
        /// </summary>
        List<Mat> corners;
        List<Mat> corners_D;

        /// <summary>
        /// The rejected corners.
        /// </summary>
        List<Mat> rejectedCorners;
        List<Mat> rejectedCorners_D;

        /// <summary>
        /// The rvecs.
        /// </summary>
        Mat rvecs;

        /// <summary>
        /// The tvecs.
        /// </summary>
        Mat tvecs;

        /// <summary>
        /// The rot mat.
        /// </summary>
        Mat rotMat;

        /// <summary>
        /// The detector parameters.
        /// </summary>
        DetectorParameters detectorParams;
        DetectorParameters detectorParams_D;

        /// <summary>
        /// The dictionary.
        /// </summary>
        Dictionary dictionary;

        /// <summary>
        /// The webcam texture to mat helper.
        /// </summary>
        HololensCameraStreamToMatHelper webCamTextureToMatHelper;

        /// <summary>
        /// The image optimization helper.
        /// </summary>
        ImageOptimizationHelper imageOptimizationHelper;


        Mat grayMat;
        Mat rgbMat4preview;
        Texture2D texture;

        // The camera matrix value of Hololens camera 896x504 size.
        // For details on the camera matrix, please refer to this page. (http://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html)
        // These values   are unique to my device, obtained from the "Windows.Media.Devices.Core.CameraIntrinsics" class. (https://docs.microsoft.com/en-us/uwp/api/windows.media.devices.core.cameraintrinsics)
        // Can get these values by using this helper script. (https://github.com/EnoxSoftware/HoloLensWithOpenCVForUnityExample/tree/master/Assets/HololensCameraIntrinsicsChecker/CameraIntrinsicsCheckerHelper)
        double fx = 1035.149;//focal length x.
        double fy = 1034.633;//focal length y.
        double cx = 404.9134;//principal point x.
        double cy = 236.2834;//principal point y.
        MatOfDouble distCoeffs;
        double distCoeffs1 = 0.2036923;//radial distortion coefficient k1.
        double distCoeffs2 = -0.2035773;//radial distortion coefficient k2.
        double distCoeffs3 = 0.0;//tangential distortion coefficient p1.
        double distCoeffs4 = 0.0;//tangential distortion coefficient p2.
        double distCoeffs5 = -0.2388065;//radial distortion coefficient k3.

        readonly static Queue<Action> ExecuteOnMainThread = new Queue<Action>();
        System.Object sync = new System.Object();
        Mat downScaleFrameMat;

        MatOfPoint2f mMOP2fptsPrev = new MatOfPoint2f();

        Mat matOpFlowThis = new Mat();
        Mat matOpFlowPrev = new Mat();
        MatOfPoint2f mMOP2fptsThis = new MatOfPoint2f();
        MatOfPoint2f obj2Points_D;
        MatOfPoint2f obj2Points;
        MatOfPoint3f obj3Points;
        
        MatOfByte mMOBStatus = new MatOfByte();
        MatOfPoint MOPcorners = new MatOfPoint();
        MatOfFloat mMOFerr = new MatOfFloat();

        Point l2_d4;
        Point l2_u1;
        Point r2_d3;
        Point r2_u2;

        MatOfDouble dist_double;

        //새로 추가한 변수
        int k, l;

        int markerHeight1;
        int markerHeight2;
        int markerWidth1;
        int markerWidth2;
        int markerHeight;
        int markerWidth;
        int oneWidth;
        int oneHeight;
        int index_x;
        int index_y;

        MatOfPoint2f goodFeature = new MatOfPoint2f();
        Mat roiMat = new Mat();
        float distance = 0;

        bool isDetect = true;
        double[] row1_2d;
        double[] row2_2d;
        double[] row3_2d;
        double[] row4_2d;
        Point3 p3d1;
        Point3 p3d2;
        Point3 p3d3;
        Point3 p3d4;
        Point3 p3d5;
        Point3 p3d6;

        Point p2d1;
        Point p2d2;
        Point p2d3;
        Point p2d4;
        Point p2d5;
        Point p2d6;
        Scalar colorRed = new Scalar(255, 0, 0, 255);
        Scalar colorGreen = new Scalar(0, 255, 0, 255);
        Scalar colorBlue = new Scalar(0, 0, 255, 255);
        Point[,] p2d = new Point[1, 6];
        Point3[,] solve_p3d = new Point3[1, 7];
        int[] xArr = new int[7];
        int[] yArr = new int[7];
        Point3[,] p3d = new Point3[9, 9];
        bool _isThreadRunning = false;
        bool isThreadRunning
        {
            get
            {
                lock (sync)
                    return _isThreadRunning;
            }
            set
            {
                lock (sync)
                    _isThreadRunning = value;
            }
        }

        bool _isDetecting = false;
        bool isDetecting
        {
            get
            {
                lock (sync)
                    return _isDetecting;
            }
            set
            {
                lock (sync)
                    _isDetecting = value;
            }
        }

        bool _hasUpdatedARTransformMatrix = false;
        bool hasUpdatedARTransformMatrix
        {
            get
            {
                lock (sync)
                    return _hasUpdatedARTransformMatrix;
            }
            set
            {
                lock (sync)
                    _hasUpdatedARTransformMatrix = value;
            }
        }

        // Use this for initialization
        void Start()
        {
            useStoredCameraParametersToggle.isOn = useStoredCameraParameters;
            enableLowPassFilterToggle.isOn = enableLowPassFilter;

            imageOptimizationHelper = gameObject.GetComponent<ImageOptimizationHelper>();
            webCamTextureToMatHelper = gameObject.GetComponent<HololensCameraStreamToMatHelper>();
#if NETFX_CORE && !DISABLE_HOLOLENSCAMSTREAM_API
            webCamTextureToMatHelper.frameMatAcquired += OnFrameMatAcquired;
#endif



            webCamTextureToMatHelper.Initialize();



            /////////////////홀로렌즈 카메라 위치 설정 ///////////////////////
            transform.SetParent(arCamera.transform, true);
    
        }

        float Dist_DvsT(MatOfPoint2f point_D, MatOfPoint2f point_T)
        {
            Size s1 = new Size(1, 4);
            Size s2 = new Size(1, 5);
            Size s3 = new Size(1, 6);
            float d = 0;
            float D1, D2, D3, D4, D5, D6;

            if (point_D.size().Equals(s1))
            {
                //   print("1x4");
                D1 = Get_dist_Double(point_D.get(0, 0), point_T.get(0, 0));
                D2 = Get_dist_Double(point_D.get(1, 0), point_T.get(1, 0));
                D3 = Get_dist_Double(point_D.get(2, 0), point_T.get(2, 0));
                D4 = Get_dist_Double(point_D.get(3, 0), point_T.get(3, 0));


                if (D1 > 2 || D2 > 2 || D3 > 2 || D4 > 2)
                {
                    isDetect = true;
                }



                d = (D1 + D2 + D3 + D4) / 4;

            }
            else if (point_D.size().Equals(s2))
            {
                // print("1x5");
                D1 = Get_dist_Double(point_D.get(0, 0), point_T.get(0, 0));
                D2 = Get_dist_Double(point_D.get(1, 0), point_T.get(1, 0));
                D3 = Get_dist_Double(point_D.get(2, 0), point_T.get(2, 0));
                D4 = Get_dist_Double(point_D.get(3, 0), point_T.get(3, 0));
                D5 = Get_dist_Double(point_D.get(4, 0), point_T.get(4, 0));

                d = (D1 + D2 + D3 + D4 + D5) / 5;

                if (D1 > 2 || D2 > 2 || D3 > 2 || D4 > 2 || D5 > 2)
                {
                    isDetect = true;
                }


            }
            else if (point_D.size().Equals(s3))
            {
                //  print("1x6");
                D1 = Get_dist_Double(point_D.get(0, 0), point_T.get(0, 0));
                D2 = Get_dist_Double(point_D.get(1, 0), point_T.get(1, 0));
                D3 = Get_dist_Double(point_D.get(2, 0), point_T.get(2, 0));
                D4 = Get_dist_Double(point_D.get(3, 0), point_T.get(3, 0));
                D5 = Get_dist_Double(point_D.get(4, 0), point_T.get(4, 0));
                D6 = Get_dist_Double(point_D.get(5, 0), point_T.get(5, 0));

                d = (D1 + D2 + D3 + D4 + D5 + D6) / 6;

                if (D1 > 2 || D2 > 2 || D3 > 2 || D4 > 2 || D5 > 2 || D6 > 2)
                {
                    isDetect = true;
                }


            }
            return d;
        }

        float Get_dist_Double(double[] pt1, double[] pt2)
        {
            double[] output = new double[2];
            float distance = 0;

            output[0] = Mathf.Pow(Mathf.Abs((float)pt1[0] - (float)pt2[0]), 2);
            output[1] = Mathf.Pow(Mathf.Abs((float)pt1[1] - (float)pt2[1]), 2);
            distance = (float)output[0] + (float)output[1];
            Mathf.Sqrt(distance);
            return distance;
        }

        /// <summary>
        /// Raises the web cam texture to mat helper initialized event.
        /// </summary>
        public void OnWebCamTextureToMatHelperInitialized()
        {
            Debug.Log("OnWebCamTextureToMatHelperInitialized");

            Mat rawSizeMat = webCamTextureToMatHelper.GetMat();
            float rawSizeWidth = rawSizeMat.width();
            float rawSizeHeight = rawSizeMat.height();

            Mat webCamTextureMat = imageOptimizationHelper.GetDownScaleMat(rawSizeMat);

            Debug.Log("Screen.width " + Screen.width + " Screen.height " + Screen.height + " Screen.orientation " + Screen.orientation);

            float width = webCamTextureMat.width();
            float height = webCamTextureMat.height();

            texture = new Texture2D((int)width, (int)height, TextureFormat.RGB24, false);

      
            // set camera parameters.
            double fx;
            double fy;
            double cx;
            double cy;

            string loadDirectoryPath = Path.Combine(Application.persistentDataPath, "HoloLensArUcoCameraCalibrationExample");
            string calibratonDirectoryName = "camera_parameters" + rawSizeWidth + "x" + rawSizeHeight;
            string loadCalibratonFileDirectoryPath = Path.Combine(loadDirectoryPath, calibratonDirectoryName);
            string loadPath = Path.Combine(loadCalibratonFileDirectoryPath, calibratonDirectoryName + ".xml");
            if (useStoredCameraParameters && File.Exists(loadPath))
            {
                CameraParameters param;
                XmlSerializer serializer = new XmlSerializer(typeof(CameraParameters));
                using (var stream = new FileStream(loadPath, FileMode.Open))
                {
                    param = (CameraParameters)serializer.Deserialize(stream);
                }

                fx = param.camera_matrix[0];
                fy = param.camera_matrix[4];
                cx = param.camera_matrix[2] / imageOptimizationHelper.downscaleRatio;
                cy = param.camera_matrix[5] / imageOptimizationHelper.downscaleRatio;

                camMatrix = new Mat(3, 3, CvType.CV_64FC1);
                camMatrix.put(0, 0, fx);
                camMatrix.put(0, 1, 0);
                camMatrix.put(0, 2, cx);
                camMatrix.put(1, 0, 0);
                camMatrix.put(1, 1, fy);
                camMatrix.put(1, 2, cy);
                camMatrix.put(2, 0, 0);
                camMatrix.put(2, 1, 0);
                camMatrix.put(2, 2, 1.0f);

                Debug.Log("Loaded CameraParameters from a stored XML file.");
                Debug.Log("loadPath: " + loadPath);

            }
            else
            {
                fx = this.fx;
                fy = this.fy;
                cx = this.cx / imageOptimizationHelper.downscaleRatio;
                cy = this.cy / imageOptimizationHelper.downscaleRatio;

                camMatrix = new Mat(3, 3, CvType.CV_64FC1);
                camMatrix.put(0, 0, fx);
                camMatrix.put(0, 1, 0);
                camMatrix.put(0, 2, cx);
                camMatrix.put(1, 0, 0);
                camMatrix.put(1, 1, fy);
                camMatrix.put(1, 2, cy);
                camMatrix.put(2, 0, 0);
                camMatrix.put(2, 1, 0);
                camMatrix.put(2, 2, 1.0f);

                distCoeffs = new MatOfDouble(this.distCoeffs1, this.distCoeffs2, this.distCoeffs3, this.distCoeffs4, this.distCoeffs5);

                Debug.Log("Created a dummy CameraParameters.");
            }

            ///////////////오른쪽 주석은 학교 Hololens 기준 값 /////////////////
            Debug.Log("camMatrix " + camMatrix.dump());   // camMatrix [ 1035.149, 0, 134.9711333333333 ]
            Debug.Log("distCoeffs " + distCoeffs.dump()); // distCoeffs [ 0.2036923, -0.2035773 ] 


            //Calibration camera
            Size imageSize = new Size(width, height);
            double apertureWidth = 0;
            double apertureHeight = 0;
            double[] fovx = new double[1];
            double[] fovy = new double[1];
            double[] focalLength = new double[1];
            Point principalPoint = new Point(0, 0);
            double[] aspectratio = new double[1];

            Calib3d.calibrationMatrixValues(camMatrix, imageSize, apertureWidth, apertureHeight, fovx, fovy, focalLength, principalPoint, aspectratio);

            Debug.Log("imageSize " + imageSize.ToString());
            Debug.Log("apertureWidth " + apertureWidth);
            Debug.Log("apertureHeight " + apertureHeight);
            Debug.Log("fovx " + fovx[0]);
            Debug.Log("fovy " + fovy[0]);
            Debug.Log("focalLength " + focalLength[0]);
            Debug.Log("principalPoint " + principalPoint.ToString());
            Debug.Log("aspectratio " + aspectratio[0]);

            // Display objects near the camera.
            arCamera.nearClipPlane = 0.01f;

            grayMat = new Mat();
            ids = new Mat();
            ids_D = new Mat();
            corners = new List<Mat>();
            corners_D = new List<Mat>();
            rejectedCorners = new List<Mat>();
            rejectedCorners_D = new List<Mat>();

            rvecs = new Mat();
            tvecs = new Mat();
            rotMat = new Mat(3, 3, CvType.CV_64FC1);


            transformationM = new Matrix4x4();

            invertYM = Matrix4x4.TRS(Vector3.zero, Quaternion.identity, new Vector3(1, -1, 1));
            Debug.Log("invertYM " + invertYM.ToString());

            invertZM = Matrix4x4.TRS(Vector3.zero, Quaternion.identity, new Vector3(1, 1, -1));
            Debug.Log("invertZM " + invertZM.ToString());

            detectorParams = DetectorParameters.create();
            detectorParams_D = DetectorParameters.create();
            dictionary = Aruco.getPredefinedDictionary(Aruco.DICT_6X6_250);


            //If WebCamera is frontFaceing,flip Mat.
            if (webCamTextureToMatHelper.GetWebCamDevice().isFrontFacing)
            {
                webCamTextureToMatHelper.flipHorizontal = true;
            }

            downScaleFrameMat = new Mat((int)height, (int)width, CvType.CV_8UC4);
            rgbMat4preview = new Mat();
        }

        /// <summary>
        /// Raises the web cam texture to mat helper disposed event.
        /// </summary>
        public void OnWebCamTextureToMatHelperDisposed()
        {
            Debug.Log("OnWebCamTextureToMatHelperDisposed");

#if !NETFX_CORE || DISABLE_HOLOLENSCAMSTREAM_API
            StopThread();
            lock (sync)
            {
                ExecuteOnMainThread.Clear();
            }
            isDetecting = false;
#endif
            hasUpdatedARTransformMatrix = false;

            if (grayMat != null)
                grayMat.Dispose();
            if (ids != null)
                ids.Dispose();
            if (ids_D != null)
                ids_D.Dispose();
            foreach (var item in corners)
            {
                item.Dispose();
            }
            corners.Clear();
            foreach (var item in corners_D)
            {
                item.Dispose();
            }
            corners_D.Clear();
            foreach (var item in rejectedCorners)
            {
                item.Dispose();
            }
            rejectedCorners.Clear();
            foreach (var item in rejectedCorners_D)
            {
                item.Dispose();
            }
            rejectedCorners_D.Clear();
            if (rvecs != null)
                rvecs.Dispose();
            if (tvecs != null)
                tvecs.Dispose();
            if (rotMat != null)
                rotMat.Dispose();

            if (rgbMat4preview != null)
                rgbMat4preview.Dispose();
        }

        /// <summary>
        /// Raises the web cam texture to mat helper error occurred event.
        /// </summary>
        /// <param name="errorCode">Error code.</param>
        public void OnWebCamTextureToMatHelperErrorOccurred(WebCamTextureToMatHelper.ErrorCode errorCode)
        {
            Debug.Log("OnWebCamTextureToMatHelperErrorOccurred " + errorCode);
        }

#if NETFX_CORE && !DISABLE_HOLOLENSCAMSTREAM_API
         public void OnFrameMatAcquired(Mat bgraMat, Matrix4x4 projectionMatrix, Matrix4x4 cameraToWorldMatrix)
         {
            downScaleFrameMat = imageOptimizationHelper.GetDownScaleMat(bgraMat);

            if (enableDetection) {

               Imgproc.cvtColor(downScaleFrameMat, grayMat, Imgproc.COLOR_BGRA2GRAY);

               // Detect markers and estimate Pose
               Aruco.detectMarkers(grayMat, dictionary, corners, ids, detectorParams, rejectedCorners, camMatrix, distCoeffs);

               if (applyEstimationPose && ids.total() > 0) {
                  Aruco.estimatePoseSingleMarkers(corners, markerLength, camMatrix, distCoeffs, rvecs, tvecs);

                  for (int i = 0; i < ids.total(); i++) {

                     //This example can display ARObject on only first detected marker.
                     if (i == 0) {

                        // Convert to unity pose data.
                        double[] rvecArr = new double[3];
                        rvecs.get(0, 0, rvecArr);
                        double[] tvecArr = new double[3];
                        tvecs.get(0, 0, tvecArr);
                        tvecArr[2] /= imageOptimizationHelper.downscaleRatio;
                        PoseData poseData = ARUtils.ConvertRvecTvecToPoseData(rvecArr, tvecArr);

                        // Changes in pos/rot below these thresholds are ignored.
                        if (enableLowPassFilter) {
                           ARUtils.LowpassPoseData(ref oldPoseData, ref poseData, positionLowPass, rotationLowPass);
                        }
                        oldPoseData = poseData;

                        // Create transform matrix.
                        transformationM = Matrix4x4.TRS(poseData.pos, poseData.rot, Vector3.one);

                        lock(sync) {
                           // Right-handed coordinates system (OpenCV) to left-handed one (Unity)
                           ARM = invertYM * transformationM;

                           // Apply Z-axis inverted matrix.
                           ARM = ARM * invertZM;
                        }

                        hasUpdatedARTransformMatrix = true;

                        break;
                     }
                  }
               }
            }

            Mat rgbMat4preview = null;
            if (displayCameraPreview) {
               rgbMat4preview = new Mat();
               Imgproc.cvtColor(downScaleFrameMat, rgbMat4preview, Imgproc.COLOR_BGRA2RGB);

               if (ids.total() > 0) {
                  Aruco.drawDetectedMarkers(rgbMat4preview, corners, ids, new Scalar(0, 255, 0));

                  if (applyEstimationPose) {
                     for (int i = 0; i < ids.total(); i++) {
                        using (Mat rvec = new Mat(rvecs, new OpenCVForUnity.Rect(0, i, 1, 1)))
                           using (Mat tvec = new Mat(tvecs, new OpenCVForUnity.Rect(0, i, 1, 1))) {

                           // In this example we are processing with RGB color image, so Axis-color correspondences are X: blue, Y: green, Z: red. (Usually X: red, Y: green, Z: blue)
                           Aruco.drawAxis(rgbMat4preview, camMatrix, distCoeffs, rvec, tvec, markerLength * 0.5f);
                        }
                     }
                  }
               }
            }


            UnityEngine.WSA.Application.InvokeOnAppThread(() => {

               if (!webCamTextureToMatHelper.IsPlaying()) return;

               if (displayCameraPreview && rgbMat4preview != null) {
                  OpenCVForUnity.Utils.fastMatToTexture2D(rgbMat4preview, texture);
               }

               if (applyEstimationPose) {
                  if (hasUpdatedARTransformMatrix) {
                     hasUpdatedARTransformMatrix = false;

                     lock(sync) {
                        // Apply camera transform matrix.
                        ARM = cameraToWorldMatrix * invertZM * ARM;

                        ARUtils.SetTransformFromMatrix(arGameObject.transform, ref ARM);
                     }
                  }
               }

               bgraMat.Dispose();
               if (rgbMat4preview != null) {
                  rgbMat4preview.Dispose();
               }

            }, false);
         }
     
        
#else

        // Update is called once per frame
        void Update()
        {
            lock (sync)
            {
                while (ExecuteOnMainThread.Count > 0)
                {
                    ExecuteOnMainThread.Dequeue().Invoke();
                }
            }

            if (webCamTextureToMatHelper.IsPlaying() && webCamTextureToMatHelper.DidUpdateThisFrame())
            {
   

                if (enableDetection && !isDetecting)
                {
                    dist_double = distCoeffs;
                    isDetecting = true;

                    downScaleFrameMat = imageOptimizationHelper.GetDownScaleMat(webCamTextureToMatHelper.GetMat());

                    StartThread(ThreadWorker);
                }
            }
        }

        private void StartThread(Action action)
        {
#if UNITY_METRO && NETFX_CORE
            System.Threading.Tasks.Task.Run(() => action());
#elif UNITY_METRO
            action.BeginInvoke(ar => action.EndInvoke(ar), null);
#else
            ThreadPool.QueueUserWorkItem (_ => action());
#endif
        }

        private void StopThread()
        {
            if (!isThreadRunning)
                return;

            while (isThreadRunning)
            {
                //Wait threading stop
            }
        }


        private void ThreadWorker()
        {
            isThreadRunning = true;

            DetectARUcoMarker();

            lock (sync)
            {
                if (ExecuteOnMainThread.Count == 0)
                {
                    ExecuteOnMainThread.Enqueue(() =>
                    {
                        OnDetectionDone();
                    });
                }
            }

            isThreadRunning = false;
        }


        float Get_dist_Point(Point pt1, Point pt2)
        {
            Point output = new Point();
            float d = 0;

            output.x = Mathf.Pow(Mathf.Abs((float)pt1.x - (float)pt2.x), 2);
            output.y = Mathf.Pow(Mathf.Abs((float)pt1.y - (float)pt2.y), 2);
            d = (float)output.x + (float)output.y;
            Mathf.Sqrt(d);
            //print("distance::;" + d);
            return d;
        }

        private void DetectARUcoMarker()
        {
            ////////////////////////// 3차원 좌표  /////////////////////////////////
            float length = markerLength / 8;
            //  print("length:: " + length);

            Point3 L_U1_0 = new Point3(-markerLength / 2, markerLength / 2, 0); // 3d (1,1)
            Point3 x11 = new Point3(L_U1_0.x + length, L_U1_0.y, 0);
            Point3 x12 = new Point3(x11.x + length, x11.y, 0);
            Point3 x13 = new Point3(x12.x + length, x12.y, 0);
            Point3 x14 = new Point3(x13.x + length, x13.y, 0);
            Point3 x15 = new Point3(x14.x + length, x14.y, 0);
            Point3 x16 = new Point3(x15.x + length, x15.y, 0);
            Point3 x17 = new Point3(x16.x + length, x16.y, 0);
            //  print("17 x:: " + x17.x + " y:: " + x17.y);
            Point3 R_U2_0 = new Point3(markerLength / 2, markerLength / 2, 0); // 3d (1, 8)

            Point3 x20 = new Point3(L_U1_0.x, L_U1_0.y - length, 0);  // 3d (2,1)
            Point3 x21 = new Point3(x20.x + length, x20.y, 0);
            Point3 x22 = new Point3(x21.x + length, x21.y, 0);
            Point3 x23 = new Point3(x22.x + length, x22.y, 0);
            Point3 x24 = new Point3(x23.x + length, x23.y, 0);
            Point3 x25 = new Point3(x24.x + length, x24.y, 0);
            Point3 x26 = new Point3(x25.x + length, x25.y, 0);
            Point3 x27 = new Point3(x26.x + length, x26.y, 0);
            //  print("27 x:: " + x27.x + " y:: " + x27.y);
            Point3 x28 = new Point3(x27.x + length, x27.y, 0);
            //  print("2 x:: " + x28.x + " y:: " + x28.y);


            Point3 x30 = new Point3(x20.x, x20.y - length, 0);  // 3d (3, 1)
            Point3 x31 = new Point3(x30.x + length, x30.y, 0);
            Point3 x32 = new Point3(x31.x + length, x31.y, 0);
            Point3 x33 = new Point3(x32.x + length, x32.y, 0);
            Point3 x34 = new Point3(x33.x + length, x33.y, 0);
            Point3 x35 = new Point3(x34.x + length, x34.y, 0);
            Point3 x36 = new Point3(x35.x + length, x35.y, 0);
            Point3 x37 = new Point3(x36.x + length, x36.y, 0);
            Point3 x38 = new Point3(x37.x + length, x37.y, 0);
            //  print("3 x:: " + x38.x + " y:: " + x38.y);


            Point3 x40 = new Point3(x30.x, x30.y - length, 0); // 3d (4, 1)
            Point3 x41 = new Point3(x40.x + length, x40.y, 0);
            Point3 x42 = new Point3(x41.x + length, x41.y, 0);
            Point3 x43 = new Point3(x42.x + length, x42.y, 0);
            Point3 x44 = new Point3(x43.x + length, x43.y, 0);
            Point3 x45 = new Point3(x44.x + length, x44.y, 0);
            Point3 x46 = new Point3(x45.x + length, x45.y, 0);
            Point3 x47 = new Point3(x46.x + length, x46.y, 0);
            Point3 x48 = new Point3(x47.x + length, x47.y, 0);
            // print("4 x:: " + x48.x + " y:: " + x48.y);


            Point3 x50 = new Point3(x40.x, x40.y - length, 0); // 3d (5, 1)
            Point3 x51 = new Point3(x50.x + length, x50.y, 0);
            Point3 x52 = new Point3(x51.x + length, x51.y, 0);
            Point3 x53 = new Point3(x52.x + length, x52.y, 0);
            Point3 x54 = new Point3(x53.x + length, x53.y, 0);
            Point3 x55 = new Point3(x54.x + length, x54.y, 0);
            Point3 x56 = new Point3(x55.x + length, x55.y, 0);
            Point3 x57 = new Point3(x56.x + length, x56.y, 0);
            Point3 x58 = new Point3(x57.x + length, x57.y, 0);
            //print("5 x:: " + x58.x + " y:: " + x58.y);



            Point3 x60 = new Point3(x50.x, x50.y - length, 0); // 3d (6, 1)
            Point3 x61 = new Point3(x60.x + length, x60.y, 0);
            Point3 x62 = new Point3(x61.x + length, x61.y, 0);
            Point3 x63 = new Point3(x62.x + length, x62.y, 0);
            Point3 x64 = new Point3(x63.x + length, x63.y, 0);
            Point3 x65 = new Point3(x64.x + length, x64.y, 0);
            Point3 x66 = new Point3(x65.x + length, x65.y, 0);
            Point3 x67 = new Point3(x66.x + length, x66.y, 0);
            Point3 x68 = new Point3(x67.x + length, x67.y, 0);
            // print("6 x:: " + x68.x + " y:: " + x68.y);


            Point3 x70 = new Point3(x60.x, x60.y - length, 0); // 3d (7, 1)
            Point3 x71 = new Point3(x70.x + length, x70.y, 0);
            Point3 x72 = new Point3(x71.x + length, x71.y, 0);
            Point3 x73 = new Point3(x72.x + length, x72.y, 0);
            Point3 x74 = new Point3(x73.x + length, x73.y, 0);
            Point3 x75 = new Point3(x74.x + length, x74.y, 0);
            Point3 x76 = new Point3(x75.x + length, x75.y, 0);
            Point3 x77 = new Point3(x76.x + length, x76.y, 0);
            Point3 x78 = new Point3(x77.x + length, x77.y, 0);
            //print("7 x:: " + x78.x + " y:: " + x78.y);

            Point3 x80 = new Point3(x70.x, x70.y - length, 0); // 3d (8, 1)
            Point3 x81 = new Point3(x80.x + length, x80.y, 0);
            Point3 x82 = new Point3(x81.x + length, x81.y, 0);
            Point3 x83 = new Point3(x82.x + length, x82.y, 0);
            Point3 x84 = new Point3(x83.x + length, x83.y, 0);
            Point3 x85 = new Point3(x84.x + length, x84.y, 0);
            Point3 x86 = new Point3(x85.x + length, x85.y, 0);
            Point3 x87 = new Point3(x86.x + length, x86.y, 0);
            Point3 x88 = new Point3(x87.x + length, x87.y, 0);
            //print("8 x:: " + x88.x + " y:: " + x88.y);

            Point3 L_D4_0 = new Point3(-markerLength / 2, -markerLength / 2, 0); // 3d (9,1)
            Point3 x91 = new Point3(L_D4_0.x + length, L_D4_0.y, 0);
            Point3 x92 = new Point3(x91.x + length, x91.y, 0);
            Point3 x93 = new Point3(x92.x + length, x92.y, 0);
            Point3 x94 = new Point3(x93.x + length, x93.y, 0);
            Point3 x95 = new Point3(x94.x + length, x94.y, 0);
            Point3 x96 = new Point3(x95.x + length, x95.y, 0);
            Point3 x97 = new Point3(x96.x + length, x96.y, 0);
            //print("9 x:: " + x97.x + " y:: " + x97.y);
            Point3 R_D3_0 = new Point3(markerLength / 2, -markerLength / 2, 0);

            // 각 3d좌표값에 해당하는 index값들 index / 3point 
            p3d[0, 0] = L_U1_0;
            p3d[0, 1] = x11;
            p3d[0, 2] = x12;
            p3d[0, 3] = x13;
            p3d[0, 4] = x14;
            p3d[0, 5] = x15;
            p3d[0, 6] = x16;
            p3d[0, 7] = x17;
            p3d[0, 8] = R_U2_0;

            p3d[1, 0] = x20;
            p3d[1, 1] = x21;
            p3d[1, 2] = x22;
            p3d[1, 3] = x23;
            p3d[1, 4] = x24;
            p3d[1, 5] = x25;
            p3d[1, 6] = x26;
            p3d[1, 7] = x27;
            p3d[1, 8] = x28;

            p3d[2, 0] = x30;
            p3d[2, 1] = x31;
            p3d[2, 2] = x32;
            p3d[2, 3] = x33;
            p3d[2, 4] = x34;
            p3d[2, 5] = x35;
            p3d[2, 6] = x36;
            p3d[2, 7] = x37;
            p3d[2, 8] = x38;

            p3d[3, 0] = x40;
            p3d[3, 1] = x41;
            p3d[3, 2] = x42;
            p3d[3, 3] = x43;
            p3d[3, 4] = x44;
            p3d[3, 5] = x45;
            p3d[3, 6] = x46;
            p3d[3, 7] = x47;
            p3d[3, 8] = x48;

            p3d[4, 0] = x50;
            p3d[4, 1] = x51;
            p3d[4, 2] = x52;
            p3d[4, 3] = x53;
            p3d[4, 4] = x54;
            p3d[4, 5] = x55;
            p3d[4, 6] = x56;
            p3d[4, 7] = x57;
            p3d[4, 8] = x58;

            p3d[5, 0] = x60;
            p3d[5, 1] = x61;
            p3d[5, 2] = x62;
            p3d[5, 3] = x63;
            p3d[5, 4] = x64;
            p3d[5, 5] = x65;
            p3d[5, 6] = x66;
            p3d[5, 7] = x67;
            p3d[5, 8] = x68;

            p3d[6, 0] = x70;
            p3d[6, 1] = x71;
            p3d[6, 2] = x72;
            p3d[6, 3] = x73;
            p3d[6, 4] = x74;
            p3d[6, 5] = x75;
            p3d[6, 6] = x76;
            p3d[6, 7] = x77;
            p3d[6, 8] = x78;

            p3d[7, 0] = x80;
            p3d[7, 1] = x81;
            p3d[7, 2] = x82;
            p3d[7, 3] = x83;
            p3d[7, 4] = x84;
            p3d[7, 5] = x85;
            p3d[7, 6] = x86;
            p3d[7, 7] = x87;
            p3d[7, 8] = x88;

            p3d[8, 0] = L_D4_0;
            p3d[8, 1] = x91;
            p3d[8, 2] = x92;
            p3d[8, 3] = x93;
            p3d[8, 4] = x94;
            p3d[8, 5] = x95;
            p3d[8, 6] = x96;
            p3d[8, 7] = x97;
            p3d[8, 8] = R_D3_0;


            //초기 detect
            if (mMOP2fptsPrev.rows() == 0 || isDetect)
            {

                Imgproc.cvtColor(downScaleFrameMat, matOpFlowThis, Imgproc.COLOR_RGBA2GRAY);
                matOpFlowThis.copyTo(matOpFlowPrev);    //이전 프레임에 현재 프레임 복사

                Aruco.detectMarkers(matOpFlowPrev, dictionary, corners, ids, detectorParams, rejectedCorners, camMatrix, distCoeffs);   //마커 검출
                //Debug.Log("ids total ::: " + ids.total());

                if (applyEstimationPose && ids.total() > 0)
                {   //마커가 검출되면 solvePnP를 위해 objpoint로 변경 후
                    //MOPfptsPrev에 코너점 복사
                    Mat selected_corner = corners[0];
                    MatOfPoint2f point1 = new MatOfPoint2f(selected_corner);

                    l2_u1 = new Point(point1.get(0, 0)[0], point1.get(0, 0)[1]);
                    r2_u2 = new Point(point1.get(0, 1)[0], point1.get(0, 1)[1]);
                    r2_d3 = new Point(point1.get(0, 2)[0], point1.get(0, 2)[1]);
                    l2_d4 = new Point(point1.get(0, 3)[0], point1.get(0, 3)[1]);
                    markerHeight1 = (int)Get_dist_Point(l2_u1, l2_d4);
                    markerHeight2 = (int)Get_dist_Point(r2_d3, r2_u2);
                    markerWidth1 = (int)Get_dist_Point(l2_u1, r2_u2);
                    markerWidth2 = (int)Get_dist_Point(l2_d4, r2_d3);
                    markerHeight = (markerHeight1 + markerHeight2) / 2;
                    markerWidth = (markerWidth1 + markerWidth2) / 2;
                    oneWidth = (int)Mathf.Sqrt(markerWidth) / 8;
                    oneHeight = (int)Mathf.Sqrt(markerHeight) / 8;

                    // Debug.Log("corner2::: " + l2_u1 + " " + l2_d4 + " " + r2_d3 + " " + r2_u2);
                    OpenCVForUnity.Rect rect = new OpenCVForUnity.Rect((int)l2_u1.x + oneWidth, (int)l2_u1.y + oneHeight, oneWidth * 6, oneWidth * 6);

                    roiMat = new Mat(matOpFlowPrev, rect);
                    //Imgproc.cvtColor(roiMat, roigMat, Imgproc.COLOR_RGBA2GRAY);

                    Imgproc.goodFeaturesToTrack(roiMat, MOPcorners, 40, 0.05, 20);

                    goodFeature.fromArray(MOPcorners.toArray());
                    //  print("mMOPfpts " + goodFeature.rows());
                    List<Point> cornersPrev = goodFeature.toList();

                    for (int i = 0; i < goodFeature.rows(); i++)
                    {//전체 Mat에서 0,0부터 맵핑됨을 방지
                        cornersPrev[i].x = cornersPrev[i].x + l2_u1.x + oneWidth;
                        cornersPrev[i].y = cornersPrev[i].y + l2_u1.y + oneHeight;
                        k = (int)cornersPrev[i].x;
                        l = (int)cornersPrev[i].y;
                        Point b = new Point(k, l);

                    }
                    downScaleFrameMat.copyTo(roiMat);

                    if (goodFeature.rows() <= 0)
                    {   //goddfeature가 하나도 안 나온 경우
                        double i = cornersPrev[0].x;
                        double j = cornersPrev[0].y;

                        cornersPrev[0] = new Point(i, j);

                        mMOP2fptsPrev = new MatOfPoint2f(l2_u1, l2_d4, r2_d3, r2_u2);

                        // print("1 :: " + l2_u1 + " " + l2_d4 + " " + r2_d3 + " " + r2_u2 + " " + cornersPrev[0]);

                    }
                    else if (goodFeature.rows() < 2)
                    {//goodfeature가 1개 나온 경우
                        double i = cornersPrev[0].x;
                        double j = cornersPrev[0].y;


                        cornersPrev[0] = new Point(i, j);


                        mMOP2fptsPrev = new MatOfPoint2f(l2_u1, l2_d4, r2_d3, r2_u2, cornersPrev[0]);
                        // print("22 :: " + l2_u1 + " " + l2_d4 + " " + r2_d3 + " " + r2_u2 + " " + cornersPrev[0]);

                    }
                    else if (goodFeature.rows() > 2)
                    {//goodfeature가 2개 이상 나온 경우
                        double i = cornersPrev[0].x;
                        double j = cornersPrev[0].y;

                        double z = cornersPrev[1].x;
                        double u = cornersPrev[1].y;

                        cornersPrev[0] = new Point(i, j);
                        cornersPrev[1] = new Point(z, u);

                        mMOP2fptsPrev = new MatOfPoint2f(l2_u1, l2_d4, r2_d3, r2_u2, cornersPrev[0], cornersPrev[1]);
                        //print("33 :: " + l2_u1 + " " + l2_d4 + " " + r2_d3 + " " + r2_u2 + " " + cornersPrev[0] + " " + cornersPrev[1]);

                    }

                    downScaleFrameMat.copyTo(roiMat);
                    obj2Points_D = mMOP2fptsPrev;
                    isDetect = false;

                    //Debug.Log("point1::: " + MOPcorners);
                }
            }
            else
            {
                print("tracking");
                matOpFlowThis.copyTo(matOpFlowPrev);
                Imgproc.cvtColor(downScaleFrameMat, matOpFlowThis, Imgproc.COLOR_RGBA2GRAY);//2

                mMOP2fptsThis.copyTo(mMOP2fptsPrev);
            }

            Video.calcOpticalFlowPyrLK(matOpFlowPrev, matOpFlowThis, mMOP2fptsPrev, mMOP2fptsThis, mMOBStatus, mMOFerr);

            if (mMOP2fptsThis.rows() > 0)
            {

                List<Point> cornersThis = mMOP2fptsThis.toList();
                List<byte> byteStatus = mMOBStatus.toList();
                //코너 네점은 항상 cornersThis[0,1,2,3] 에 들어있음
                //코너 네점도 optical Flow로 계속 변하므로, 코너 네 점에 대해 마커 길이를 새로 구해야함
                //왼위, 왼다와 오위, 오다의 길이차이(높이)가 달라서 
                //둘다 구한 후 평균을 구해야함 (가로도 마찬가지)
                markerHeight1 = (int)Get_dist_Point(cornersThis[0], cornersThis[1]);

                markerHeight2 = (int)Get_dist_Point(cornersThis[2], cornersThis[3]);
                markerWidth1 = (int)Get_dist_Point(cornersThis[0], cornersThis[3]);
                markerWidth2 = (int)Get_dist_Point(cornersThis[1], cornersThis[2]);
                markerHeight = (markerHeight1 + markerHeight2) / 2;
                markerWidth = (markerWidth1 + markerWidth2) / 2;
                oneWidth = (int)Mathf.Sqrt(markerWidth) / 8;
                oneHeight = (int)Mathf.Sqrt(markerHeight) / 8;

                int x = 0;
                int y = mMOP2fptsThis.rows();

                for (x = 0; x < y; x++)
                {
                    //goodfeature로 나온 점들의 3D points 맵핑을 위해 인덱스를 구하는 과정
                    //이건 제대로 구해지므로 건들일 필요 X

                    //print("onewidth:: " + oneWidth + "oneheight::: " + oneHeight);

                    double i = cornersThis[x].x;
                    double j = cornersThis[x].y;
                    double ii = cornersThis[x].x - cornersThis[0].x - oneWidth;
                    double jj = cornersThis[x].y - cornersThis[0].y - oneHeight;

                    //print("ii:: " + ii + " jj::: " + jj);

                    double q_x = ii / oneWidth;
                    double q_y = jj / oneHeight;

                    double r_x = ii % oneWidth;
                    double r_y = jj % oneHeight;

                    if (r_x > oneWidth / 2)
                    {
                        q_x++;
                    }

                    if (r_y > oneHeight / 2)
                    {
                        q_y++;
                    }

                    index_x = (int)q_x;
                    index_y = (int)q_y;

                    p2d[0, x] = new Point(i, j);
                    xArr[x] = index_y + 1;
                    yArr[x] = index_x + 1;

            
                }
                //왼업 인덱스
                xArr[0] = 0;
                yArr[0] = 0;
                //왼다 인덱스
                xArr[1] = 8;
                yArr[1] = 0;
                //오다 인덱스
                xArr[2] = 8;
                yArr[2] = 8;
                //오업 인덱스
                xArr[3] = 0;
                yArr[3] = 8;

                for (int i = 0; i < mMOP2fptsThis.rows(); i++)
                {
                    //print("index: " + xArr[i] + " " + yArr[i]);

                    if (xArr[i] < 0 || xArr[i] > 8 || yArr[i] < 0 || yArr[i] > 8)
                    {
                        isDetect = true;
                    }
                    else
                    {
                        for (int j = 0; j < mMOP2fptsThis.rows(); j++)
                        {
                            //solvePnP에 쓸 3D points를 넣음
                            solve_p3d[0, i] = p3d[xArr[i], yArr[i]];
                        }
                    }
                }
            }

            if (applyEstimationPose)
            {
                if (mMOP2fptsThis.rows() == 4)
                {   //good feature 없는 경우
                    p2d1 = new Point(p2d[0, 0].x, p2d[0, 0].y);
                    p2d2 = new Point(p2d[0, 1].x, p2d[0, 1].y);
                    p2d3 = new Point(p2d[0, 2].x, p2d[0, 2].y);
                    p2d4 = new Point(p2d[0, 3].x, p2d[0, 3].y);

                    obj2Points = new MatOfPoint2f(p2d1, p2d2, p2d3, p2d4);
                    obj3Points = new MatOfPoint3f(L_U1_0, L_D4_0, R_D3_0, R_U2_0);
                }
                if (mMOP2fptsThis.rows() == 5)
                {   //good feature 1개인 경우
                    p2d1 = new Point(p2d[0, 0].x, p2d[0, 0].y);
                    p2d2 = new Point(p2d[0, 1].x, p2d[0, 1].y);
                    p2d3 = new Point(p2d[0, 2].x, p2d[0, 2].y);
                    p2d4 = new Point(p2d[0, 3].x, p2d[0, 3].y);
                    p2d5 = new Point(p2d[0, 4].x, p2d[0, 4].y);

                    p3d5 = new Point3(solve_p3d[0, 4].x, solve_p3d[0, 4].y, 0);

                    obj2Points = new MatOfPoint2f(p2d1, p2d2, p2d3, p2d4, p2d5);
                    obj3Points = new MatOfPoint3f(L_U1_0, L_D4_0, R_D3_0, R_U2_0, p3d5);
                }
                else if (mMOP2fptsThis.rows() == 6)
                {   //good feature 2개인 경우
                    p2d1 = new Point(p2d[0, 0].x, p2d[0, 0].y);
                    p2d2 = new Point(p2d[0, 1].x, p2d[0, 1].y);
                    p2d3 = new Point(p2d[0, 2].x, p2d[0, 2].y);
                    p2d4 = new Point(p2d[0, 3].x, p2d[0, 3].y);
                    p2d5 = new Point(p2d[0, 4].x, p2d[0, 4].y);
                    p2d6 = new Point(p2d[0, 5].x, p2d[0, 5].y);

                    p3d5 = new Point3(solve_p3d[0, 4].x, solve_p3d[0, 4].y, 0);
                    p3d6 = new Point3(solve_p3d[0, 5].x, solve_p3d[0, 5].y, 0);

                    obj2Points = new MatOfPoint2f(p2d1, p2d2, p2d3, p2d4, p2d5, p2d6);
                    obj3Points = new MatOfPoint3f(L_U1_0, L_D4_0, R_D3_0, R_U2_0, p3d5, p3d6);
                }
            }

            // 이전 detect 현재 detect 거리 차이
            if (obj2Points != null)
            {
                distance = Dist_DvsT(obj2Points, obj2Points_D);
                // print("distance ::: " + distance);
            }

            if (applyEstimationPose && ids.total() > 0)
            {
                //  what is this distance unit 
                if (distance <= 200)
                {
                    print("Tracking");
                    Calib3d.solvePnP(obj3Points, obj2Points, camMatrix, distCoeffs, rvecs, tvecs);
                }
                else
                {
                    print("Detecting");
                    Calib3d.solvePnP(obj3Points, obj2Points_D, camMatrix, distCoeffs, rvecs, tvecs);
                    isDetect = true;
                }
            }

            if (applyEstimationPose && ids.total() > 0 && !isDetect)
            {
                for (int i = 0; i < ids.total(); i++)
                {
                    //This example can display ARObject on only first detected marker.
                    if (i == 0)
                    {
                        // Convert to unity pose data.
                        double[] rvecArr = new double[3];
                        rvecs.get(0, 0, rvecArr);
                        double[] tvecArr = new double[3];
                        tvecs.get(0, 0, tvecArr);
                        tvecArr[2] /= imageOptimizationHelper.downscaleRatio;
                        PoseData poseData = ARUtils.ConvertRvecTvecToPoseData(rvecArr, tvecArr);

                        // Changes in pos/rot below these thresholds are ignored.
                        if (enableLowPassFilter)
                        {
                            ARUtils.LowpassPoseData(ref oldPoseData, ref poseData, positionLowPass, rotationLowPass);
                        }
                        oldPoseData = poseData;

                        // Create transform matrix.
                        transformationM = Matrix4x4.TRS(poseData.pos, poseData.rot, Vector3.one);

                        lock (sync)
                        {
                            // Right-handed coordinates system (OpenCV) to left-handed one (Unity)
                            ARM = invertYM * transformationM;

                            // Apply Z-axis inverted matrix.
                            ARM = ARM * invertZM;
                        }

                        hasUpdatedARTransformMatrix = true;

                        break;
                    }
                }
            }
        }

        private void OnDetectionDone()
        {

            if (applyEstimationPose)
            {
                if (hasUpdatedARTransformMatrix)
                {
                    hasUpdatedARTransformMatrix = false;

                    // Apply the cameraToWorld matrix with the Z-axis inverted.         
                    ARM = arCamera.cameraToWorldMatrix * invertZM * ARM;

                    ARUtils.SetTransformFromMatrix(arGameObject.transform, ref ARM);
                }
            }

            isDetecting = false;
        }
#endif

        /// <summary>
        /// Raises the destroy event.
        /// </summary>
        void OnDestroy()
        {
            imageOptimizationHelper.Dispose();
#if NETFX_CORE && !DISABLE_HOLOLENSCAMSTREAM_API
            webCamTextureToMatHelper.frameMatAcquired -= OnFrameMatAcquired;
#endif
            webCamTextureToMatHelper.Dispose();
        }

        /// <summary>
        /// Raises the back button click event.
        /// </summary>
        public void OnBackButtonClick()
        {
#if UNITY_5_3 || UNITY_5_3_OR_NEWER
            SceneManager.LoadScene("HoloLensWithOpenCVForUnityExample");
#else
            Application.LoadLevel("HoloLensWithOpenCVForUnityExample");
#endif
        }

        /// <summary>
        /// Raises the play button click event.
        /// </summary>
        public void OnPlayButtonClick()
        {
            webCamTextureToMatHelper.Play();
        }

        /// <summary>
        /// Raises the pause button click event.
        /// </summary>
        public void OnPauseButtonClick()
        {
            webCamTextureToMatHelper.Pause();
        }

        /// <summary>
        /// Raises the stop button click event.
        /// </summary>
        public void OnStopButtonClick()
        {
            webCamTextureToMatHelper.Stop();
        }

        /// <summary>
        /// Raises the change camera button click event.
        /// </summary>
        public void OnChangeCameraButtonClick()
        {
            webCamTextureToMatHelper.requestedIsFrontFacing = !webCamTextureToMatHelper.IsFrontFacing();
        }

        /// <summary>
        /// Raises the use stored camera parameters toggle value changed event.
        /// </summary>
        public void OnUseStoredCameraParametersToggleValueChanged()
        {
            if (useStoredCameraParametersToggle.isOn)
            {
                useStoredCameraParameters = true;
            }
            else
            {
                useStoredCameraParameters = false;
            }

            if (webCamTextureToMatHelper != null && webCamTextureToMatHelper.IsInitialized())
            {
                webCamTextureToMatHelper.Initialize();
            }
        }

        /// <summary>
        /// Raises the enable low pass filter toggle value changed event.
        /// </summary>
        public void OnEnableLowPassFilterToggleValueChanged()
        {
            if (enableLowPassFilterToggle.isOn)
            {
                enableLowPassFilter = true;
            }
            else
            {
                enableLowPassFilter = false;
            }
        }

        /// <summary>
        /// Raises the tapped event.
        /// </summary>
        public void OnTapped()
        {
            if (EventSystem.current.IsPointerOverGameObject())
                return;

            arCube.GetComponent<MeshRenderer>().material.color = Color.red;
            
        }


        public void OnFrameMatAcquired(Mat bgraMat, Matrix4x4 projectionMatrix, Matrix4x4 cameraToWorldMatrix)
        {
            downScaleFrameMat = imageOptimizationHelper.GetDownScaleMat(bgraMat);

            if (enableDetection)
            {

                Imgproc.cvtColor(downScaleFrameMat, grayMat, Imgproc.COLOR_BGRA2GRAY);

                // Detect markers and estimate Pose
                Aruco.detectMarkers(grayMat, dictionary, corners, ids, detectorParams, rejectedCorners, camMatrix, distCoeffs);

                if (applyEstimationPose && ids.total() > 0)
                {
                    Aruco.estimatePoseSingleMarkers(corners, markerLength, camMatrix, distCoeffs, rvecs, tvecs);

                    for (int i = 0; i < ids.total(); i++)
                    {

                        //This example can display ARObject on only first detected marker.
                        if (i == 0)
                        {

                            // Convert to unity pose data.
                            double[] rvecArr = new double[3];
                            rvecs.get(0, 0, rvecArr);
                            double[] tvecArr = new double[3];
                            tvecs.get(0, 0, tvecArr);
                            tvecArr[2] /= imageOptimizationHelper.downscaleRatio;
                            PoseData poseData = ARUtils.ConvertRvecTvecToPoseData(rvecArr, tvecArr);

                            // Changes in pos/rot below these thresholds are ignored.
                            if (enableLowPassFilter)
                            {
                                ARUtils.LowpassPoseData(ref oldPoseData, ref poseData, positionLowPass, rotationLowPass);
                            }
                            oldPoseData = poseData;

                            // Create transform matrix.
                            transformationM = Matrix4x4.TRS(poseData.pos, poseData.rot, Vector3.one);

                            lock (sync)
                            {
                                // Right-handed coordinates system (OpenCV) to left-handed one (Unity)
                                ARM = invertYM * transformationM;

                                // Apply Z-axis inverted matrix.
                                ARM = ARM * invertZM;
                            }

                            hasUpdatedARTransformMatrix = true;

                            break;
                        }
                    }
                }
            }

            Mat rgbMat4preview = null;
         
        }
    }
}