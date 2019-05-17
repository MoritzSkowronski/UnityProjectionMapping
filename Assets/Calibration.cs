using UnityEngine;
using System.Collections;
using OpenCVForUnity.CoreModule;
using OpenCVForUnity.Calib3dModule;
using System.Collections.Generic;
using System.Text;
using System.IO;
using Assets;

public class Calibration
{
    // NB some code adapted from OpenCVSharp camera calibration example:
    // https://github.com/shimat/opencvsharp/blob/master/sample/CStyleSamplesCS/Samples/CalibrateCamera.cs

    private Camera _mainCamera;
    private static int _numImages = 1;
    private double height;
    private double width;

    public Calibration (Camera mainCamera)
    {
        _mainCamera = mainCamera;
        height = (double)Screen.height;
        width = (double)Screen.width;
    }

    public double calibrateFromCorrespondences (List<Vector3> imagePoints, List<Vector3> worldPoints)
    {
        List<Mat> imagePointsCvMat = new List<Mat> ();
        imagePointsCvMat.Add (PutImagePointsIntoCVMat (imagePoints));
        List<Mat> worldPointsCvMat = new List<Mat> ();
        worldPointsCvMat.Add (PutObjectPointsIntoCVMat (worldPoints));

        Size size = new Size (width, height);
        Mat intrinsic = CreateIntrinsicGuess (height, width);
        MatOfDouble distortion = new MatOfDouble (0, 0, 0, 0);
        List<Mat> rotation = new List<Mat> ();
        List<Mat> translation = new List<Mat> ();
        int flags = 6377; //Calib3d.CALIB_ZERO_TANGENT_DIST | Calib3d.CALIB_USE_INTRINSIC_GUESS | Calib3d.CALIB_FIX_K1 | Calib3d.CALIB_FIX_K2 | Calib3d.CALIB_FIX_K3 | Calib3d.CALIB_FIX_K4 | Calib3d.CALIB_FIX_K5;

//        Debug.Log ("imagePoint2fMat: " + imagePointsCvMat [0].dump ());
//        Debug.Log ("worldPoints3fMat: " + worldPointsCvMat [0].dump ());
//        Debug.Log ("size: " + size);
//        Debug.Log ("intrinsic: " + intrinsic.dump ());
//        Debug.Log ("distortion: " + distortion.dump ());
//        Debug.Log ("flags: " + flags);

        double repErr = -1;
        repErr = Calib3d.calibrateCamera (worldPointsCvMat, imagePointsCvMat, size, intrinsic, distortion, rotation, translation, flags);
        ApplyCalibrationToUnityCamera (intrinsic, rotation [0], translation [0]);

//        Debug.Log ("intrinsic: " + intrinsic.dump ());
//        Debug.Log ("rotation: " + rotation [0].dump ());
//        Debug.Log ("translation: " + translation [0].dump ());

        return repErr;
    }

    private void ApplyCalibrationToUnityCamera (Mat intrinsic, Mat rotation, Mat translation)
    {
        Mat rotationInverse = GetRotationMatrixFromRotationVector (rotation).t (); // transpose is same as inverse for rotation matrix
        Mat transFinal = (rotationInverse * -1) * translation;
        _mainCamera.projectionMatrix = LoadProjectionMatrix ((float)intrinsic.get (0, 0) [0], (float)intrinsic.get (1, 1) [0], (float)intrinsic.get (0, 2) [0], (float)intrinsic.get (1, 2) [0]);
        ApplyTranslationAndRotationToCamera (transFinal, RotationConversion.RotationMatrixToEulerZXY (rotationInverse));
    }

    private static Mat CreatePointCountMatrix (int numPoints)
    {
        int[] pointCountsValue = new int[_numImages];
        pointCountsValue [0] = numPoints;
        Mat pointCounts = new Mat (_numImages, 1, CvType.CV_32SC1);
        pointCounts.put (0, 0, pointCountsValue);

        return pointCounts;
    }

    private static Mat GetRotationMatrixFromRotationVector (Mat rotation_)
    {
        Mat rotationFull = new Mat (3, 3, CvType.CV_64FC1);
        // get full rotation matrix from rotation vector
        Calib3d.Rodrigues (rotation_, rotationFull); 
        double[] LHSflipBackMatrix = new double[] { 1.0, 0.0, 0.0, 
            0.0, 1.0, 0.0, 
            0.0, 0.0, -1.0
        };
        Mat LHSflipBackMatrixM = new Mat (3, 3, CvType.CV_64FC1);
        LHSflipBackMatrixM.put (0, 0, LHSflipBackMatrix);
        Mat rotLHS = rotationFull * LHSflipBackMatrixM; // invert Z (as we did before when savings points) to get from RHS -> LHS
        return rotLHS;
    }

    private Matrix4x4 LoadProjectionMatrix (float fx, float fy, float cx, float cy)
    {
        // https://github.com/kylemcdonald/ofxCv/blob/88620c51198fc3992fdfb5c0404c37da5855e1e1/libs/ofxCv/src/Calibration.cpp
        float w = _mainCamera.pixelWidth;
        float h = _mainCamera.pixelHeight;
        float nearDist = _mainCamera.nearClipPlane;
        float farDist = _mainCamera.farClipPlane;

        return MakeFrustumMatrix (
            nearDist * (-cx) / fx, nearDist * (w - cx) / fx,
            nearDist * (cy) / fy, nearDist * (cy - h) / fy,
            nearDist, farDist);
    }

    private Matrix4x4 MakeFrustumMatrix (float left, float right,
                                         float bottom, float top,
                                         float zNear, float zFar)
    {
        // https://github.com/openframeworks/openFrameworks/blob/master/libs/openFrameworks/math/ofMatrix4x4.cpp
        // note transpose of ofMatrix4x4 wr.t OpenGL documentation, since the OSG use post multiplication rather than pre.
        // NB this has been transposed here from the original openframeworks code

        float A = (right + left) / (right - left);
        float B = (top + bottom) / (top - bottom);
        float C = -(zFar + zNear) / (zFar - zNear);
        float D = -2.0f * zFar * zNear / (zFar - zNear);

        var persp = new Matrix4x4 ();
        persp [0, 0] = 2.0f * zNear / (right - left);
        persp [1, 1] = 2.0f * zNear / (top - bottom);
        persp [2, 0] = A;
        persp [2, 1] = B;
        persp [2, 2] = C;
        persp [2, 3] = -1.0f;
        persp [3, 2] = D;

        var rhsToLhs = new Matrix4x4 ();
        rhsToLhs [0, 0] = 1.0f;
        rhsToLhs [1, 1] = -1.0f; // Flip Y (RHS -> LHS)
        rhsToLhs [2, 2] = 1.0f;
        rhsToLhs [3, 3] = 1.0f;

        return rhsToLhs * persp.transpose; // see comment above
    }

    private void ApplyTranslationAndRotationToCamera (Mat translation, Rotation r)
    {
        double tx = translation.get (0, 0) [0];
        double ty = translation.get (1, 0) [0];
        double tz = translation.get (2, 0) [0];

        _mainCamera.transform.position = new Vector3 ((float)tx, (float)ty, (float)tz);
        _mainCamera.transform.eulerAngles = new Vector3 ((float)r.X, (float)r.Y, (float)r.Z);
    }

    private Mat CreateIntrinsicGuess (double height, double width)
    {
        // from https://docs.google.com/spreadsheet/ccc?key=0AuC4NW61c3-cdDFhb1JxWUFIVWpEdXhabFNjdDJLZXc#gid=0
        // taken from http://www.neilmendoza.com/projector-field-view-calculator/
        float hfov = 91.2705674249382f;
        float vfov = 59.8076333281726f;

        double fx = (double)((float)width / (2.0f * Mathf.Tan (0.5f * hfov * Mathf.Deg2Rad)));
        double fy = (double)((float)height / (2.0f * Mathf.Tan (0.5f * vfov * Mathf.Deg2Rad)));

        double cy = height / 2.0;
        double cx = width / 2.0;

        double[] intrGuess = new double[] { fx, 0.0, cx, 
            0.0, fy, cy, 
            0.0, 0.0, 1.0
        };

        Mat m = new Mat (3, 3, CvType.CV_64FC1);
        m.put (0, 0, intrGuess);

        return m;
    }

    private MatOfPoint3f PutObjectPointsIntoCVMat (List<Vector3> _objectPositions)
    {
        Point3[] worldPointsArray = new Point3[_objectPositions.Count];
        for (int i = 0; i < _objectPositions.Count; i++) {
            worldPointsArray [i] = new Point3 (_objectPositions [i].x, _objectPositions [i].y, _objectPositions [i].z * -1);
        }
        return new MatOfPoint3f (worldPointsArray);
    }

    private MatOfPoint2f PutImagePointsIntoCVMat (List<Vector3> _imagePositions)
    {
        Point[] imagePointsArray = new Point[_imagePositions.Count];
        for (int i = 0; i < _imagePositions.Count; i++) {
            imagePointsArray [i] = new Point (_imagePositions [i].x, _imagePositions [i].y);
        }
        return new MatOfPoint2f (imagePointsArray);
    }
}