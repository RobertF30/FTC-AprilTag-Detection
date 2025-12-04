package org.firstinspires.ftc.teamcode.apriltagdetection;

import android.util.Pair;
import org.firstinspires.ftc.teamcode.apriltagdetection.intrinsics.CameraIntrinsics;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point3;
import org.opencv.imgproc.Imgproc;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.apriltag.AprilTagDetectorJNI;
import org.openftc.easyopencv.OpenCvPipeline;
import java.util.ArrayList;

public class AprilTagDetectionPipeline extends OpenCvPipeline {
    private long apriltagRef;
    private final Mat grey = new Mat();
    private ArrayList<AprilTagDetection> detections = new ArrayList<>();

    private Mat cameraMatrix;

    private final double fx;
    private final double fy;
    private final double cx;
    private final double cy;

    //TODO: change this to match the size of your april tag (the size is in METERS)
    private final double tagsize = 0.2;
    private final double tagsizeX;
    private final double tagsizeY;


    /**
     * Creates a new Pipeline.
     * @param cameraIntrinsics the class which contains the camera intrinsics parameters
     */
    public AprilTagDetectionPipeline(CameraIntrinsics cameraIntrinsics)
    {
        this.fx = cameraIntrinsics.fx;
        this.fy = cameraIntrinsics.fy;
        this.cx = cameraIntrinsics.cx;
        this.cy = cameraIntrinsics.cy;

        this.tagsizeX = tagsize;
        this.tagsizeY = tagsize;

        initMatrix();

        apriltagRef = AprilTagDetectorJNI.createApriltagDetector(AprilTagDetectorJNI.TagFamily.TAG_36h11.string, 3, 3);
    }

    /**
     * Creates a new Pipeline.
     * @param fx x coordinate of focal point
     * @param fy y coordinate of focal point
     * @param cx x coordinate of center point
     * @param cy y coordinate of center point
     */
    public AprilTagDetectionPipeline( double fx, double fy, double cx, double cy) {
        this.tagsizeX = tagsize;
        this.tagsizeY = tagsize;
        this.fx = fx;
        this.fy = fy;
        this.cx = cx;
        this.cy = cy;

        initMatrix();

        apriltagRef = AprilTagDetectorJNI.createApriltagDetector(AprilTagDetectorJNI.TagFamily.TAG_36h11.string, 3, 3);
    }

    @Override
    protected void finalize() {
        if (apriltagRef != 0) {
            AprilTagDetectorJNI.releaseApriltagDetector(apriltagRef);
            apriltagRef = 0;
        }
    }


    @Override
    public Mat processFrame(Mat input) {

        Imgproc.cvtColor(input, grey, Imgproc.COLOR_RGBA2GRAY);
        detections = AprilTagDetectorJNI.runAprilTagDetectorSimple(apriltagRef, grey, tagsize, fx, fy, cx, cy);
        return input;
    }

    /**
     * Returns a list of all tags currently in sight.
     * @return a list of AprilTagDetections
     */
    public ArrayList<AprilTagDetection> getLatestDetections() {
        return detections;
    }

    /**
     * Creates a matrix containing the camera intrinsics.
     * <p>
     * fx 0 cx
     * </p><p>
     * 0 fy cy
     * </p><p>
     * 0  0  1
     * </p>
     */
    private void initMatrix() {
        cameraMatrix = new Mat(3, 3, CvType.CV_32FC1);

        cameraMatrix.put(0, 0, fx);
        cameraMatrix.put(0, 1, 0);
        cameraMatrix.put(0, 2, cx);

        cameraMatrix.put(1, 0, 0);
        cameraMatrix.put(1, 1, fy);
        cameraMatrix.put(1, 2, cy);

        cameraMatrix.put(2, 0, 0);
        cameraMatrix.put(2, 1, 0);
        cameraMatrix.put(2, 2, 1);
    }

    /**
     * Returns the rotation and translation vectors.
     * @param detection the aprilTag
     * @return a pair with the rotation vector on the first position and translation vector on the second
     */
    public Pair<Mat,Mat> getImgVectors(AprilTagDetection detection){

        if(detection == null)
            return null;

        MatOfPoint2f points2d = new MatOfPoint2f(detection.corners);


        Point3[] arrayPoints3d = new Point3[4];
        arrayPoints3d[0] = new Point3(-tagsizeX / 2, tagsizeY / 2, 0);
        arrayPoints3d[1] = new Point3(tagsizeX / 2, tagsizeY / 2, 0);
        arrayPoints3d[2] = new Point3(tagsizeX / 2, -tagsizeY / 2, 0);
        arrayPoints3d[3] = new Point3(-tagsizeX / 2, -tagsizeY / 2, 0);
        MatOfPoint3f points3d = new MatOfPoint3f(arrayPoints3d);


        Mat rotationVector = new Mat();
        Mat translationVector = new Mat();
        Calib3d.solvePnP(points3d, points2d, cameraMatrix, new MatOfDouble(), rotationVector, translationVector, false);

        return new Pair<>(rotationVector,translationVector);
    }
}