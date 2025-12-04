package org.firstinspires.ftc.teamcode.apriltagdetection;

import android.util.Pair;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.opencv.core.Mat;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class CameraDetector {

    private final OpenCvCamera controlHubCam;
    private final AprilTagDetectionPipeline aprilTagDetectionPipeline;

    private List<Integer> requiredTags;

    // TODO: change this to match your camera resolution
    private static final int cameraWidth = 640;
    private static final int cameraHeight = 480;

    // TODO: modify this value if necessary (default is 1)
    private static final double distanceMultiplier = 1.0;

    private String data;
    private AprilTagDetection detection;

    /**
     * Creates a detector for the specified AprilTags.
     *
     * @param aprilTagDetectionPipeline the OpenCvPipeline used
     * @param hardwareMap               the HardwareMap which contains the camera
     * @param requiredTags              one or more tags to be searched
     */
    public CameraDetector(AprilTagDetectionPipeline aprilTagDetectionPipeline,HardwareMap hardwareMap, Integer... requiredTags) {
        this.aprilTagDetectionPipeline = aprilTagDetectionPipeline;
        this.requiredTags = new ArrayList<>(Arrays.asList(requiredTags));

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        //TODO: change "Webcam 1" to match the name in your configuration
        controlHubCam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        controlHubCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                controlHubCam.startStreaming(cameraWidth, cameraHeight, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });

        controlHubCam.setPipeline(aprilTagDetectionPipeline);
    }

    /**
     * Change the tags to be searched.
     * @param tags one or more tag ids
     */
    public void changeTags(Integer... tags) {
        this.requiredTags = new ArrayList<>(Arrays.asList(tags));
    }

    /**
     * Adds the data about the tags to the given telemetry.
     *
     * @param telemetry the telemetry in which the data will be added
     */
    public void displayData(Telemetry telemetry) {
        telemetry.addLine(data);
    }

    /**
     * Returns the position of the camera relative to the center of the AprilTag.
     *
     * @param distanceUnit the distance measuring unit
     * @param angleUnit    the angle measuring unit
     * @return the position in a 3D space
     * @see Pose3D
     */
    public Pose3D getPose3D(DistanceUnit distanceUnit, AngleUnit angleUnit) {

        Pair<Mat, Mat> vectors = aprilTagDetectionPipeline.getImgVectors(detection);

        if(vectors == null)
            return null;

        Mat RMat = vectors.first;
        Mat TMat = vectors.second;

        double[] coordinates = {TMat.get(0, 0)[0], TMat.get(2, 0)[0], TMat.get(1, 0)[0]};
        double[] angles = {RMat.get(0, 0)[0], RMat.get(2, 0)[0], RMat.get(1, 0)[0]};

        switch (distanceUnit) {
            case CM:
                for (int i = 0; i < coordinates.length; i++)
                    coordinates[i] *= 100 * distanceMultiplier;
                break;
            case MM:
                for (int i = 0; i < coordinates.length; i++)
                    coordinates[i] *= 1000 * distanceMultiplier;
                break;
            case INCH:
                for (int i = 0; i < coordinates.length; i++)
                    coordinates[i] *= (100 / 2.54) * distanceMultiplier;
                break;
            case METER:
                break;
        }

        switch (angleUnit) {
            case DEGREES:
                for (int i = 0; i < angles.length; i++)
                    angles[i] = Math.toDegrees(angles[i]);
                break;
            case RADIANS:
                break;
        }

        return new Pose3D(coordinates[0], coordinates[1], coordinates[2], angles[0], angles[1], angles[2]);
    }

    /**
     * Scans for the required AprilTags.
     * Has to be called in a loop.
     */
    public void scan() {

        ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

        if (currentDetections.isEmpty()) {
            data = "No tag detected!";
            return;
        }

        boolean tagFound = false;
        try {
            for (AprilTagDetection detection : currentDetections)
                if (requiredTags.contains(detection.id)) {
                    this.detection = detection;
                    tagFound = true;
                    break;
                }
        } catch (Exception ignored) {
        }

        if (tagFound)
            data = "Tag detected!";
        else
            data = "Required tag not found!";
    }

    /**
     * Returns the id of the last found tag.
     *
     * @return the id of the AprilTag or -1 if no match was found
     */
    public int getLastFoundId() {
        if (detection == null)
            return -1;
        return detection.id;
    }

}
