package org.firstinspires.ftc.teamcode.apriltagdetection.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.apriltagdetection.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.apriltagdetection.CameraDetector;
import org.firstinspires.ftc.teamcode.apriltagdetection.Pose3D;
import org.firstinspires.ftc.teamcode.apriltagdetection.intrinsics.CameraIntrinsics;

@TeleOp
public class ExampleTeleop extends LinearOpMode {

    @Override
    public void runOpMode(){

        CameraDetector detector = new CameraDetector(new AprilTagDetectionPipeline(new CameraIntrinsics()),hardwareMap,20,21,22,23);

        waitForStart();

        while (opModeIsActive()) {
            detector.scan();
            detector.displayData(telemetry);
            telemetry.addData("tag",detector.getLastFoundId());
            Pose3D pose = detector.getPose3D(DistanceUnit.CM, AngleUnit.DEGREES);

            if(pose != null)
            {
                telemetry.addData("x",pose.x);
                telemetry.addData("y",pose.y);
                telemetry.addData("z",pose.z);
                telemetry.addData("pitch",pose.pitch);
                telemetry.addData("roll",pose.roll);
                telemetry.addData("yaw",pose.yaw);
            }

            telemetry.update();

        }
    }
}
