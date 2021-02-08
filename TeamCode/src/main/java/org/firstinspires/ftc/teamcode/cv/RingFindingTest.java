package org.firstinspires.ftc.teamcode.cv;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.firstinspires.ftc.teamcode.ourOpModes.resources.IMU;
import org.firstinspires.ftc.teamcode.ourOpModes.resources.RotationUtil;
import org.firstinspires.ftc.teamcode.ourOpModes.robotParts.Mecanum;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "RingFindingTest", group = "Auto")
public class RingFindingTest extends LinearOpMode {
    Mecanum MecanumDrive;
    IMU imu;

    Detection detector;
    int cameraMonitorViewId;
    OpenCvCamera webcam;

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive = new Mecanum(hardwareMap);
        imu = new IMU(hardwareMap, telemetry);


        //Setting up Camera
        cameraMonitorViewId = hardwareMap.appContext.getResources().
                getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().
                createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        detector = new Detection(telemetry);


        webcam.setPipeline(detector);

        //Opening and Streaming from Camera

        webcam.openCameraDeviceAsync(() -> {
            webcam.startStreaming(352, 288, OpenCvCameraRotation.UPRIGHT);
        });


        waitForStart();

        while(opModeIsActive()){
            if(!gamepad1.a && detector.angle > 0.1) {

                double turnPwr = RotationUtil.turnLeftOrRight(0, detector.angle, Math.PI * 2);
                MecanumDrive.drive(0, 0,
                        (Math.abs(turnPwr) > 0.08) ? 2 * turnPwr : 0);
            }
        }

        webcam.stopStreaming();
    }


}