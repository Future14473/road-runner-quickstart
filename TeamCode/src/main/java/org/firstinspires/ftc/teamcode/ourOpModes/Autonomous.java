// old autonomous op mode
//package org.firstinspires.ftc.teamcode.ourOpModes;
//
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
//import org.firstinspires.ftc.teamcode.drive.TwoWheelTrackingLocalizer;
//import org.firstinspires.ftc.teamcode.ourMovementLib.Follower;
//import org.firstinspires.ftc.teamcode.ourMovementLib.paths.DaPath;
//import org.firstinspires.ftc.teamcode.ourOpModes.resources.Timing;
//import org.firstinspires.ftc.teamcode.ourOpModes.robotParts.Mecanum;
//
//@TeleOp(name="Auto", group="Teleop")
//public class Autonomous extends LinearOpMode {
//
//    Timing timer = new Timing(Autonomous.this);
//    Follower follower;
//    TwoWheelTrackingLocalizer odometry;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        odometry = new TwoWheelTrackingLocalizer(hardwareMap, new SampleMecanumDrive(hardwareMap));
//        follower = new Follower(
//                new Mecanum(hardwareMap),
//                odometry,
//                Autonomous.this,
//                telemetry);
//
//        waitForStart();
//
//
//        while (opModeIsActive()){
//            odometry.update();
//            telemetry.addData("Current Position X", odometry.getPoseEstimate().getY());
//            telemetry.addData("Current Position Y", odometry.getPoseEstimate().getX());
//            telemetry.addData("Current Position R", odometry.getPoseEstimate().getHeading());
//
//            telemetry.update();
//            if (gamepad1.x){
//                follower.debugGoTo(DaPath.forward);
//            }
//            if (gamepad1.y){
//                follower.goTo(DaPath.strafe);
//            }
//            if (gamepad1.a){
//                follower.goTo(DaPath.turn);
//            }
//            if(gamepad1.b){
//                follower.goTo(DaPath.origin);
//            }
//        }
//    }
//
//
//}

package org.firstinspires.ftc.teamcode.ourOpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.cv.Detection;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.TwoWheelTrackingLocalizer;
import org.firstinspires.ftc.teamcode.ourMovementLib.Follower;
import org.firstinspires.ftc.teamcode.ourOpModes.resources.IMU;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(group = "drive")
public class Autonomous extends LinearOpMode {
    @Override
    public void runOpMode() {
        VuforiaPhone vuforia = new VuforiaPhone(hardwareMap, telemetry);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // TwoWheelTrackingLocalizer myTwoWheelLoc = new TwoWheelTrackingLocalizer(hardwareMap, drive);

        Follower follower = new Follower(drive, vuforia, this, telemetry, gamepad1);

        IMU imu = new IMU(hardwareMap, telemetry);

        //Setting up CV Camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().
                getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvCamera webcam = OpenCvCameraFactory.getInstance().
                createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        Detection detector = new Detection(telemetry);

        webcam.setPipeline(detector);

        //Beginning Detection during INIT phase
        webcam.openCameraDeviceAsync(() -> {
            webcam.startStreaming(352, 288, OpenCvCameraRotation.UPRIGHT);
            
        telemetry.addData("Autonomous", "Hold A for manual control");
        telemetry.update();


        /*
         * Start robot like this. Make sure heading is aligned.
         *        WALL
         * -----------------------
         *      |           |
         *Tape->|   HERE    | <- Tape
         *      |           |
         */

        waitForStart();
        webcam.stopStreaming();

        // GRAB WOBBLE

        // Start Vuforia
        vuforia.beginTracking();


        // A BLOCK
        follower.DRIVE_MAINTAIN_HEADING(0.4, 0, 0, 3000, imu);
        follower.goTo(-4, 22, -0.31); // Goto powershot or high goal spot. IDK see which one more reliable
        shoot();
        follower.goTo(3, 39.8, 0); // vumark lock on position
        follower.goTo(20.2, 44.3, 0.26);
        // PLACE WOBBLE

        // B BLOCK
        follower.DRIVE_MAINTAIN_HEADING(0.4, 0, 0, 3000, imu);
        follower.goTo(-4, 22, -0.31); // Goto powershot or high goal spot. IDK see which one more reliable
        shoot();
        follower.goTo(3, 39.8, 0); // vumark lock on position
        follower.goTo(40.8, 24.0, 0.34);
        // PLACE WOBBLE

        // C box
        follower.DRIVE_MAINTAIN_HEADING(0.4, 0, 0, 3000, imu);
        follower.goTo(-4, 22, -0.31); // Goto powershot or high goal spot. IDK see which one more reliable
        shoot();
        follower.goTo(3, 39.8, 0); // vumark lock on position
        follower.DRIVE_MAINTAIN_HEADING(0.4, 0, 0, 1700, imu);
        // PLACE WOBBLE

        // Finally, park, or something

        // Powershot location
        // follower.goTo(-4, 22, -0.31);

        // high goal location
        // follower.goTo(4.5, 28.5, -0.07);
    }

    void shoot(){
        // spin shooter up
        // wait 3 secs
        // spin taco and transfer
        // wait 3 secs for ring movement
        // wait another 3 secs for shoot
    }

        //TODO check if these ring counts really correspond to the first second and third squares
//        follower.goTo(12.3,42.5,0); //0 ring square
//        follower.goTo(35, 25, 0); //1 ring square
//        follower.goTo(55, 48, 0); //4 ring square


        // This doesn't work because the configuration for turning is bad
        // Nothing wrong with my code here
        // follower.goTo(0,0,Math.PI/2);

        //Timing.delay(1000);
        //follower.austinStop();

        /*while (!isStopRequested()) {
            telemetry.addData("Follower Position", follower.getPositionOdoTest().toString());
//            follower.debugGoTo(new PathPoint(0,10,0));


            drive.update();
            myTwoWheelLoc.update();

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("x Parallel Encoder", myTwoWheelLoc.parallelEncoder.getCorrectedVelocity());
            telemetry.addData("y Perpendicular Encoder", myTwoWheelLoc.perpendicularEncoder.getCorrectedVelocity());
            telemetry.update();

        }

         */

}
