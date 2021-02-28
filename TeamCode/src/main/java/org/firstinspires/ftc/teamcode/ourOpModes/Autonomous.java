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
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.teamcode.cv.Detection;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.ourMovementLib.Follower;
import org.firstinspires.ftc.teamcode.ourOpModes.resources.IMU;
import org.firstinspires.ftc.teamcode.ourOpModes.resources.Timing;
import org.firstinspires.ftc.teamcode.ourOpModes.robotParts.Wobble_Arm;
import org.opencv.core.Mat;
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
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(group = "drive")
public class Autonomous extends LinearOpMode {
    DcMotorEx shooter, taco;
    DcMotor intake;
    CRServo shooter_roller1, shooter_roller2;
    Wobble_Arm wobble_arm;
    Timing timer = new Timing(this);

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().
                getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvCamera webcam = OpenCvCameraFactory.getInstance().
                createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        Detection detector = new Detection(telemetry);


        webcam.setPipeline(detector);

        //Opening and Streaming from Camera

        webcam.openCameraDeviceAsync(() -> {
            webcam.startStreaming(352, 288, OpenCvCameraRotation.UPRIGHT);
        });


        VuforiaPhone vuforia = new VuforiaPhone(hardwareMap, telemetry);
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        taco = hardwareMap.get(DcMotorEx.class, "taco");
        intake = hardwareMap.get(DcMotor.class, "intake");

        shooter_roller1 = hardwareMap.get(CRServo.class, "shooter_roller1");
        shooter_roller2 = hardwareMap.get(CRServo.class, "shooter_roller2");
        shooter_roller2.setDirection(DcMotorSimple.Direction.REVERSE);
        wobble_arm = new Wobble_Arm(hardwareMap, Autonomous.this);

        //intake.setPower(1.0);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // TwoWheelTrackingLocalizer myTwoWheelLoc = new TwoWheelTrackingLocalizer(hardwareMap, drive);

        //regulate_shooter_vel();

        IMU imu = new IMU(hardwareMap, telemetry);

        Follower follower = new Follower(drive, vuforia, this, telemetry, gamepad1, imu);


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
        shooter.setPower(-1);


        webcam.stopStreaming();

        // GRAB WOBBLE
        wobble_arm.grab();
        timer.safeDelay(200);
        wobble_arm.up();

        // Start Vuforia

        vuforia.beginTracking();

        follower.DRIVE_MAINTAIN_HEADING(0.9, -0, 0, 1100, imu);
        follower.DRIVE_MAINTAIN_HEADING(0, -0.4, 0, 500, imu);


//Great High Goal Position
//        follower.goTo(-4, 32, 0);
        follower.goTo(-4, 26, 0);

        telemetry.addData("Going to ", "High Goal");
        telemetry.update();

        //follower.goTo(-4, 6.9, 0);
        //shoot1();
        //follower.goTo(-8, -1, 0);
        //shoot2();
        //follower.goTo(-4, 2, 0);
        shoot3();

        shooter_roller1.setPower(0);
        shooter_roller2.setPower(0);

        shooter.setVelocity(0);



        if(detector.stack == 0) {
            // A BLOCK
            telemetry.addData("Shooting", "A Block");
            telemetry.update();
            follower.goTo(3, 39.8, 0); // vumark lock on position
            follower.goTo(13.2, 46, 0.3);
            follower.goToHeading(0);
        }
        else if(detector.stack == 1){
            // B BLOCK
            telemetry.addData("Shooting", "B Block");
            telemetry.update();
            follower.DRIVE_MAINTAIN_HEADING(0.5, 0,0, 2000, imu);
            //follower.goTo(3, 39.8, 0); // vumark lock on position
            //follower.goTo(35, 18, 0.42);
            follower.goToHeading(0);
        }
        else{
            // C BLOCK
            telemetry.addData("Shooting", "C Block");
            telemetry.update();
            follower.goTo(3, 45, 0); // vumark lock on position
            follower.DRIVE_MAINTAIN_HEADING(0.4, 0, 0, 2950, imu);
            follower.goToHeading(-0.3);
        }

        // PLACE WOBBLE
        wobble_arm.down();
        timer.safeDelay(500);
        wobble_arm.safeReleaseWobble();
        follower.goToHeading(0);



        if(detector.stack != 1 && detector.stack!= 0){
            follower.DRIVE_MAINTAIN_HEADING(-0.4, 0, 0, 2950, imu);
        }
        if(detector.stack == 1){
            follower.DRIVE_MAINTAIN_HEADING(-0.5, -0.0, 0, 1100, imu);
        }

        follower.goTo(10, 39.8, 0); // vumark lock on position

        //OpenGLMatrix location = vuforia.getLocation();
//        while(location == null){
//            // Search for left image
//            location = vuforia.getLocation();
//            follower.DRIVE(-0.4, 0, 0);
//        }


        /*



*/
        // Finally, park, or something

        // Powershot location
        // follower.goTo(-4, 22, -0.31);

        // high goal location
        // follower.goTo(4.5, 28.5, -0.07);
    }

    /*
    Notes for shooting, shoot 2 and 3 are not tested yet
    The idea is to separate the 3 rings into 3 different sections:
        1st ring: Second Roller only, place on the shooter entrance but not touching the fly wheel
        2nd ring: Second Roller and Taco, place between the taco and second roller
        3rd ring: Second Roller and Taco, place between intake and taco entrance
    * */

    volatile int shooter_vel = 0;
    void regulate_shooter_vel(){
        new Thread(()-> {
            while (!isStopRequested()) {
               // telemetry.addData("shooter vel", shooter.getVelocity());
                if (Math.abs(shooter.getVelocity()) < shooter_vel) {
                    shooter.setPower(-0.7);
                }else{
                    shooter.setPower(-0.7);
                }
            }
        }).start();
    }


    void shoot1() { // shoot 1st ring
        // spin shooter up
        shooter.setPower(-0.92);
        // wait 3 secs
        timer.safeDelay(3000);
        //for first ring only spin roller
        shooter_roller1.setPower(1);
        shooter_roller2.setPower(1);

        timer.safeDelay(3050);
        shooter.setPower(-0);
        shooter_roller1.setPower(0);
        shooter_roller2.setPower(0);
    }
    void shoot2() { //shoot 2nd ring
        // spin shooter up
        //THIS ONLY WORKS IN A LOOP, IT'S THE SAME AS shooter.setVelocity(...) without the if statement
        shooter.setPower(-0.92);
        // wait 3 secs
        timer.safeDelay(3000);
        // for 2nd ring only spin roller and taco

        shooter_roller1.setPower(1);
        shooter_roller2.setPower(1);
        taco.setPower(1);

        timer.safeDelay(4050);
        shooter.setPower(-0);
    }

    void shoot3() { //shoot 3rd ring
        // spin shooter up
        // wait 3 secs
        // for 2nd ring only spin roller and taco

        shooter_roller1.setPower(1);
        shooter_roller2.setPower(1);

        taco.setPower(0.3);
        timer.safeDelay(2000);
        taco.setPower(0.6);

        intake.setPower(-0.5);

        timer.safeDelay(6050);
        shooter.setPower(-0);
    }



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
