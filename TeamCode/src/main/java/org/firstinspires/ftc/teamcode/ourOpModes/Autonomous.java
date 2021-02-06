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

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.TwoWheelTrackingLocalizer;
import org.firstinspires.ftc.teamcode.ourMovementLib.Follower;
import org.firstinspires.ftc.teamcode.ourOpModes.robotParts.Mecanum;

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
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        TwoWheelTrackingLocalizer myTwoWheelLoc = new TwoWheelTrackingLocalizer(hardwareMap, drive);
        Follower follower = new Follower(new Mecanum(hardwareMap), myTwoWheelLoc, Autonomous.this, telemetry);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (!isStopRequested()) {
            telemetry.addData("Follower Position", follower.getPositionOdoTest().toString());
            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("x Parallel Encoder", myTwoWheelLoc.parallelEncoder.getCorrectedVelocity());
            telemetry.addData("y Perpendicular Encoder", myTwoWheelLoc.perpendicularEncoder.getCorrectedVelocity());
            telemetry.update();
        }
    }
}
