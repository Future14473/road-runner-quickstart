package org.firstinspires.ftc.teamcode.ourOpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Roadrunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Roadrunner.TwoWheelTrackingLocalizer;

import java.util.Vector;

@TeleOp(group = "teleop")
public class AutonomousPointsTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
//        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
//
//        // all of the trajectories/poses
//        Pose2d startPose = new Pose2d(-47, 9, Math.toRadians(0));
//        Trajectory toHighGoal = drive.trajectoryBuilder(startPose)
//                .splineTo(new Vector2d(57.64, -3.37), 0.336)
//                .build();
//
//        waitForStart();
//
//        drive.setPoseEstimate(startPose);
//
//        while (opModeIsActive()){
//            drive.followTrajectory(toHighGoal);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // We want to start the bot at x: 10, y: -8, heading: 90 degrees
        Pose2d startPose = new Pose2d(-60.8, 30.92, 0);

        waitForStart();
        drive.setPoseEstimate(startPose);

        Trajectory toHighGoal = drive.trajectoryBuilder(drive.getPoseEstimate())
                .splineTo(new Vector2d(-6.5, 27.4), Math.toRadians(12.5))
                .build();

        Trajectory toCollection = drive.trajectoryBuilder(drive.getPoseEstimate())
                .splineTo(new Vector2d(2.9, 24.9), 0)
                .build();
        while (opModeIsActive()){
            if (gamepad1.dpad_up){
                drive.followTrajectory(toHighGoal);
            }

            if (gamepad1.dpad_right){
                drive.followTrajectory(toCollection);

            }
        }


//            if (gamepad1.b){
//                //go to starting point
//                follower.goTo(0,0,0);
//            }
//
//            if (gamepad1.dpad_up){
//                follower.goTo(30,0,0);
//            }
//
//            if (gamepad1.dpad_down){
//                follower.goTo(0,24,0);
//            }
//
//            if (gamepad1.dpad_right){
//                follower.goTo(24,24,0);
//            }
//
//            if (gamepad1.dpad_left){
//                follower.goTo(0,0,0);
//            }



    }
}
