package org.firstinspires.ftc.teamcode.ourOpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Roadrunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Roadrunner.TwoWheelTrackingLocalizer;
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
        Pose2d startPose = new Pose2d(10, -8, Math.toRadians(90));

        waitForStart();
        drive.setPoseEstimate(startPose);

        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(20, 9), Math.toRadians(45))
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .splineTo(new Vector2d(20, 9), Math.toRadians(45))
                .build();

        drive.followTrajectory(traj1);
        drive.followTrajectory(traj2);

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
