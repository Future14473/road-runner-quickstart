package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;

@Config
@TeleOp
public class CycleTest extends LinearOpMode {
    public static double startX = 23, startY = 65;
    public static double collectX = 37, prepOutX = 5;
    public static double dumpX = -6, dumpY = 43, dumpR = 240;

    //.splineTo(new Vector2d(dumpX, dumpY), Math.toRadians(dumpR))

    @Override
    public void runOpMode() throws InterruptedException {
        SampleTankDrive tankDrive = new SampleTankDrive(hardwareMap);
        tankDrive.setPoseEstimate(new Pose2d(startX, startY, 0));
        Trajectory intakeTraj = tankDrive.trajectoryBuilder(new Pose2d(startX,startY, 0))
                .splineTo(new Vector2d(collectX, startY), Math.toRadians(0))
                .build();
        Trajectory toHalfOut = tankDrive.trajectoryBuilder(intakeTraj.end(), true)
                .splineTo(new Vector2d(prepOutX, startY), Math.toRadians(0))
                .build();
        Trajectory toAllianceTraj = tankDrive.trajectoryBuilder(new Pose2d(prepOutX, startY, Math.toRadians(0)), true)
                .splineTo(new Vector2d(dumpX, dumpY), Math.toRadians(dumpR))
                .build();

        /*
        Trajectory fromAllianceTraj = tankDrive.trajectoryBuilder(new Pose2d(dumpX, dumpY, Math.toRadians(dumpR)), false)
                .splineTo(new Vector2d(prepOutX, startY), Math.toRadians(0))
                .build();
        Trajectory fromHalfOut = tankDrive.trajectoryBuilder(new Pose2d(prepOutX, startY, Math.toRadians(0)), false)
                .splineTo(new Vector2d(startX, startY), Math.toRadians(0))
                .build();


         */



        waitForStart();
        tankDrive.followTrajectory(intakeTraj);
        tankDrive.followTrajectory(toHalfOut);
        tankDrive.followTrajectory(toAllianceTraj);
        //tankDrive.followTrajectory(fromAllianceTraj);
        //tankDrive.followTrajectory(fromHalfOut);
    }
}
