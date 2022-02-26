package org.firstinspires.ftc.teamcode.Hardware.Opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.Duck.Duck;
import org.firstinspires.ftc.teamcode.Hardware.Intake.Intake;
import org.firstinspires.ftc.teamcode.Hardware.Outtake.Turret;
import org.firstinspires.ftc.teamcode.Hardware.util.Timer;
import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;

@Autonomous
@Config
public class AutoRedDuck extends LinearOpMode {
    public static double preloadX = -20, preloadY = -54.5, preloadH = 290,
            duckX = -60, duckY = -66.5, duckH = 180,
            scoreDuckX = -30, scoreDuckY = -49, scoreDuckH = 0, parkX = 50, parkY = -42;
    public static long duckWait = 3000;


    @Override
    public void runOpMode() throws InterruptedException {
        SampleTankDrive drive = new SampleTankDrive(hardwareMap);
        Turret turret = new Turret(hardwareMap, this);
        Duck duck = new Duck(hardwareMap);
        Timer timer = new Timer(this);
        Intake intake = new Intake(hardwareMap);

        Pose2d start = new Pose2d(-36,70, Math.toRadians(270));
        Trajectory preload, duckPath, alignDuck, scoreDuck, park;
        preload = drive.trajectoryBuilder(start)
                .splineTo(new Vector2d(preloadX, preloadY), Math.toRadians(preloadH))
                .addTemporalMarker(0, () -> {
                    turret.preloadUp();
                })
                .build();

        drive.setPoseEstimate(start); //todo read into Ramsette heading

        turret.closeDumper();
        intake.drop();
        waitForStart();

        // Preload
        drive.followTrajectory(preload);
        drive.turnTo(Math.toRadians(preloadH));
        turret.preloadDown();

        duckPath = drive.trajectoryBuilder(drive.getPoseEstimate(), true)
                .splineTo(new Vector2d(duckX, duckY), Math.toRadians(duckH))
                .build();

        // Duck Drop
        drive.followTrajectory(duckPath);
        drive.turn(Math.toRadians(10));
//        drive.turnTo(0);
        duck.setBlue();
        duck.move();
        timer.safeDelay(duckWait);

        //Pickup Duck
        intake.in();
        drive.turnTo(Math.toRadians(90));
        drive.turnTo(Math.toRadians(0));
        turret.closeDumper();
        intake.stop();
        duck.setStop(); duck.move();

        scoreDuck = drive.trajectoryBuilder(drive.getPoseEstimate())
                .splineTo(new Vector2d(scoreDuckX, scoreDuckY), Math.toRadians(scoreDuckH))

                .build();

        // Score Duck
        drive.followTrajectory(scoreDuck);
        drive.turn(Math.toRadians(10));
        alignDuck = drive.trajectoryBuilder(drive.getPoseEstimate())
                .back(0.5)
                .build();
        drive.followTrajectory(alignDuck);
        drive.turnTo(Math.toRadians(0));
        turret.duckScorePrep();
        turret.down();

        park = drive.trajectoryBuilder(drive.getPoseEstimate())
                .splineTo(new Vector2d(parkX, parkY), Math.toRadians(0))
                .build();

        //park
        drive.followTrajectory(park);
    }
}
