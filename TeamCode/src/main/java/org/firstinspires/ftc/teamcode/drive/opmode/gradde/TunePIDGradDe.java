package org.firstinspires.ftc.teamcode.drive.opmode.gradde;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.gradde.mathUtils.GradientDescent;

import java.util.ArrayList;
import java.util.List;

/*
 * Op mode for preliminary tuning of the follower PID coefficients (located in the drive base
 * classes). The robot drives back and forth in a straight line indefinitely. Utilization of the
 * dashboard is recommended for this tuning routine. To access the dashboard, connect your computer
 * to the RC's WiFi network. In your browser, navigate to https://192.168.49.1:8080/dash if you're
 * using the RC phone or https://192.168.43.1:8080/dash if you are using the Control Hub. Once
 * you've successfully connected, start the program, and your robot will begin moving forward and
 * backward. You should observe the target position (green) and your pose estimate (blue) and adjust
 * your follower PID coefficients such that you follow the target position as accurately as possible.
 * If you are using SampleMecanumDrive, you should be tuning TRANSLATIONAL_PID and HEADING_PID.
 * If you are using SampleTankDrive, you should be tuning AXIAL_PID, CROSS_TRACK_PID, and HEADING_PID.
 * These coefficients can be tuned live in dashboard.
 *
 * This opmode is designed as a convenient, coarse tuning for the follower PID coefficients. It
 * is recommended that you use the FollowerPIDTuner opmode for further fine tuning.
 */
@Config
@Autonomous(group = "drive")
public class TunePIDGradDe extends LinearOpMode {

    public static double DISTANCE = 50;
    public static double startLearningRateKv = 0.01;

    @Override
    public void runOpMode() throws InterruptedException {
        /*Set up the path*/ SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory trajectoryForward = drive.trajectoryBuilder(new Pose2d())
                .forward(DISTANCE)
                .build();

        Trajectory trajectoryBackward = drive.trajectoryBuilder(trajectoryForward.end())
                .back(DISTANCE)
                .build();

        GradientDescent kPTuner = new GradientDescent(startLearningRateKv, 1);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {

            // collect data first time
            SampleMecanumDrive.TRANSLATIONAL_PID.kP = 1.0;
            drive.followTrajectory(trajectoryForward);
            drive.followTrajectory(trajectoryBackward);
            }
        int count = 0;
        for (Pose2d error: drive.arrError) {
            Log.println(Log.ASSERT, "Array Error " + String.valueOf(count), String.valueOf(Math.toDegrees(error.getHeading())));
            count++;
        }


//            ArrayList<Pose2d> firstLoss = drive.arrError;
//            firstLoss.forEach(Pose2d pos.heading: );
//            drive.resetArrError();

            // collect data for second time
//            SampleMecanumDrive.TRANSLATIONAL_PID.kP = 2.0;
//            drive.followTrajectoryAsyncRecordLoss(trajectoryForward);
//            drive.followTrajectoryAsyncRecordLoss(trajectoryBackward);
//
//            SampleMecanumDrive.HEADING_PID.kP = kPTuner.getNextTunerVar(drive.arrError);

        }
    }
