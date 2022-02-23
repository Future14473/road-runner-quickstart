package org.firstinspires.ftc.teamcode.Hardware.Opmodes;

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
    public static double intakeX = 49, intakeY = 65;
    public static double leaveX = 23;
    public static double endX = -4, endY = 43;
    public static double endAngle = 0;
  //  public static double collectX = 37, prepOutX = 5;
  //  public static double dumpX = -6, dumpY = 43, dumpR = 240;

    //.splineTo(new Vector2d(dumpX, dumpY), Math.toRadians(dumpR))

    @Override
    public void runOpMode() throws InterruptedException {
        SampleTankDrive tankDrive = new SampleTankDrive(hardwareMap);
        tankDrive.setPoseEstimate(new Pose2d(intakeX, intakeY));
        Trajectory trajectoryforward = tankDrive.trajectoryBuilder(new Pose2d(intakeX, intakeY), true)
                .splineTo(new Vector2d(leaveX, intakeY), Math.toRadians(0))
                .splineTo(new Vector2d(endX, endY), Math.toRadians(endAngle))
                .build();
        Trajectory trajectoryspline = tankDrive.trajectoryBuilder(new Pose2d(leaveX, intakeY), true)
                .splineTo(new Vector2d(endX, endY), Math.toRadians(endAngle)).build();


        Trajectory trajectorybackward = tankDrive.trajectoryBuilder(new Pose2d(endX,endY, endAngle), false)
                .splineTo(new Vector2d(leaveX, intakeY), Math.toRadians(0))
                .splineTo(new Vector2d(intakeX, intakeY), Math.toRadians(0))
                .build();
//        Trajectory trajectorysplinebackward = tankDrive.trajectoryBuilder(new Pose2d(leaveX, intakeY), false)
//                .splineTo(new Vector2d(intakeX, intakeY), Math.toRadians(endAngle)).build();



//        Trajectory intakeTraj = tankDrive.trajectoryBuilder(new Pose2d(intakeX, intakeY, 180))
//                .splineTo(new Vector2d(leaveX, intakeY), Math.toRadians(180))
//                .splineTo(new Vector2d(endX, endY), Math.toRadians(240))
//                .build();
//                .splineTo(new Vector2d(collectX, startY), Math.toRadians(0))
//                .build();
//        Trajectory toHalfOut = tankDrive.trajectoryBuilder(intakeTraj.end(), true)
//                .splineTo(new Vector2d(prepOutX, startY), Math.toRadians(0))
//                .build();
//        Trajectory toAllianceTraj = tankDrive.trajectoryBuilder(new Pose2d(prepOutX, startY, Math.toRadians(0)), true)
//                .splineTo(new Vector2d(dumpX, dumpY), Math.toRadians(dumpR))
//                .build();

        /*
        Trajectory fromAllianceTraj = tankDrive.trajectoryBuilder(new Pose2d(dumpX, dumpY, Math.toRadians(dumpR)), false)
                .splineTo(new Vector2d(prepOutX, startY), Math.toRadians(0))
                .build();
        Trajectory fromHalfOut = tankDrive.trajectoryBuilder(new Pose2d(prepOutX, startY, Math.toRadians(0)), false)
                .splineTo(new Vector2d(startX, startY), Math.toRadians(0))
                .build();


         */



        waitForStart();
        for (int i = 0; i < 100; i++) {
            tankDrive.followTrajectory(trajectoryforward);
            tankDrive.followTrajectory(trajectoryspline);
            tankDrive.followTrajectory(trajectorybackward);
//            tankDrive.followTrajectory(trajectorysplinebackward);
        }
//        tankDrive.followTrajectory(toHalfOut);
//        tankDrive.followTrajectory(toAllianceTraj);
        //tankDrive.followTrajectory(fromAllianceTraj);
        //tankDrive.followTrajectory(fromHalfOut);
    }
}
