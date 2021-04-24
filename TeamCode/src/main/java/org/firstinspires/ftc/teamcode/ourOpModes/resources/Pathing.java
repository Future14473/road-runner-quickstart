package org.firstinspires.ftc.teamcode.ourOpModes.resources;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.Roadrunner.SampleMecanumDrive;

public class Pathing {
    SampleMecanumDrive drive;
    public static Pose2d startTeleopPosition = null;

    public Pathing(SampleMecanumDrive drive){
        this.drive = drive;
    }

    public void setAutoHome(){
        Pose2d startPose = new Pose2d(-54.5, 20, 0);
        drive.setPoseEstimate(startPose);
    }

    public void highGoal(){
        goToLineConstant(-10, 33, Math.toRadians(20));
    }

    public void collectPos(){
        goToLineConstant(0, 30, Math.toRadians(35));
    }

    public void powerShot1(){
        goToLineConstant(-3.78, 15.42, Math.toRadians(17.27));
    }

    public void powerShot2(){
        goToLineConstant(-3.78, 15.42, Math.toRadians(9.5));
    }

    public void powerShot3(){
        goToLineConstant(-3.78, 15.42, Math.toRadians(6.5));
    }



    public void goToSplineHeading(double x, double y, double heading){
        Trajectory destination = drive.trajectoryBuilder(drive.getPoseEstimate())
                .splineToSplineHeading(new Pose2d(x, y, heading), heading)
                .build();

        drive.followTrajectory(destination);
    }


    public void goToLineConstant(double x, double y, double heading){
        Pose2d p = drive.getPoseEstimate();
        double pnAngle = p.getHeading() <= Math.PI ? p.getHeading(): p.getHeading() - 2* Math.PI;
        boolean reverse = Math.abs(pnAngle) < Math.PI / 2 && drive.getPoseEstimate().getX() > x;
        ;
        Trajectory destination = drive.trajectoryBuilder(drive.getPoseEstimate(), reverse)
                .lineToConstantHeading(new Vector2d(x, y))
                .build();

        drive.followTrajectory(destination);
    }

    public void goToLineStraight(double x, double y){
        Pose2d p = drive.getPoseEstimate();
        double pnAngle = p.getHeading() <= Math.PI ? p.getHeading(): p.getHeading() - 2* Math.PI;
        boolean reverse = Math.abs(pnAngle) < Math.PI / 2 && drive.getPoseEstimate().getX() > x;
        ;
        Trajectory destination = drive.trajectoryBuilder(drive.getPoseEstimate(), reverse)
                .lineToLinearHeading(new Pose2d(x,y,0))
                .build();

        drive.followTrajectory(destination);
    }


    public void DRIVE(double forward, double sideways, double rotate, SampleMecanumDrive drivetrain) {
        drivetrain.setWeightedDrivePower(
                new Pose2d(
                        forward,
                        -sideways, // note the drivetrain wheels are reversed so sideways is positive right
                        -rotate * 2
                )
        );
    }
}
