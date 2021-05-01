package org.firstinspires.ftc.teamcode.ourOpModes.resources;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.Roadrunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RobotParts.Wobble_Arm;
import org.opencv.core.Mat;

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

    public void goToTwoSplineHeading(double x, double y, double heading, double x2, double y2, double heading2){
        Trajectory destination = drive.trajectoryBuilder(drive.getPoseEstimate())
                .splineToSplineHeading(new Pose2d(x, y, heading), heading)
                .splineToSplineHeading(new Pose2d(x2, y2), heading2)
                .build();

        drive.followTrajectory(destination);
    }

    PIDFController turn_PID = new PIDFController(new PIDCoefficients(1, 0, 0));
    public void turn_to_heading_PID(double dest_heading, double power_coeff, double forward, double left){

        double heading_delta = RotationUtil.turnLeftOrRight(
                drive.getPoseEstimate().getHeading(), dest_heading, Math.PI*2);

        turn_PID.setTargetPosition(heading_delta);
        turn_PID.setTargetVelocity(0);

        double heading_correction = turn_PID.update(0) * power_coeff;
        if(Math.abs(heading_delta) < Math.toRadians(3))
            heading_correction = 0;

        drive.setDrivePower((new Pose2d(forward, left, heading_correction)));
    }

    public void turn_relative(double angle_delta, double forward, double left){
        drive.setDrivePower((new Pose2d(forward, left, angle_delta)));
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

    public void goToLine(double x, double y, double heading){
        Pose2d p = drive.getPoseEstimate();
        double pnAngle = p.getHeading() <= Math.PI ? p.getHeading(): p.getHeading() - 2* Math.PI;
        boolean reverse = Math.abs(pnAngle) < Math.PI / 2 && drive.getPoseEstimate().getX() > x;
        ;
        Trajectory destination = drive.trajectoryBuilder(drive.getPoseEstimate(), reverse)
                .lineToLinearHeading(new Pose2d(x, y, heading))
                .build();

        drive.followTrajectory(destination);
    }

    public void goToLineSlow(double x, double y, double heading){
        Pose2d p = drive.getPoseEstimate();
        double pnAngle = p.getHeading() <= Math.PI ? p.getHeading(): p.getHeading() - 2* Math.PI;
        boolean reverse = Math.abs(pnAngle) < Math.PI / 2 && drive.getPoseEstimate().getX() > x;

//        Trajectory destination = drive.trajectoryBuilder(drive.getPoseEstimate(), reverse)
//                .lineToLinearHeading(new Pose2d(x, y, heading))
//                .build();

//        Trajectory destination = drive.trajectoryBuilder(drive.getPoseEstimate(), false)
//                // This spline is limited to 15 in/s and will be slower
//                .splineTo(
//                        new Vector2d(30, 30), 0,
//                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
//                )

        drive.followTrajectory(destination);
    }

    public void goToLineWobbleDown(double x, double y, double heading, double dropPoint, Wobble_Arm wobble_arm){
        Pose2d p = drive.getPoseEstimate();
        double pnAngle = p.getHeading() <= Math.PI ? p.getHeading(): p.getHeading() - 2* Math.PI;
        boolean reverse = Math.abs(pnAngle) < Math.PI / 2 && drive.getPoseEstimate().getX() > x;
        ;
        Trajectory destination = drive.trajectoryBuilder(drive.getPoseEstimate(), reverse)
                .lineToLinearHeading(new Pose2d(x, y, heading))
                .addTemporalMarker(dropPoint, wobble_arm::down)
                .build();

        drive.followTrajectory(destination);
    }

    public void goToLineWobbleUp(double x, double y, double heading, double dropPoint, Wobble_Arm wobble_arm){
        Pose2d p = drive.getPoseEstimate();
        double pnAngle = p.getHeading() <= Math.PI ? p.getHeading(): p.getHeading() - 2* Math.PI;
        boolean reverse = Math.abs(pnAngle) < Math.PI / 2 && drive.getPoseEstimate().getX() > x;
        ;
        Trajectory destination = drive.trajectoryBuilder(drive.getPoseEstimate(), reverse)
                .lineToLinearHeading(new Pose2d(x, y, heading))
                .addTemporalMarker(dropPoint, wobble_arm::up)
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
