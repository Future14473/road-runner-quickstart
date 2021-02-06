package org.firstinspires.ftc.teamcode.ourMovementLib;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.TwoWheelTrackingLocalizer;
import org.firstinspires.ftc.teamcode.ourOpModes.resources.RotationUtil;
import org.firstinspires.ftc.teamcode.ourOpModes.resources.pose;
import org.firstinspires.ftc.teamcode.ourOpModes.robotParts.Mecanum;

public class Follower {
    private final Mecanum drivetrain;
    private final TwoWheelTrackingLocalizer odometry;
    private final Telemetry telemetry;
    private final LinearOpMode opmode;
    public pose position = new pose(0,0,0);

    public Follower(Mecanum drivetrain, TwoWheelTrackingLocalizer odometry, LinearOpMode opmode, Telemetry telemetry){
        this.drivetrain = drivetrain;
        this.odometry = odometry;
        this.telemetry = telemetry;
        
        this.opmode = opmode;
    }

    public Pose2d getPositionOdoTest() {
        return odometry.getPoseEstimate();
    }

    public void goTo(PathPoint dest){
        while (!isArrived(dest) && opmode.opModeIsActive()){
            telemetry.addData("Destination is ", dest.toString());
            telemetry.update();
        }}

    boolean isArrived(PathPoint dest) {
        odometry.update();
        Pose2d pose2Dposition = odometry.getPoseEstimate();
        //note the order getY and getX becasue rr uses the official aviation xy coordinate system
        position = new pose(pose2Dposition.getY(),pose2Dposition.getX(),pose2Dposition.getHeading());

        pose diff = new pose(dest.x - position.x, dest.y - position.y,
                RotationUtil.turnLeftOrRight(position.r, dest.dir, Math.PI * 2));

        // to intrinsic
        //todo might need to remove this part
//        point intrinsic = new point(diff.x, diff.y).rotate(-position.r);
//        diff.x = intrinsic.x;
//        diff.y = intrinsic.y;

        Log.e("diff (intrinsic): ", String.format("%.1f %.1f %.1f", diff.x, diff.y, diff.r));
        Log.e("Current position", String.format("%.1f %.1f %.1f", position.x, position.y, position.r));

        // To consider:
        // 1) speeds below 0.1 cannot overcome static friction of drivetrain
        //    Therefore, all speeds below 0.1 will be rounded up to 0.1
        // 2) because of (1), robot will jerk when it gets near a point
        //    So stop moving when close enough

        double xVel = Math.abs(diff.x) < 2 ? 0 : Math.max(Math.abs(diff.x) / 200, 0.1) * -Math.signum(diff.x);
        double yVel = Math.abs(diff.y) < 2 ? 0 : Math.max(Math.abs(diff.y) / 200, 0.1) * -Math.signum(diff.y);
        double rVel = Math.abs(diff.r) < 0.05 ? 0 : Math.max(Math.abs(diff.r), 0.1) * Math.signum(diff.r);

        // because we're doing big motion, the robot tends to overshoot
        drivetrain.drive(xVel/3, yVel/3, rVel/3);
        telemetry.addData("Current Position Our pose class", position.toString());
        telemetry.addData("Current position Pose2D class", pose2Dposition.toString());
        telemetry.addData("To Point Amount", diff.toString());

        // return true if reached point
        return (xVel == 0 && yVel == 0 && rVel == 0);
    }

    public void debugGoTo(PathPoint dest){
        while (!debugIsArrived(dest) && opmode.opModeIsActive()){
            telemetry.addData("Destination is ", dest.toString());
            telemetry.update();
        }}

    boolean debugIsArrived(PathPoint dest) {
        odometry.update();
        Pose2d pose2Dposition = odometry.getPoseEstimate();
        //note the order getY and getX becasue rr uses the official aviation xy coordinate system
        position = new pose(pose2Dposition.getY(),pose2Dposition.getX(),pose2Dposition.getHeading());
        pose diff = new pose(dest.x - position.x, dest.y - position.y,42000);
//                RotationUtil.turnLeftOrRight(position.r, dest.dir, Math.PI * 2));
//
//        // to intrinsic
//        //todo might need to remove this part
////        point intrinsic = new point(diff.x, diff.y).rotate(-position.r);
////        diff.x = intrinsic.x;
////        diff.y = intrinsic.y;
//
//        Log.e("diff (intrinsic): ", String.format("%.1f %.1f %.1f", diff.x, diff.y, diff.r));
//        Log.e("Current position", String.format("%.1f %.1f %.1f", position.x, position.y, position.r));
//
//        // To consider:
//        // 1) speeds below 0.1 cannot overcome static friction of drivetrain
//        //    Therefore, all speeds below 0.1 will be rounded up to 0.1
//        // 2) because of (1), robot will jerk when it gets near a point
//        //    So stop moving when close enough
//
//        double xVel = Math.abs(diff.x) < 2 ? 0 : Math.max(Math.abs(diff.x) / 200, 0.1) * -Math.signum(diff.x);
//        double yVel = Math.abs(diff.y) < 2 ? 0 : Math.max(Math.abs(diff.y) / 200, 0.1) * -Math.signum(diff.y);
//        double rVel = Math.abs(diff.r) < 0.05 ? 0 : Math.max(Math.abs(diff.r), 0.1) * Math.signum(diff.r);
//
//        // because we're doing big motion, the robot tends to overshoot
//        drivetrain.drive(xVel/3, yVel/3, rVel/3);
        drivetrain.drive(0, diff.y, 0);
        telemetry.addData("diff", diff.toString());
        telemetry.addData("Current Position Our pose class", position.toString());
        telemetry.addData("Current position Pose2D class", pose2Dposition.toString());
//        telemetry.addData("To Point Amount", diff.toString());
//
//        // return true if reached point
//        return (xVel == 0 && yVel == 0 && rVel == 0);
        return (false);
    }


}