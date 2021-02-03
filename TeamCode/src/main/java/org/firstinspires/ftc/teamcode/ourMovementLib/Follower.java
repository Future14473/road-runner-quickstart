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

    public Follower(Mecanum drivetrain, TwoWheelTrackingLocalizer odometry, LinearOpMode opmode, Telemetry telemetry){
        this.drivetrain = drivetrain;
        this.odometry = odometry;
        this.telemetry = telemetry;
        
        this.opmode = opmode;
    }

    public void goTo(PathPoint dest){
        while (!isArrived(dest) && opmode.opModeIsActive()){
            // go to the point
        }}

    boolean isArrived(PathPoint dest) {

        Pose2d position = odometry.getPoseEstimate();

        pose diff = new pose(dest.x - position.getX(), dest.y - position.getY(),
                RotationUtil.turnLeftOrRight(position.getHeading(), dest.dir, Math.PI * 2));

        // to intrinsic
        point intrinsic = new point(diff.x, diff.y).rotate(-position.getHeading());
        diff.x = intrinsic.x;
        diff.y = intrinsic.y;

        Log.e("diff (intrinsic): ", String.format("%.1f %.1f %.1f", diff.x, diff.y, diff.r));

        // To consider:
        // 1) speeds below 0.1 cannot overcome static friction of drivetrain
        //    Therefore, all speeds below 0.1 will be rounded up to 0.1
        // 2) because of (1), robot will jerk when it gets near a point
        //    So stop moving when close enough

        double xVel = Math.abs(diff.x) < 2 ? 0 : Math.max(Math.abs(diff.x) / 200, 0.1) * Math.signum(diff.x);
        double yVel = Math.abs(diff.y) < 2 ? 0 : Math.max(Math.abs(diff.y) / 200, 0.1) * Math.signum(diff.y);
        double rVel = Math.abs(diff.r) < 0.05 ? 0 : Math.max(Math.abs(diff.r), 0.1) * Math.signum(diff.r);

        // because we're doing big motion, the robot tends to overshoot
        drivetrain.drive(xVel, yVel, rVel);

        telemetry.addData("To Point Amount", diff);

        // return true if reached point
        return (xVel == 0 && yVel == 0 && rVel == 0);
    }
}