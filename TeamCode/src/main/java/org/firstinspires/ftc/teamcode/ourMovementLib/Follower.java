package org.firstinspires.ftc.teamcode.ourMovementLib;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.TwoWheelTrackingLocalizer;
import org.firstinspires.ftc.teamcode.ourOpModes.resources.RotationUtil;
import org.firstinspires.ftc.teamcode.ourOpModes.resources.Timing;
import org.firstinspires.ftc.teamcode.ourOpModes.resources.pose;
import org.firstinspires.ftc.teamcode.ourOpModes.robotParts.Mecanum;

public class Follower {
    private final SampleMecanumDrive drivetrain;
    private final TwoWheelTrackingLocalizer odometry;
    private final Telemetry telemetry;
    private final LinearOpMode opmode;
    private final Gamepad gamepad;

    public pose position = new pose(0,0,0);

    public Follower(SampleMecanumDrive drivetrain, TwoWheelTrackingLocalizer odometry, LinearOpMode opmode, Telemetry telemetry, Gamepad gamepad){
        this.drivetrain = drivetrain;
        this.odometry = odometry;
        this.telemetry = telemetry;
        this.gamepad = gamepad;
        
        this.opmode = opmode;
    }

    public boolean goTo(PathPoint dest){
        // set arrived to true to exit the function
        boolean arrived = false;
        while (!arrived && opmode.opModeIsActive()){
            // make odometry calculate new position
            odometry.update();
            // get new position in Aviation coordinates
            Pose2d position = odometry.getPoseEstimate();
            // no fucking "x" or "y." Just forwardAxis and rightAxis for fucking clarity
            double forwardAxis = position.getX();
            double rightAxis = position.getY();
            double turnAxis = position.getHeading();

            // show current position (round decimals ffs)
            telemetry.addData("Current Position", String.format("V axis: %.2f H axis: %.2f", forwardAxis, rightAxis));

            // A pressed means manual control
            // Otherwise, let the robot move to destination
            if (gamepad.a){
                // the y stick is negative when you push up, so invert it
                DRIVE(-gamepad.right_stick_y, gamepad.right_stick_x, gamepad.left_stick_x);
            }else{
                // calculate how far robot needs to go
                double forwardDistance = dest.y - forwardAxis; // positive means forward
                double rightDistance = dest.x - rightAxis; // positive means right
                double turnDistance = RotationUtil.turnLeftOrRight(turnAxis, dest.dir, Math.PI * 2); // positive means turn right

                // tell the user how far robot needs to go (round decimals ffs)
                telemetry.addData("Distance to Go",String.format("Forward: %.2f Right: %.2f", forwardDistance, rightDistance));

                // decide how much power the robot should use to move on the forward axis
                double forwardPower;
                // if within one inch, stop. That's close enough
                if(forwardDistance < 1) {
                    forwardPower = 0;
                }else {
                    // anything within 5 inches means the robot starts slowing
                    forwardPower = forwardDistance / 5;
                    // but too little power means the robot won't move at all
                    if(forwardPower < 0.1)
                        // if power too low, make it higher
                        forwardPower = 0.1;
                }

                // decide how much power the robot should use to move on the right axis
                double rightPower;
                // if within one inch, stop. That's close enough
                if(forwardDistance < 1) {
                    rightPower = 0;
                }else {
                    // anything within 5 inches means the robot starts slowing
                    rightPower = rightDistance / 5;
                    // but too little power means the robot won't move at all
                    if(rightPower < 0.1)
                        // if power too low, make it higher
                        rightPower = 0.1;
                }

                // decide how much power the robot should use to turn
                double turnPower;
                // if within one radian, stop. That's close enough
                if(forwardDistance < 1) {
                    turnPower = 0;
                }else {
                    // anything within 0.7 rad means the robot starts slowing
                    turnPower = turnDistance / 0.7;
                    // but too little power means the robot won't move at all
                    if(turnPower < 0.01)
                        // if power too low, make it higher
                        turnPower = 0.01;
                }

                // convenient divide for testing
                DRIVE(forwardPower/2, rightPower/2, turnDistance/2);

                // if all the powers are 0 then we've arrived
                if(forwardPower == 0 && rightPower == 0 && turnPower == 0){
                    arrived = true;
                }

            }

            // tell telemetry to send the new data
            telemetry.update();
        }

        if(1==0)
            return true;
        else
            return false;
    }
    /*
     * For POSITIVE forward parameter: go forward
     * For POSITIVE sideways parameter: go right
     * For POSITIVE rotate parameter: turn right
     */
    void DRIVE(double forward, double sideways, double rotate) {
        drivetrain.setWeightedDrivePower(
                new Pose2d(
                        forward,
                        -sideways,
                        -rotate
                )
        );
    }


        /*
    boolean isArrived(PathPoint dest) {
        odometry.update();
        Pose2d pose2Dposition = odometry.getPoseEstimate();
        //note the order getY and getX becasue rr uses the aviation xy coordinate system
        position = new pose(-pose2Dposition.getY(),pose2Dposition.getX(),pose2Dposition.getHeading());

        pose diff = new pose(dest.x - position.x, dest.y - position.y,
                RotationUtil.turnLeftOrRight(position.r, dest.dir, Math.PI * 2));

        // to intrinsic
        //todo might need to remove this part
//        point intrinsic = new point(diff.x, diff.y).rotate(-position.r);
//        diff.x = intrinsic.x;
//        diff.y = intrinsic.y;

        // To consider:
        // 1) speeds below 0.1 cannot overcome static friction of drivetrain
        //    Therefore, all speeds below 0.1 will be rounded up to 0.1
        // 2) because of (1), robot will jerk when it gets near a point
        //    So stop moving when close enough

        double xVel = Math.abs(diff.x) < 2 ? 0 : Math.max(Math.abs(diff.x) / 200, 0.1) * -Math.signum(diff.x);
        double yVel = Math.abs(diff.y) < 2 ? 0 : Math.max(Math.abs(diff.y) / 200, 0.1) * Math.signum(diff.y);
        double rVel = Math.abs(diff.r) < 0.05 ? 0 : Math.max(Math.abs(diff.r), 0.1) * -Math.signum(diff.r);

        // because we're doing big motion, the robot tends to overshoot
        if(!gamepad.a) drivetrain.drive(xVel/3, yVel/3, 0*-rVel/3);
        telemetry.addData("Current Position cartesian coords", position.toString());
        telemetry.addData("To be travelled2", xVel + " " + yVel + " " + rVel);

        if(gamepad.a) {
            drivetrain.drive(
                    -gamepad.left_stick_y,
                    -gamepad.left_stick_x,
                    -gamepad.right_stick_x
            );
        }

        // return true if reached point
        return (xVel == 0 && yVel == 0 && rVel == 0);
    }

    public void debugGoTo(PathPoint dest){
        while (!debugIsArrived(dest) && opmode.opModeIsActive()){
            telemetry.addData("Destination is ", dest.toString());
            telemetry.update();
        }
    }

    public void austinTest(PathPoint dest){
        odometry.update();
        Pose2d pose2Dposition = odometry.getPoseEstimate();
        position = new pose(pose2Dposition.getY(),pose2Dposition.getX(),pose2Dposition.getHeading());
        pose diff = new pose(dest.x - position.x, dest.y - position.y,0);
        telemetry.addData("diff", diff.toString());
        telemetry.addData("Current Position Our pose class", position.toString());
        telemetry.addData("Current position Pose2D class", pose2Dposition.toString());
        telemetry.update();
        drivetrain.drive(0, diff.y, 0);
    }
    public void austinStop(){
        drivetrain.drive(0,0,0);
    }

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

        // todo fix the mecanum turn in place issue
        drivetrain.drive(0, 0, 0);
        telemetry.addData("diff", diff.toString());
        telemetry.addData("Current Position Our pose class", position.toString());
        telemetry.addData("Current position Pose2D class", pose2Dposition.toString());
//        telemetry.addData("To Point Amount", diff.toString());
//
//        // return true if reached point
//        return (xVel == 0 && yVel == 0 && rVel == 0);
        return (false);
    }*/


}