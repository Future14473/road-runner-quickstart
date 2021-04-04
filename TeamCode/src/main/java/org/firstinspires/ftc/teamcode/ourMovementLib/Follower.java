package org.firstinspires.ftc.teamcode.ourMovementLib;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.ourOpModes.resources.IMU;
import org.firstinspires.ftc.teamcode.ourOpModes.resources.RotationUtil;
import org.firstinspires.ftc.teamcode.ourOpModes.resources.Timing;
import org.firstinspires.ftc.teamcode.ourOpModes.resources.pose;
import org.firstinspires.ftc.teamcode.ourOpModes.robotParts.Mecanum;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

public class Follower {
    private final SampleMecanumDrive drivetrain;
    private final StandardTrackingWheelLocalizer odometry;
    private final Telemetry telemetry;
    private final LinearOpMode opmode;
    private final Gamepad gamepad;
    private static final float mmPerInch = 25.4f;



    public Follower(SampleMecanumDrive drivetrain, StandardTrackingWheelLocalizer odometry, LinearOpMode opmode, Telemetry telemetry, Gamepad gamepad, IMU imu){
        this.drivetrain = drivetrain;
        this.telemetry = telemetry;
        this.gamepad = gamepad;
        this.odometry = odometry;
        this.opmode = opmode;
    }

    public void goTo(double forwardAxisDest, double rightAxisDest, double turnAxisDest){
        // set arrived to true to exit the function
        boolean arrived = false;
        while (!arrived && opmode.opModeIsActive()){
            // get new position in Aviation coordinates

            VectorF translation = location.getTranslation();
            Orientation rotation = Orientation.getOrientation(location, EXTRINSIC, XYZ, DEGREES);
            // no fucking "x" or "y." Just forwardAxisPos and rightAxisPos for fucking clarity
            forwardAxisPos = translation.get(0) / mmPerInch;
            rightAxisPos = translation.get(1) / mmPerInch; //TODO Encoder configuration is flipped
            // X axis should be positive rightward, but is not. I'll fix it here, lest risk
            // breaking the config

            // Getting angle inconsistency that are really impacting the shooter accuracy
            //turnAxisPos =  imu.getHeading();
            turnAxisPos = Math.toRadians(rotation.thirdAngle);



            //destination
            telemetry.addData("Destination", String.format("V: %.1f H: %.1f R: %.2f", forwardAxisDest, rightAxisDest, turnAxisDest));

            // show current position (round decimals ffs)
            telemetry.addData("Current Position", String.format("V: %.1f H: %.1f R: %.2f", forwardAxisPos, rightAxisPos, turnAxisPos));

            // calculate how far robot needs to go
            double forwardDistance = forwardAxisDest - forwardAxisPos; // positive means forward
            double rightDistance = -(rightAxisDest - rightAxisPos); // positive means right
            double turnDistance = RotationUtil.turnLeftOrRight(turnAxisPos, turnAxisDest, Math.PI * 2); // positive means turn right

            // Careful here! Forward according to the coordinate plane
            // is NOT forward according to the robot
            // Rotate the distance to be in the perspective of the robot
            point robotDirection = new point(rightDistance, forwardDistance);
            robotDirection = robotDirection.rotate(turnDistance);
            // None of this rotation until the drivetrain gets weighted
            //forwardDistance = robotDirection.y;
            //rightDistance = robotDirection.x;

            // tell the user how far robot needs to go (round decimals ffs)
            telemetry.addData("Distance to Go",String.format("V: %.1f H: %.1f R: %.2f", forwardDistance, rightDistance, turnDistance));

            // decide how much power the robot should use to move on the forward axis
            double forwardPower = convertDistanceToPower(forwardDistance);
            // decide how much power the robot should use to move on the right axis
            double rightPower = convertDistanceToPower(rightDistance);
            // decide how much power the robot should use to turn
            double turnPower = convertAnglesToPower(turnDistance);

            telemetry.addData("Power",String.format("V: %.1f H: %.1f R: %.2f", forwardPower, rightPower, -turnPower));

            // A pressed means manual control
            // Otherwise, let the robot move to destination
            if (gamepad.a){
                // the y stick is negative when you push up, so invert it
                DRIVE(-gamepad.left_stick_y, gamepad.left_stick_x, gamepad.right_stick_x);
            }else{
                DRIVE(forwardPower, rightPower, -turnPower * 0.4);

                // if all the powers are 0 then we've arrived
                if(forwardPower == 0 && rightPower == 0 && turnPower == 0){
                    arrived = true;
                }
            }

            // tell telemetry to send the new data
            telemetry.update();
        }
    }

    public void goToHeading(double turnAxisDest){
        boolean arrived = false;
        while (!arrived && opmode.opModeIsActive()){
            // get new position in Aviation coordinates
            OpenGLMatrix location = vuforia.getLocation();
            // Getting angle inconsistency that are really impacting the shooter accuracy
            double turnAxisPos =  imu.getHeading(); //Math.toRadians(rotation.thirdAngle);

            telemetry.addData("Current Position", String.format("R: %.2f", turnAxisPos));


            // calculate how far robot needs to go
            double turnDistance = RotationUtil.turnLeftOrRight(turnAxisPos, turnAxisDest, Math.PI * 2); // positive means turn right

            telemetry.addData("Distance to Go",String.format("R: %.2f", turnDistance));


            // decide how much power the robot should use to turn
            double turnPower = convertAnglesToPower(turnDistance);

            telemetry.addData("Power",String.format("R: %.2f", -turnPower));


            // A pressed means manual control
            // Otherwise, let the robot move to destination
            if (gamepad.a){
                // the y stick is negative when you push up, so invert it
                DRIVE(-gamepad.left_stick_y, gamepad.left_stick_x, gamepad.right_stick_x);
            }else{
                DRIVE(0,0, -turnPower);

                // if all the powers are 0 then we've arrived
                if(turnPower == 0){
                    arrived = true;
                }
            }

            // tell telemetry to send the new data
            telemetry.update();
        }
    }
    /*
     * For POSITIVE forward parameter: go forward
     * For POSITIVE sideways parameter: go right
     * For POSITIVE rotate parameter: turn right
     */
    public void DRIVE(double forward, double sideways, double rotate) {
        drivetrain.setWeightedDrivePower(
                new Pose2d(
                        forward,
                        sideways, // note the drivetrain wheels are reversed so sideways is positive right
                        -rotate * 2
                )
        );
    }

    double convertDistanceToPower(double distance){
        // if within one inch, stop. That's close enough
        if(Math.abs(distance) < 1)
            return 0;

        double power;
        // anything within 7 inches means the robot starts slowing
        power = distance / 14; //needs to be slower, too jerky now
        // but too little power means the robot won't move at all
        if(Math.abs(power) < 0.15)
            // if power too low, make it higher
            power = 0.15 * Math.signum(power);

        if(Math.abs(power) > 0.6)
            // if power too high, make it lower
            power = 0.6 * Math.signum(power);

        return power; //it is over shoting like crazy rn --Kyle so i multiply by constant
    }

    double convertAnglesToPower(double angle){
        // if within 0.1 radian, stop. That's close enough
        if(Math.abs(angle) <= 0.04)
           return  0;

        double power;
        // anything within 0.05 rad means the robot starts slowing
        power = angle / 1.8;
        // but too little power means the robot won't move at all
        if(Math.abs(power) < 0.2)
            // if power too low, make it higher
            power = 0.2 * Math.signum(power);

        return power;
    }

    // radians
    public void DRIVE_MAINTAIN_HEADING(double forward_power, double right_power, double turnAxisDest, double miliseconds, IMU imu){
        long start_time = System.currentTimeMillis();
        while (System.currentTimeMillis() - start_time < miliseconds && !opmode.isStopRequested()) {
            double turnAxisPos = imu.getHeading();
            double turnDistance = RotationUtil.turnLeftOrRight(turnAxisPos, turnAxisDest, Math.PI * 2); // positive means turn right
            double turnPower = convertAnglesToPower(turnDistance);
            DRIVE(forward_power, right_power, -turnPower);
        }
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