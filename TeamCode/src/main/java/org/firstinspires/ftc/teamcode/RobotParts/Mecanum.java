package org.firstinspires.ftc.teamcode.RobotParts;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Mecanum {
    RampVelocityMotor frontLeft;
    RampVelocityMotor frontRight;
    RampVelocityMotor backLeft;
    RampVelocityMotor backRight;

    //for debugging
    double frontLeftSpeed;
    double frontRightSpeed;
    double backLeftSpeed;
    double backRightSpeed;

    private static final double maxTicksPerSec = 2000;
    private static final double wheelToCenter = 21;

    //top l, top r, bottom l, bottom r
    public Mecanum(HardwareMap hardwareMap){  //todo bring back the list of motors
        frontLeft   = new RampVelocityMotor(hardwareMap.get(DcMotorEx.class, "frontLeft"));
        frontRight  = new RampVelocityMotor(hardwareMap.get(DcMotorEx.class, "frontRight"));
        backRight   = new RampVelocityMotor(hardwareMap.get(DcMotorEx.class, "backRight"));
        backLeft    = new RampVelocityMotor(hardwareMap.get(DcMotorEx.class, "backLeft"));

        frontRight.self.setDirection(DcMotorEx.Direction.REVERSE);
        backRight.self.setDirection(DcMotorEx.Direction.REVERSE);
    }
//    // for more debugging
//    double getFrontLeftSpeed(){ return frontLeftSpeed; }
//    double getBackLeftSpeed(){ return backLeftSpeed; }
//    double getFrontRightSpeed(){ return frontRightSpeed; }
//    double getBackRightSpeed(){ return backRightSpeed; }

    //turn speed be in robot angle

//    public void driveTeleopPower(double xSpeed, double ySpeed, double turnSpeed) {
//
//        double frontLeftSpeed = ySpeed + xSpeed + turnSpeed;
//        double frontRightSpeed = ySpeed - xSpeed - turnSpeed;
//        double backLeftSpeed = ySpeed - xSpeed + turnSpeed;
//        double backRightSpeed = ySpeed + xSpeed - turnSpeed;
//
//        frontLeft.setPower(frontLeftSpeed);
//        frontRight.setVelocity(frontRightSpeed);
//        backLeft.setVelocity(backLeftSpeed);
//        backRight.setVelocity(backRightSpeed);
//    }

    public void drive(double xSpeed, double ySpeed, double turnSpeed){
        // radians = circumference / radius
        //turnSpeed /= wheelToCenter;

//        point move = new point(xSpeed, ySpeed);
//
//        maximum max = new maximum(
//                move.y + move.x + turnSpeed,  //FL
//                move.y - move.x - turnSpeed,        //FR
//                move.y - move.x + turnSpeed,        //BL
//                move.y + move.x - turnSpeed);       //BR
        //max.squishIntoRange(1.0);


        frontLeft.setVelocity(  (int)(maxTicksPerSec * (-ySpeed - xSpeed + turnSpeed)));
        frontRight.setVelocity( (int)(maxTicksPerSec * (-ySpeed + xSpeed - turnSpeed)));
        backLeft.setVelocity(   (int)(maxTicksPerSec * (-ySpeed + xSpeed + turnSpeed)));
        backRight.setVelocity(  (int)(maxTicksPerSec * (-ySpeed - xSpeed - turnSpeed)));

//        //for debugging
//        frontLeftSpeed = frontLeft.getVelocity();
//        frontRightSpeed = frontRight.getVelocity();
//        backLeftSpeed = backLeft.getVelocity();
//        backRightSpeed = backRight.getVelocity();
    }


}
