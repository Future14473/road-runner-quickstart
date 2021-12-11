package org.firstinspires.ftc.teamcode.RobotParts;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class EncoderMecanum {
    private DcMotorEx leftFront, leftRear, rightRear, rightFront;
    public static int drivetrainSpeed = 1000;
    public static double ENCODERS_PER_INCHES = 1;
    public static double ENCODERS_PER_DEGREES = 1;

    public EncoderMecanum(HardwareMap hardwareMap){
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);

//        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void moveEncoders(int forward, int strafe, int turn){
        leftFront.setTargetPosition(forward + strafe + turn);
        leftRear.setTargetPosition(forward - strafe + turn);
        rightFront.setTargetPosition(forward - strafe - turn);
        rightRear.setTargetPosition(forward + strafe - turn);


        leftFront.setVelocity(drivetrainSpeed);
        leftRear.setVelocity(drivetrainSpeed);
        rightFront.setVelocity(drivetrainSpeed);
        rightRear.setVelocity(drivetrainSpeed);


    }

    public int inToEncoders(double inches){
        return (int)(inches * ENCODERS_PER_INCHES);
    }

    public int degreesToEncoders (double degrees){
        return (int)(degrees * ENCODERS_PER_DEGREES);
    }

    public void moveInches(double forwardInches, double strafeInches, double turnDegrees){
        moveEncoders(inToEncoders(forwardInches), inToEncoders(strafeInches), degreesToEncoders(turnDegrees));

    }
}
