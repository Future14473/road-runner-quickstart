package org.firstinspires.ftc.teamcode.RobotParts;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;

import java.util.Arrays;
import java.util.List;

@Config
public class EncoderMecanum {
    public DcMotorEx leftFront, leftRear, rightRear, rightFront;
    Telemetry telemetry;
//    public static PIDFCoefficients pidCoefficients = new PIDFCoefficients(10,0.5,0,
//            DriveConstants.getMotorVelocityF(
//                    DriveConstants.MAX_RPM / 60 * DriveConstants.TICKS_PER_REV)
//            );
    public static int drivetrainSpeed = 1000;
    public static double ENCODERS_PER_INCHES = 4666;
    public static double ENCODERS_PER_DEGREES = 1;
    private VoltageSensor batteryVoltageSensor;
    private List<DcMotorEx> motors;

    public EncoderMecanum(HardwareMap hardwareMap, Telemetry telemetry){
        this.telemetry = telemetry;
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

//        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
//        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

//        setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidCoefficients);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setTargetPosition(0);
        leftRear.setTargetPosition(0);
        rightFront.setTargetPosition(0);
        rightRear.setTargetPosition(0);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);


    }

    public void moveEncoders(int forward, int strafe, int turn){
        leftFront.setTargetPosition(forward + strafe + turn);
        leftRear.setTargetPosition(forward - strafe + turn);
        rightFront.setTargetPosition(forward - strafe - turn);
        rightRear.setTargetPosition(forward + strafe - turn);
        for (DcMotorEx motor : motors){
            telemetry.addData("Target Position", motor.getTargetPosition());
        }


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

    public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
        PIDFCoefficients compensatedCoefficients = new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d,
                coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        );
        for (DcMotorEx motor : motors) {
            motor.setPIDFCoefficients(runMode, compensatedCoefficients);
        }
    }

    public void telemetryEncoderPositions(){
        for (DcMotorEx motor : motors){
            telemetry.addData("Motor Encoder Position", motor.getCurrentPosition());
        }
    }

}
