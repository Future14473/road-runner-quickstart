package org.firstinspires.ftc.teamcode.RobotParts;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Output {
    Servo dumper;
    Servo dumper2;
    DcMotorEx linearSlide;
    DcMotorEx linearSlide2;

    public static double dumperOutPos = 0.83;
    public static double dumperOutPosHalf = 0.67;
    public static double dumperInPos = 0.5;
    public static int velocity = 1000;

    public Output(HardwareMap hardwareMap) {
        dumper = hardwareMap.get(Servo.class, "dumper");
        dumper2 = hardwareMap.get(Servo.class, "dumper2");

        linearSlide = hardwareMap.get(DcMotorEx.class, "linearSlide");
        linearSlide2 = hardwareMap.get(DcMotorEx.class, "linearSlide2");


        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlide.setTargetPosition(0);
        linearSlide2.setTargetPosition(0);
        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }
    public void flipInDumper() {
        dumper.setPosition(dumperInPos);
        dumper2.setPosition(dumperInPos);
    }
    public void flipHalfDumper() {
        dumper.setPosition(dumperOutPosHalf);
        dumper2.setPosition(dumperOutPosHalf);
    }
    public void flipOutDumper() {
        dumper.setPosition(dumperOutPos);
        dumper2.setPosition(dumperOutPos);
    }


    public void extend() {
        linearSlide.setTargetPosition(-415);
        linearSlide2.setTargetPosition(-415);
        linearSlide.setVelocity(velocity);
        linearSlide2.setVelocity(velocity);
    }

    public void retract(){
        linearSlide.setTargetPosition(0);
        linearSlide2.setTargetPosition(0);
        linearSlide.setVelocity(velocity);
        linearSlide2.setVelocity(velocity);
    }

    public void retractFlipIn(){
        flipInDumper();
        retract();
    }

    public double getTopDumperPos() {
        return dumper.getPosition();
    }
    public double getBottomDumperPos () {
        return dumper.getPosition();
    }
}


