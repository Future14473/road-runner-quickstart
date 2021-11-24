package org.firstinspires.ftc.teamcode.RobotParts;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Output {
    Servo bottomDumper;
    Servo topDumper;
    DcMotorEx linearSlide;
    DcMotorEx linearSlide2;

    public static double bottomDumperOutPos = -1.0;
    public static double bottomDumperInPos = 1.0;
    public static double topDumperOutPos = -1.0;
    public static double topDumperInPos = 1.0;
    public static int velocity = 1000;

    public Output(HardwareMap hardwareMap) {
        bottomDumper = hardwareMap.get(Servo.class, "bottomDumper");
        topDumper = hardwareMap.get(Servo.class, "topDumper");

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
        bottomDumper.setPosition(bottomDumperInPos);
        topDumper.setPosition(topDumperInPos);
    }
    public void flipOutDumper() {
        bottomDumper.setPosition(bottomDumperOutPos);
        topDumper.setPosition(topDumperOutPos);
    }

    public void extend() {
        linearSlide.setTargetPosition(-303);
        linearSlide2.setTargetPosition(-303);
        linearSlide.setVelocity(velocity);
        linearSlide2.setVelocity(velocity);
    }

    public void retract(){
        linearSlide.setTargetPosition(0);
        linearSlide2.setTargetPosition(0);
        linearSlide.setVelocity(velocity);
        linearSlide2.setVelocity(velocity);
    }

    public double getTopDumperPos() {
        return topDumper.getPosition();
    }
    public double getBottomDumperPos () {
        return bottomDumper.getPosition();
    }
}


