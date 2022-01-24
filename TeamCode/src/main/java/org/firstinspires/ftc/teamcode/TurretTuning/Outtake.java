package org.firstinspires.ftc.teamcode.TurretTuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Outtake {
    DcMotorEx LeftSlide;
    DcMotorEx RightSlide;

    public static int velocity = 1000;

    public static int retractInPos = 0;
    public static int highGoal = -415;
    public static int midGoal = -290;
    public static int lowGoal = -200;


    public Outtake(HardwareMap hardwareMap) {

        LeftSlide = hardwareMap.get(DcMotorEx.class, "linearSlide");
        RightSlide = hardwareMap.get(DcMotorEx.class, "linearSlide2");


        LeftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftSlide.setTargetPosition(0);
        RightSlide.setTargetPosition(0);
        LeftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }


    public void extendHigh() {
        LeftSlide.setTargetPosition(highGoal);
        RightSlide.setTargetPosition(highGoal);
        LeftSlide.setVelocity(velocity);
        RightSlide.setVelocity(velocity);
    }

    public void extendMid() {
        LeftSlide.setTargetPosition(midGoal);
        RightSlide.setTargetPosition(midGoal);
        LeftSlide.setVelocity(velocity);
        RightSlide.setVelocity(velocity);
    }

    public void extendLow() {
        LeftSlide.setTargetPosition(lowGoal);
        RightSlide.setTargetPosition(lowGoal);
        LeftSlide.setVelocity(velocity);
        RightSlide.setVelocity(velocity);
    }

    public void retract() {
        LeftSlide.setTargetPosition(retractInPos);
        RightSlide.setTargetPosition(retractInPos);
        LeftSlide.setVelocity(velocity);
        RightSlide.setVelocity(velocity);
    }
}


