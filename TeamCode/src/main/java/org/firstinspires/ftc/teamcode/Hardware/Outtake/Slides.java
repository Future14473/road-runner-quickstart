package org.firstinspires.ftc.teamcode.Hardware.Outtake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Slides {
    DcMotorEx LeftSlide;
    DcMotorEx RightSlide;

    public static int velocity = 5000;

    public static int retractInPos = 0;
    public static int highGoal = 737;
    public static int midGoalBlue = 376;
    public static int midGoalRed = 379;
    public static int preLowGoal = 265;
    public static int prepReturnFromLowHeight = 300;
    public static int lowGoal = 175;
    public static int sharedHeight = 250;


    public Slides(HardwareMap hardwareMap) {
        LeftSlide = hardwareMap.get(DcMotorEx.class, "frontSlide");
        RightSlide = hardwareMap.get(DcMotorEx.class, "backSlide");
    }

    public void resetEncoders(){
        LeftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftSlide.setTargetPosition(0);
        RightSlide.setTargetPosition(0);
        LeftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void scoreShared(){
        LeftSlide.setTargetPosition(sharedHeight);
        RightSlide.setTargetPosition(sharedHeight);
        LeftSlide.setVelocity(velocity);
        RightSlide.setVelocity(velocity);
    }
    public int getHeight(){return LeftSlide.getCurrentPosition();}

    public boolean isBusy(){return LeftSlide.isBusy();}

    public void extendHigh() {
        LeftSlide.setTargetPosition(highGoal);
        RightSlide.setTargetPosition(highGoal);
        LeftSlide.setVelocity(velocity);
        RightSlide.setVelocity(velocity);
    }

    public void extendMidBlue() {
        LeftSlide.setTargetPosition(midGoalBlue);
        RightSlide.setTargetPosition(midGoalBlue);
        LeftSlide.setVelocity(velocity);
        RightSlide.setVelocity(velocity);
    }

    public void extendMidRed() {
        LeftSlide.setTargetPosition(midGoalRed);
        RightSlide.setTargetPosition(midGoalRed);
        LeftSlide.setVelocity(velocity);
        RightSlide.setVelocity(velocity);
    }


    // todo make the shared hub less dump and lower
    public void extendLowPrepBlue() {
        LeftSlide.setTargetPosition(preLowGoal);
        RightSlide.setTargetPosition(preLowGoal);
        LeftSlide.setVelocity(velocity);
        RightSlide.setVelocity(velocity);
    }
    public void extendLow() {
        LeftSlide.setTargetPosition(lowGoal);
        RightSlide.setTargetPosition(lowGoal);
        LeftSlide.setVelocity(velocity);
        RightSlide.setVelocity(velocity);
    }

    public void preRetract(){
        LeftSlide.setTargetPosition(prepReturnFromLowHeight);
        RightSlide.setTargetPosition(prepReturnFromLowHeight);
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


