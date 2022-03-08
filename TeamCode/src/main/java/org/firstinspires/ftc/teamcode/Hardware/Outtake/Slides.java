package org.firstinspires.ftc.teamcode.Hardware.Outtake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Slides {
    DcMotorEx frontSlide;
    DcMotorEx backSlide;

    public static int velocity = 5000;

    public static int home = 0;

    public static int highGoal = 737;

    public static int midGoalBlue = 376;
    public static int midGoalRed = 379;

    public static int preLowGoal = 265;
    public static int prepReturnFromLowHeight = 300;
    public static int lowGoal = 175;

    public static int sharedHeight = 160;

    public static int incrementAmt = 50;


    public Slides(HardwareMap hardwareMap) {
        frontSlide = hardwareMap.get(DcMotorEx.class, "frontSlide");
        backSlide = hardwareMap.get(DcMotorEx.class, "backSlide");
        frontSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontSlide.setTargetPosition(0);
        backSlide.setTargetPosition(0);
        frontSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void resetEncoders(){
        frontSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontSlide.setTargetPosition(0);
        backSlide.setTargetPosition(0);
        frontSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void incrementDown(){
        frontSlide.setTargetPosition(frontSlide.getCurrentPosition() - incrementAmt);
        backSlide.setTargetPosition(frontSlide.getCurrentPosition() - incrementAmt);
        frontSlide.setVelocity(velocity);
        backSlide.setVelocity(velocity);
    }

    public void incrementUp(){
        frontSlide.setTargetPosition(frontSlide.getCurrentPosition() + incrementAmt);
        backSlide.setTargetPosition(frontSlide.getCurrentPosition() + incrementAmt);
        frontSlide.setVelocity(velocity);
        backSlide.setVelocity(velocity);
    }

    public void scoreShared(){
        frontSlide.setTargetPosition(sharedHeight);
        backSlide.setTargetPosition(sharedHeight);
        frontSlide.setVelocity(velocity);
        backSlide.setVelocity(velocity);
    }
    public int getHeight(){return frontSlide.getCurrentPosition();}

    public boolean isBusy(){return frontSlide.isBusy();}

    public void extendHigh() {
        frontSlide.setTargetPosition(highGoal);
        backSlide.setTargetPosition(highGoal);
        frontSlide.setVelocity(velocity);
        backSlide.setVelocity(velocity);
    }

    public void extendMidBlue() {
        frontSlide.setTargetPosition(midGoalBlue);
        backSlide.setTargetPosition(midGoalBlue);
        frontSlide.setVelocity(velocity);
        backSlide.setVelocity(velocity);
    }

    public void extendMidRed() {
        frontSlide.setTargetPosition(midGoalRed);
        backSlide.setTargetPosition(midGoalRed);
        frontSlide.setVelocity(velocity);
        backSlide.setVelocity(velocity);
    }


    // todo make the shared hub less dump and lower
    public void extendLowPrepBlue() {
        frontSlide.setTargetPosition(preLowGoal);
        backSlide.setTargetPosition(preLowGoal);
        frontSlide.setVelocity(velocity);
        backSlide.setVelocity(velocity);
    }
    public void extendLow() {
        frontSlide.setTargetPosition(lowGoal);
        backSlide.setTargetPosition(lowGoal);
        frontSlide.setVelocity(velocity);
        backSlide.setVelocity(velocity);
    }

    public void preRetract(){
        frontSlide.setTargetPosition(prepReturnFromLowHeight);
        backSlide.setTargetPosition(prepReturnFromLowHeight);
        frontSlide.setVelocity(velocity);
        backSlide.setVelocity(velocity);
    }

    public void retract() {
        frontSlide.setTargetPosition(home);
        backSlide.setTargetPosition(home);
        frontSlide.setVelocity(velocity);
        backSlide.setVelocity(velocity);
    }
}


