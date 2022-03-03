package org.firstinspires.ftc.teamcode.Hardware.Outtake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Linkages {
    Servo leftExtender, rightExtender;

    public static int toggleIndex = 0;

    public static double incrementAmt = 0.005;
    public static double leftExtenderOutPos = 0.675;
    public static double leftExtenderInPos = 0.975;

    public static double rightExtenderOutPos = 0.18,
                            rightExtenderInPos = 0.7,

                            rightFarShared = 0.35,
                            rightMidShared = 0.4,
                            rightCloseOutPos = 0.6,
                            rightLowAuto = 0.43,
                            leftLowAuto = 0.3;


    public static double[] sharedHubPoses = {rightCloseOutPos, rightMidShared, /*rightExtenderSharedOutPos*/};

    public Linkages(HardwareMap hardwareMap) {
        rightExtender = hardwareMap.get(Servo.class, "RightExtender");
        leftExtender = hardwareMap.get(Servo.class, "LeftExtender");
        leftExtender.setDirection(Servo.Direction.REVERSE);
    }

    public void increment(){
        leftExtender.setPosition(leftExtender.getPosition()+incrementAmt);
        rightExtender.setPosition(rightExtender.getPosition()+incrementAmt);
    }
    public void decrement(){
        leftExtender.setPosition(leftExtender.getPosition()-incrementAmt);
        rightExtender.setPosition(rightExtender.getPosition()-incrementAmt);
    }

    public void extend(){
        leftExtender.setPosition(leftExtenderOutPos);
        rightExtender.setPosition(rightExtenderOutPos);
    }

    public void extendLowAuto(){
        leftExtender.setPosition(leftLowAuto);
        rightExtender.setPosition(rightLowAuto);
    }

    public void retract(){
        leftExtender.setPosition(leftExtenderInPos);
        rightExtender.setPosition(rightExtenderInPos);
    }

    public void extendSharedMid(){
        rightExtender.setPosition(rightMidShared);
    }
    public void extendShareFar(){ rightExtender.setPosition(rightFarShared);}
    public void extendShareClose(){ rightExtender.setPosition(rightCloseOutPos);}

    public void toggle(){
        // see if it is at the out position
//        if (rightExtender.getPosition() - rightExtenderSharedOutPos < 0.05){
//            toggleIndex = 1;
//        }
        toggleIndex++;
        toggleIndex %= 2;
        rightExtender.setPosition(sharedHubPoses[toggleIndex]);
    }

}



