package org.firstinspires.ftc.teamcode.Hardware.Outtake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Linkages {
    Servo leftExtender, rightExtender;

    public static int toggleIndex = 0;
    public final static int MID = 0, CLOSE = 1, FAR = 2;

    public static double incrementAmt = 0.005;

    // REMEMBER Ins is 1.0 (big) and Out is 0 (small)
    // Out Small || In Big
    public static double fullOut = 0.18,
                              in = 1.0,
// vikram
                            closeShared = 0.6,
                            midShared = 0.46,
                            farShared = 0.3,

                            lowAuto = 0.25;

    // DO NOT change the order, turret uses the toggle index to find the position
    public static double[] sharedHubPoses = {midShared, closeShared, farShared};

    public Linkages(HardwareMap hardwareMap) {
        rightExtender = hardwareMap.get(Servo.class, "RightExtender");
        leftExtender = hardwareMap.get(Servo.class, "LeftExtender");
//        leftExtender.setDirection(Servo.Direction.REVERSE);
    }

    public void extend(){
        leftExtender.setPosition(fullOut);
        rightExtender.setPosition(fullOut);
    }

    public void extendLowAuto(){
        leftExtender.setPosition(lowAuto);
        rightExtender.setPosition(lowAuto);
    }

    public void retract(){
        leftExtender.setPosition(in);
        rightExtender.setPosition(in);
    }

    public void extendSharedMid(){
        leftExtender.setPosition(midShared);
        rightExtender.setPosition(midShared);
    }
    public void extendShareFar(){
        leftExtender.setPosition(closeShared);
        rightExtender.setPosition(closeShared);
    }
    public void extendShareClose(){
        leftExtender.setPosition(farShared);
        rightExtender.setPosition(farShared);
    }

    public void toggle(){
        // see if it is at the out position
//        if (rightExtender.getPosition() - rightExtenderSharedOutPos < 0.05){
//            toggleIndex = 1;
//        }
        toggleIndex++;
        toggleIndex %= 3;
        extendToToggle();
    }

    public void extendToToggle(){
        leftExtender.setPosition(sharedHubPoses[toggleIndex]);
        rightExtender.setPosition(sharedHubPoses[toggleIndex]);
    }

    public int getToggleIndex() {return toggleIndex;}

    // deprecated
    public void increment(){
        leftExtender.setPosition(leftExtender.getPosition()+incrementAmt);
        rightExtender.setPosition(rightExtender.getPosition()+incrementAmt);
    }
    public void decrement(){
        leftExtender.setPosition(leftExtender.getPosition()-incrementAmt);
        rightExtender.setPosition(rightExtender.getPosition()-incrementAmt);
    }
}



