package org.firstinspires.ftc.teamcode.Hardware.Outtake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Linkages {
    Servo leftExtender, rightExtender;

    public static int toggleIndex = 0;

    public static double incrementAmt = 0.005;

    public static double fullOut = 0.18,
//                            leftFullOut = 0.5,
                              in = 1.0,
//                            leftIn = 1.0,

                            farShared = 0.2,
                            midShared = 0.4,
                            closeShared = 0.6,

                            lowAuto = 0.25;
//                            leftLowAuto = 0.3;


    public static double[] sharedHubPoses = {closeShared, midShared, farShared};

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
        leftExtender.setPosition(farShared);
        rightExtender.setPosition(farShared);
    }
    public void extendShareClose(){
        leftExtender.setPosition(closeShared);
        rightExtender.setPosition(closeShared);
    }

    public void toggle(){
        // see if it is at the out position
//        if (rightExtender.getPosition() - rightExtenderSharedOutPos < 0.05){
//            toggleIndex = 1;
//        }
        toggleIndex++;
        toggleIndex %= 3;
        leftExtender.setPosition(sharedHubPoses[toggleIndex]);
        rightExtender.setPosition(sharedHubPoses[toggleIndex]);
    }

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



