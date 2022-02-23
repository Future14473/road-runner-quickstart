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
                            rightExtenderInPos = 0.75,

                            rightExtenderSharedOutPos = 0.3,
                            rightHalfOutPos = 0.465,
                            rightCloseOutPos = 0.6;


    public static double[] sharedHubPoses = {rightExtenderSharedOutPos, rightHalfOutPos, rightCloseOutPos};

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
    public void retract(){
        leftExtender.setPosition(leftExtenderInPos);
        rightExtender.setPosition(rightExtenderInPos);
    }

    public void extendShared(){
        rightExtender.setPosition(rightExtenderSharedOutPos);
    }

    public void toggle(){
        // see if it is at the out position
        if (rightExtender.getPosition() - rightExtenderSharedOutPos < 0.05){
            toggleIndex = 0;
        }
        toggleIndex++;
        toggleIndex %= 3;
        rightExtender.setPosition(sharedHubPoses[toggleIndex]);
    }
}



