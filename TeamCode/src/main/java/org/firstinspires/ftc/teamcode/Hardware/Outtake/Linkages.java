package org.firstinspires.ftc.teamcode.Hardware.Outtake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Linkages {
    Servo leftExtender, rightExtender;

    int toggleIndex = 0;

    public static double incrementAmt = 0.005;
    public static double leftExtenderOutPos = 0.675;
    public static double leftExtenderInPos = 0.975;

    public static double rightExtenderOutPos = 0.18;
    public static double rightExtenderInPos = 0.75,
                            rightHalfOutPos = 0.465,
                            rightCloseOutPos = 0.27;


    public static double[] extenderPoses = {rightExtenderOutPos, rightHalfOutPos, rightCloseOutPos};

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

    public void toggle(){
        // see if it is at the out position
        if (rightExtender.getPosition() - rightExtenderOutPos < 0.05){
            toggleIndex = 0;
        }
        toggleIndex++;
        rightExtender.setPosition(extenderPoses[toggleIndex]);
    }
}



