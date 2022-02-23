package org.firstinspires.ftc.teamcode.Hardware.Outtake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Linkages {
    Servo leftExtender, rightExtender;

    public static double incrementAmt = 0.005;
    public static double leftExtenderOutPos = 0.675;
    public static double rightExtenderOutPos = 0.18;
    public static double rightExtenderInPos = 0.75;
    public static double leftExtenderInPos = 0.975;

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
}



