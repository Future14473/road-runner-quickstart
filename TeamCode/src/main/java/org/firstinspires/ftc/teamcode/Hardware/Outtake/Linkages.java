package org.firstinspires.ftc.teamcode.Hardware.Outtake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Linkages {
    Servo leftExtender, rightExtender;

    public static double incrementAmt = 0.003 ;
    public static double leftExtenderOutPos = 0.675;
    public static double rightExtenderOutPos = 0.25;
    public static double rightExtenderInPos = 0.75;
    //public static double dumperOutPos = 0.6;
    //public static double dumperHalfPos = 0.45;
    public static double leftExtenderInPos = 0.975;
    //public static double dumperInPos = 0.2;

    public Linkages(HardwareMap hardwareMap) {
        //dumper = hardwareMap.get(Servo.class, "dumper");
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

   /* public void flipOutDumper() { dumper.setPosition(dumperOutPos); }
    public void flipHalfDumper() {
        dumper.setPosition(dumperHalfPos);
    }
    public void flipInDumper() { dumper.setPosition(dumperInPos); }

    public void dumperIn(){
        flipInDumper();
    }
    public void dumperOut(){
        flipOutDumper();
    }
*/
    public void extend(){
        leftExtender.setPosition(leftExtenderOutPos);
        rightExtender.setPosition(rightExtenderOutPos);
    }
    public void retract(){
        leftExtender.setPosition(leftExtenderInPos);
        rightExtender.setPosition(rightExtenderInPos);
    }
}



