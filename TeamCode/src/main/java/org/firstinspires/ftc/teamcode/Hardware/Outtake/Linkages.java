package org.firstinspires.ftc.teamcode.Hardware.Outtake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Linkages {
    Servo leftExtender, rightExtender, dumper;

    public static double leftExtenderOutPos = 0.35;
    public static double rightExtenderOutPos = 0.35;
    public static double dumperOutPos = 0.35;
    public static double dumperHalfPos = 0.17;
    public static double leftExtenderInPos = 0.35;
    public static double rightExtenderInPos = 0.35;
    public static double dumperInPos = 0.35;

    public Linkages(HardwareMap hardwareMap) {
        dumper = hardwareMap.get(Servo.class, "dumper");
        rightExtender = hardwareMap.get(Servo.class, "RightExtender");
        leftExtender = hardwareMap.get(Servo.class, "LeftExtender");

    }

    public void flipOutDumper() { dumper.setPosition(dumperOutPos); }
    public void flipHalfDumper() {
        dumper.setPosition(dumperHalfPos);
    }
    public void flipInDumper() { dumper.setPosition(dumperInPos); }

    public void extendLeftExtender() {
        dumper.setPosition(leftExtenderOutPos);
    }
    public void extendRightExtender() {
        dumper.setPosition(rightExtenderOutPos);
    }
    public void retractLeftExtender() { dumper.setPosition(leftExtenderInPos); }
    public void retractRightExtender() { dumper.setPosition(rightExtenderInPos); }

    public void DumperIn(){
        flipInDumper();
    }
    public void DumperOut(){
        flipOutDumper();
    }
    public void DumperHalf(){

        flipHalfDumper();
    }

    public void extend(){
        extendLeftExtender();
        extendRightExtender();
    }
    public void retract(){
        retractLeftExtender();
        retractRightExtender();
    }
}



