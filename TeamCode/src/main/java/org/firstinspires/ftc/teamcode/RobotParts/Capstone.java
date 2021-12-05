package org.firstinspires.ftc.teamcode.RobotParts;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
@Config
public class Capstone {
    Servo capstone;
    public static double downPos = 0.925;
    public static double highestLimitPos = 0;
    public static double holdUpPos = 0.4;
    public static double capPos = 0.65;
    public static double preCapPos = 0.6;

    public Capstone(HardwareMap hardwareMap){
        capstone = hardwareMap.get(Servo.class, "capstone");
    }
    public void down(){
        capstone.setPosition(downPos);
    }

    public void holdUp(){
        capstone.setPosition(holdUpPos);
    }

    public void cap(){
        capstone.setPosition(capPos);
    }

    public void highestLimit(){
        capstone.setPosition(highestLimitPos);
    }

    public void preCap(){
        capstone.setPosition(preCapPos);
    }

}


