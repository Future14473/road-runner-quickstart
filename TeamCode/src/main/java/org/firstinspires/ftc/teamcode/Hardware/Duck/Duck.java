package org.firstinspires.ftc.teamcode.Hardware.Duck;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Hardware.util.Timer;

@Config
public class Duck {
    CRServo duck;
    double duckPower = 0;
    public static double tempspeed = 0.55;
    public static long autoDelayTime = 2500;



    public Duck (HardwareMap hardwareMap){
        duck = hardwareMap.get(CRServo.class, "duck");
    }

    // the blue duck station is to your right if you stand facing the shelf
    public void setDuckPowerVar(double speed){
        duckPower = -speed;
    }

    public void setBlue(double speed){
        duck.setPower(speed);
    }

    public void setRed(double speed){
        duck.setPower(-speed);
    }

    public void setStop(){
        duckPower = 0;
    }


    }

