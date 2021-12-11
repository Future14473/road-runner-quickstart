package org.firstinspires.ftc.teamcode.Mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Duck {
    CRServo duck;
    double duckPower = 0;
    public static double tempspeed = 0.65;
    public static long autoDelayTime = 2500;
    public Duck (HardwareMap hardwareMap){
        duck = hardwareMap.get(CRServo.class, "duck");
    }

    // the blue duck station is to your right if you stand facing the shelf

    public void setDuckPowerVar(double speed){
        duckPower = -speed;
    }

    public void setBlueSpeed(){
        duckPower = tempspeed;
    }
    public void setAutoSpeed(){
        duckPower = tempspeed;
    }

    public void setRedSpeed(){
        duckPower = -1.0;
    }

    public void setStopSpeed(){
        duckPower = 0;
    }

    public void setSpeed(){
        duck.setPower(duckPower);
    }

}
