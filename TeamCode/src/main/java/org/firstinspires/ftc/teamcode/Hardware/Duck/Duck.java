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

    public void autoDuckBlue(Timer timer){
        duck.setPower(0.7);
        timer.safeDelay(1000);
        duck.setPower(0.2);
        timer.safeDelay(3300);
    }
    public void autoDuckRed(Timer timer){
        duck.setPower(-0.7);
        timer.safeDelay(1000);
        duck.setPower(-0.2);
        timer.safeDelay(3300);
    }

    public void setPower(double pow){duck.setPower(pow);}

    public void setBlue(){
        duckPower = 1.0;
    }

    public void setRed(){
        duckPower = -1.0;
    }

    public void setStop(){
        duckPower = 0;
    }

    public void move(){
        duck.setPower(duckPower);
    }


    }

