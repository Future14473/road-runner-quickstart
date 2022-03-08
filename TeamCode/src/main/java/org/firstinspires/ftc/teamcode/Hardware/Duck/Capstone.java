package org.firstinspires.ftc.teamcode.Hardware.Duck;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Capstone {
    Servo capArm;
    public static double down = 0.1, collect = 0.5, up = 0.8;
    boolean isDown = true;
    public Capstone(HardwareMap hardwareMap){
        capArm = hardwareMap.get(Servo.class, "capArm");
        down();
    }

    public void toggle(){
        if(isDown){
            up();
        } else{
            down();
        }
    }
    public void down(){capArm.setPosition(down);}
    public void collect(){capArm.setPosition(collect);}
    public void up(){capArm.setPosition(up);}
}
