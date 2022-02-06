package org.firstinspires.ftc.teamcode.Hardware.Duck;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Duck {
    CRServo duck;


    public Duck(HardwareMap hardwareMap){
        duck = hardwareMap.get(CRServo.class, "duck");
    }

    public spinBlue
}
