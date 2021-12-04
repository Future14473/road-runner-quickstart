package org.firstinspires.ftc.teamcode.RobotParts;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
@Config
public class Capstone {
    Servo capstone;
    public static double grabPos = 1;
    public static double placePos = 0;

    public Capstone(HardwareMap hardwareMap){
        capstone = hardwareMap.get(Servo.class, "capstone");
    }
    public void grab(){
        capstone.setPosition(grabPos);
    }
    public void place(){
        capstone.setPosition(placePos);
    }

}


