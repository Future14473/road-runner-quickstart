package org.firstinspires.ftc.teamcode.RobotParts;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
@Config
public class Capstone {
    Servo capstoneTop;
    Servo capstoneDown;
    public static double grabPos = 1;
    public static double placePos = 0;

    public Capstone(HardwareMap hardwareMap){
        capstoneTop = hardwareMap.get(Servo.class, "castoneTop");
        capstoneDown = hardwareMap.get(Servo.class, "capstoneDown");
    }
    public void grab(){
        capstoneDown.setPosition(grabPos);
        capstoneTop.setPosition(grabPos);
    }
    public void place(){
        capstoneDown.setPosition(placePos);
        capstoneTop.setPosition(placePos);
    }

    }


