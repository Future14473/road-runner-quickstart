package org.firstinspires.ftc.teamcode.RobotParts;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Duck {
    CRServo duck;

    public Duck (HardwareMap hardwareMap){
        duck = hardwareMap.get(CRServo.class, "duck");
//        duckTop = hardwareMap.get(CRServoImplEx.class, "duckTop");
//        duckBottom = hardwareMap.get(CRServoImplEx.class, "duckBottom")
    }

    // the blue duck station is to your right if you stand facing the shelf
    public void setBlueSpeed(){
        duck.setPower(1.0);
    }

    public void setRedSpeed(){
        {duck.setPower(-1.0);}
    }

    public void setStopSpeed(){
        duck.setPower(0.0);
    }

}
