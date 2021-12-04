package org.firstinspires.ftc.teamcode.RobotParts;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Duck {
    CRServo duck;
    double duckPower = 0;
    public Duck (HardwareMap hardwareMap){
        duck = hardwareMap.get(CRServo.class, "duck");
    }

    // the blue duck station is to your right if you stand facing the shelf
    public void setBlueSpeed(){
        duckPower = 1.0;
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
