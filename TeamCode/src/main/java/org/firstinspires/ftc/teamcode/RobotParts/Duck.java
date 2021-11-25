package org.firstinspires.ftc.teamcode.RobotParts;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Duck {
    CRServo duckTop, duckBottom;
    double duckSpeed;

    public Duck (HardwareMap hardwareMap){
        duckTop = hardwareMap.get(CRServo.class, "duckTop");
        duckBottom = hardwareMap.get(CRServo.class, "duckBottom");
//        duckTop = hardwareMap.get(CRServoImplEx.class, "duckTop");
//        duckBottom = hardwareMap.get(CRServoImplEx.class, "duckBottom")
    }

    // the blue duck station is to your right if you stand facing the shelf
    public void setBlueSpeed(){
        duckSpeed = 1.0;
    }

    public void setRedSpeed(){
        duckSpeed = -1.0;
    }

    public void setStopSpeed(){
        duckSpeed = 0;
    }

    public void moveDuck(){
        duckTop.setPower(duckSpeed);
        duckBottom.setPower(duckSpeed);
    }

}
