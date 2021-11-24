package org.firstinspires.ftc.teamcode.RobotParts;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Duck {
    CRServo duckTop, duckBottom;

    public Duck (HardwareMap hardwareMap){
        duckTop = hardwareMap.get(CRServo.class, "duckTop");
        duckBottom = hardwareMap.get(CRServo.class, "duckBottom");
//        duckTop = hardwareMap.get(CRServoImplEx.class, "duckTop");
//        duckBottom = hardwareMap.get(CRServoImplEx.class, "duckBottom")
    }

    // the blue duck station is to your right if you stand facing the shelf
    public void setBlueSpeed(){
        duckTop.setPower(1.0);
        duckBottom.setPower(1.0);
    }

    public void setRedSpeed(){
        duckTop.setPower(-1.0);
        duckBottom.setPower(-1.0);
    }

}
