package org.firstinspires.ftc.teamcode.ourOpModes.robotParts;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ShooterFlicker {
    Servo flicker;
    public ShooterFlicker(HardwareMap hardwareMap) {
        flicker = hardwareMap.get(Servo.class, "flicker");
    }

    private void flickIn(){

    }

    public void flick(){

    }
}

