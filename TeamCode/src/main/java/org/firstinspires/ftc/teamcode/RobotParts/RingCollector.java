package org.firstinspires.ftc.teamcode.ourOpModes.robotParts;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RingCollector {
    DcMotor intake, taco;
    public RingCollector(HardwareMap hardwareMap){
        intake = hardwareMap.get(DcMotor.class, "intake");
        taco = hardwareMap.get(DcMotor.class, "taco");
    }
    public void collect(){
        intake.setPower(1);
        taco.setPower(1);
    }
    public void out(){
        intake.setPower(-1);
        taco.setPower(-1);
    }

    public void sleep(){
        intake.setPower(0);
        taco.setPower(0);
    }
}

