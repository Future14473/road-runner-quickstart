package org.firstinspires.ftc.teamcode.Hardware.Intake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    DcMotor intake;

    public Intake(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotor.class, "intake");
    }
    public void in(){
        intake.setPower(1.0);
    }

    public void out(){
        intake.setPower(-1.0);
    }

    public void stop() {intake.setPower(0);}
}
