package org.firstinspires.ftc.teamcode.TurretTuning;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    DcMotor noodles;
    double powerIn = 1.0;
    double powerOut = -1.0;


    public Intake(HardwareMap hardwareMap) {
        noodles = hardwareMap.get(DcMotor.class, "noodles");
        noodles.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setNoodlePower(double power) {
        noodles.setPower(power);
    }

    public void inNoodles() {
        noodles.setPower(1.0);
    }

    public void outNoodles() {
        noodles.setPower(-1.0);
    }

    public void stopNoodles() {
        noodles.setPower(0);
    }

    public void slideOutInNoodles() {
        inNoodles();
    }

    public void slideInOutNoodles() {
        stopNoodles();
    }
}
