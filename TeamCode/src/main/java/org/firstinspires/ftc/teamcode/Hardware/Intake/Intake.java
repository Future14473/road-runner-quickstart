package org.firstinspires.ftc.teamcode.Hardware.Intake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Intake {
    DcMotor intake;
    Servo dropDown;
    public static double dropPos = 1.0, upPos = 0.0;

    public Intake(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotor.class, "intake");
        dropDown = hardwareMap.get(Servo.class, "dropDown");

        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        dropDown.setDirection(Servo.Direction.REVERSE);

        drop();
    }
    public void in(){
        intake.setPower(1.0);
    }

    public void out(){
        intake.setPower(-1.0);
    }

    public void stop() {intake.setPower(0);}

    public void drop() {dropDown.setPosition(dropPos);}

    public void up() {dropDown.setPosition(upPos);}
}
