package org.firstinspires.ftc.teamcode.RobotParts;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {
    DcMotor noodles;
    Servo retracter;

    public Intake(HardwareMap hardwareMap){
        noodles = hardwareMap.get(DcMotor.class, "noodles");
        retracter = hardwareMap.get(Servo.class, "retracter");
    }
    void setNoodlePower(double power){
        noodles.setPower(power);
    }

    public void inNoodles(){
        noodles.setPower(1.0);
    }
    public void outNoodles(){
        noodles.setPower(-1.0);
    }
}
