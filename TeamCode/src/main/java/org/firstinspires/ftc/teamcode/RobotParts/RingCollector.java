package org.firstinspires.ftc.teamcode.RobotParts;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RingCollector {
    DcMotor intake1,intake2, taco;
    public RingCollector(HardwareMap hardwareMap){
        intake1 = hardwareMap.get(DcMotor.class, "intake");
        intake2 = hardwareMap.get(DcMotor.class, "intake2");
        intake2.setDirection(DcMotorSimple.Direction.REVERSE);
        taco = hardwareMap.get(DcMotor.class, "taco");
    }
    public void collect(double speed){
        intake1.setPower(speed);
        intake2.setPower(speed);
        taco.setPower(speed);
    }

}

