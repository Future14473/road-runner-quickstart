package org.firstinspires.ftc.teamcode.TurretTuning;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Turret {
    DcMotorEx lazySusan;
    public Turret(HardwareMap hardwareMap){
        lazySusan = hardwareMap.get(DcMotorEx.class, "lazySusan");
        lazySusan.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }
    public Integer getLazySusanVelo(){
       return (int) lazySusan.getVelocity();
    }

    public void setLazySusanPower(double power){
        lazySusan.setPower(power);
    }

    public void rotateDegrees(double degrees){
        lazySusan.setTargetPosition(TurretConstants.turretDegreesToTicks(degrees));
        lazySusan.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lazySusan.setVelocity(200);
    }
}
