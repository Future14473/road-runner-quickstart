package org.firstinspires.ftc.teamcode.TurretTuning;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Turret {
    DcMotorEx lazySusan;
    public Turret(HardwareMap hardwareMap){
        lazySusan = hardwareMap.get(DcMotorEx.class, "lazySusan");
    }
    public Integer getLazySusanVelo(){
       return (int) lazySusan.getVelocity();
    }

    public void setLazySusanPower(double power){
        lazySusan.setPower(power);
    }
}
