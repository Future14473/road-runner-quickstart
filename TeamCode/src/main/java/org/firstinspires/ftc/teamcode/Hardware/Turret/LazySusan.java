package org.firstinspires.ftc.teamcode.Hardware.Turret;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LazySusan {
    DcMotorEx lazySusan;
    public LazySusan(HardwareMap hardwareMap){
        lazySusan = hardwareMap.get(DcMotorEx.class, "lazySusan");
        lazySusan.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public Integer getVelo(){
       return (int) lazySusan.getVelocity();
    }

    public void setPower(double power){
        lazySusan.setPower(power);
    }

    public void rotateToDegrees(double degrees){
        lazySusan.setTargetPosition(TurretConstants.turretDegreesToTicks(degrees));
        lazySusan.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lazySusan.setVelocity(200);
    }

    public double getDegrees(){
        return lazySusan.getCurrentPosition() * (1/TurretConstants.LAZY_SUSAN_TICKS_PER_REVOLUTION) * (1/TurretConstants.MOTOR_ROTATIONS_PER_TURRET_ROTATIONS) * 360;
    }

    public double getTargetDegrees(){
        return lazySusan.getTargetPosition() * (1/TurretConstants.LAZY_SUSAN_TICKS_PER_REVOLUTION) * (1/TurretConstants.MOTOR_ROTATIONS_PER_TURRET_ROTATIONS) * 360;
    }

    public double getTicks(){
        return lazySusan.getCurrentPosition();
    }

    public double getTargetTicks(){
        return lazySusan.getTargetPosition();
    }

    public DcMotorEx.RunMode getRunMode() {return lazySusan.getMode();}
}
