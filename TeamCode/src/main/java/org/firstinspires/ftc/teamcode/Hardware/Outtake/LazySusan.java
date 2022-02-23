package org.firstinspires.ftc.teamcode.Hardware.Outtake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@Config
public class LazySusan {
    private VoltageSensor batteryVoltageSensor;
    public static int velo = 2000;
    public static int incrementAmt = 15;

    DcMotorEx lazySusan;

    public LazySusan(HardwareMap hardwareMap){
        lazySusan = hardwareMap.get(DcMotorEx.class, "lazySusan");
        lazySusan.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        lazySusan.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
    }
    public Integer getVelo(){
       return (int) lazySusan.getVelocity();
    }

    public boolean isStillMoving(){
        return lazySusan.isBusy();
    }

    public void setPower(double power){
        lazySusan.setPower(power);
    }
    public void setVelocity(int vel){
        lazySusan.setVelocity(vel);
    }

    public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
        PIDFCoefficients compensatedCoefficients = new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d,
                coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        );
        lazySusan.setPIDFCoefficients(runMode, compensatedCoefficients);
    }

    public void rotateToDegrees(double degrees){
        lazySusan.setTargetPosition(TurretConstants.turretDegreesToTicks(degrees));
        lazySusan.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lazySusan.setVelocity(velo);
    }

    public void turnRightIncrement(){
        rotateToDegrees(this.getDegrees() + incrementAmt);
    }
    public void turnLeftIncrement(){
        rotateToDegrees(this.getDegrees() - incrementAmt);
    }

    public double getDegrees(){
        // get the position from raw ticks to raw degrees (can end up over 360degrees or under -360)
        double pos = lazySusan.getCurrentPosition() * (1/TurretConstants.LAZY_SUSAN_TICKS_PER_REVOLUTION) * (1/TurretConstants.MOTOR_ROTATIONS_PER_TURRET_ROTATIONS) * 360;
        pos %= 360;
//        return pos + ((pos)<0 ? 360 : 0);
        return pos;
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

    public String getPIDCoef(){
        return String.valueOf(lazySusan.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION).p) + ", " +
                String.valueOf(lazySusan.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION).i) + ", " +
        String.valueOf(lazySusan.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION).d);
//        return lazySusan.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public DcMotorEx.RunMode getRunMode() {return lazySusan.getMode();}
}
