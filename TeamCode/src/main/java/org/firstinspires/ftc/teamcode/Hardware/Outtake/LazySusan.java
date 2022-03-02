package org.firstinspires.ftc.teamcode.Hardware.Outtake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware.util.Timer;

@Config
public class LazySusan {
    private VoltageSensor batteryVoltageSensor;
    public static int velo = 2000;
    public static int incrementAmt = 7;

    DcMotorEx lazySusan;
    Telemetry telemetry;

    public LazySusan(HardwareMap hardwareMap){
        lazySusan = hardwareMap.get(DcMotorEx.class, "lazySusan");
        lazySusan.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        lazySusan.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
    }

    public LazySusan(HardwareMap hardwareMap, Telemetry telemetry){
        lazySusan = hardwareMap.get(DcMotorEx.class, "lazySusan");
        lazySusan.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        lazySusan.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        this.telemetry = telemetry;
    }

    public Integer getVelo(){
       return (int) lazySusan.getVelocity();
    }

    public boolean isStillMoving(){
        return lazySusan.isBusy();
    }

    // see if the turret is back to the starting position
    public boolean isHome() {return Math.abs(getDegrees()) < 3;}

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

    // todo wrapping issue
//    public void rotateToDegreesRobotCentric(double degrees){
//        if ((degrees - getDegrees() > 180) || (degrees - getDegrees() < 0)) {
//            lazySusan.setTargetPosition(TurretConstants.turretDegreesToTicks(degrees-360));
//        } else {
//            lazySusan.setTargetPosition(TurretConstants.turretDegreesToTicks(degrees));
//        }
////        lazySusan.setTargetPosition(TurretConstants.turretDegreesToTicks(degrees));
//
//        lazySusan.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        telemetry.addData("current", getDegrees());
//        telemetry.addData("target", degrees);
//        telemetry.addData("Velo before check degrees - current < 180", velo);
////        if (degrees - getDegrees() > 0){
////            velo = -velo;
////        }
//        telemetry.addData("Corrected Velo" , velo);
//
//        lazySusan.setVelocity(velo);
//
//    }

    public void rotateToAbsolutePos(double degrees){
        lazySusan.setTargetPosition(TurretConstants.turretDegreesToTicks(degrees));
        lazySusan.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lazySusan.setVelocity(velo);
    }

    public void incrementPos(double degrees){
        lazySusan.setTargetPosition(TurretConstants.turretDegreesToTicks(getAbsoluteDegrees() + degrees));
        lazySusan.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lazySusan.setVelocity(velo);
    }

    public double[] rotateToDegreesRobotCentric(double degrees){
        double option1 = degrees;
        double option2;
        if(degrees < 0) option2 = degrees + 360;
        else option2 = degrees - 360;

        if(Math.abs(getDegrees() - option1) > Math.abs(getDegrees() - option2)){
            incrementPos(- getDegrees() + option2);
        } else {
            incrementPos(- getDegrees() + option1);
        }
        double[] options = {option1, option2};
        return options;
    }

    public void setZero(Timer timer){
        lazySusan.setVelocity(100);
        timer.safeDelay(1000);
        lazySusan.setVelocity(0); // stop
        // zero the turret
        lazySusan.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void turnRightIncrement(){
        rotateToDegreesRobotCentric(this.getDegrees() + incrementAmt);
    }
    public void turnLeftIncrement(){
        rotateToDegreesRobotCentric(this.getDegrees() - incrementAmt);
    }

    public double getDegrees(){
        // get the position from raw ticks to raw degrees (can end up over 360degrees or under -360)
        double pos = lazySusan.getCurrentPosition() * (1/TurretConstants.LAZY_SUSAN_TICKS_PER_REVOLUTION) * (1/TurretConstants.MOTOR_ROTATIONS_PER_TURRET_ROTATIONS) * 360;
        pos %= 360;
//        return pos + ((pos)<0 ? 360 : 0);

        //return pos < 0 ? pos + 360 : pos;
        return pos;
    }

    public double getAbsoluteDegrees(){
        // get the position from raw ticks to raw degrees (can end up over 360degrees or under -360)
        double pos = lazySusan.getCurrentPosition() * (1/TurretConstants.LAZY_SUSAN_TICKS_PER_REVOLUTION) * (1/TurretConstants.MOTOR_ROTATIONS_PER_TURRET_ROTATIONS) * 360;
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
