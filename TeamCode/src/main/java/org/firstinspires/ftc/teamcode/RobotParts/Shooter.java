package org.firstinspires.ftc.teamcode.RobotParts;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.ourOpModes.DirtyGlobalVariables;

public class Shooter {
    DcMotorEx shooter_motor;
    int powerShotSpeed = 1350, highGoalSpeed = 1650;
    int tarVelocity = highGoalSpeed;

    public Shooter(HardwareMap hardwareMap){
        shooter_motor = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter_motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        Log.e("PIDF Shooter vals", String.valueOf(shooter_motor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER)));
        shooter_motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(10,3,4,0));
    }

    public double getShooterVelocity(){
        return shooter_motor.getVelocity();
    }

    public int getTargetVelocity(){ return tarVelocity;}

    public void setHighGoalSpeed(){
        tarVelocity = highGoalSpeed;
    }

    public void setPowerShotSpeed(){
        tarVelocity = powerShotSpeed;
    }

    //for faster tuning
    public void increaseSpeed(){
        tarVelocity += 10;
    }

    public void decreaseSpeed(){
        tarVelocity -= 10;
    }

    public void setSpeed(){
        shooter_motor.setVelocity(tarVelocity);
    }
    public void stop(){
        tarVelocity = 0;
    }
}