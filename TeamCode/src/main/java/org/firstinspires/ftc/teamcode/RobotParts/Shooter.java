package org.firstinspires.ftc.teamcode.RobotParts;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.ourOpModes.DirtyGlobalVariables;

@Config
public class Shooter {
    DcMotorEx shooter_motor;

    public static int powerShotSpeed = 1350, highGoalSpeed = 1510, highGoalSpeedTeleop = 1550;
    int tarVelocity = highGoalSpeed;

    public static PIDFCoefficients pidf = new PIDFCoefficients(10, 3,6, 0);

    public Shooter(HardwareMap hardwareMap){
        shooter_motor = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter_motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        Log.e("PIDF Shooter vals", String.valueOf(shooter_motor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER)));

        shooter_motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
    }

    public double getShooterVelocity(){
        return shooter_motor.getVelocity();
    }

    public int getTargetVelocity(){ return tarVelocity;}

    public void setHighGoalSpeed(){
        tarVelocity = highGoalSpeed;
    }

    public void setHighGoalSpeedTeleop(){
        tarVelocity = highGoalSpeedTeleop;
    }

    public void setPowerShotSpeed(){
        tarVelocity = powerShotSpeed;
    }

    //for faster tuning
    public void increaseSpeed(){
        tarVelocity += 700;
    }

    public void decreaseSpeed(){
        tarVelocity -= 700;
    }

    public void setSpeed(){
        shooter_motor.setVelocity(tarVelocity);
    }
    public void stop(){
        tarVelocity = 0;
    }

    public void stopHard(){
        shooter_motor.setVelocity(0);
    }
}