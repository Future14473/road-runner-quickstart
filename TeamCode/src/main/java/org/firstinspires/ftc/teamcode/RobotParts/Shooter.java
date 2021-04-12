package org.firstinspires.ftc.teamcode.RobotParts;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Shooter {
    DcMotorEx shooter_motor;
    int powerShotSpeed = 1670, highGoalSpeed = 1670;
    int tarVelocity = highGoalSpeed;

    public Shooter(HardwareMap hardwareMap){
        shooter_motor = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter_motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
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
        shooter_motor.setVelocity(0);
    }
}