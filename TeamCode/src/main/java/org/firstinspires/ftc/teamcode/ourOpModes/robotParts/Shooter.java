package org.firstinspires.ftc.teamcode.ourOpModes.robotParts;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Shooter {
    DcMotorEx shooter_motor, shooter_encoder;
    int powerShotSpeed = 1400, highGoalSpeed = 1600;

    public Shooter(HardwareMap hardwareMap){
        shooter_motor = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter_encoder = hardwareMap.get(DcMotorEx.class, "backLeft");
    }

    public double getShooterVelocity(){
        return shooter_encoder.getVelocity();
    }

    public void setHighGoalSpeed(){
        if(shooter_encoder.getVelocity() < highGoalSpeed)
        {
            shooter_motor.setVelocity(200); //todo check if this actually works
        }
    }

    public void setPowerShotSpeed(){
        if(shooter_encoder.getVelocity() < powerShotSpeed)
        {
            shooter_motor.setVelocity(200); //todo check if this actually works
        }
    }
}
