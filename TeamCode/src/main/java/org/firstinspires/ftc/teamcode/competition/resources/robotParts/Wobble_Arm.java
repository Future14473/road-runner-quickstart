package org.firstinspires.ftc.teamcode.competition.resources.robotParts;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Wobble_Arm {
    public Servo angler;
    public Servo gripper;

    public double downPos = 0.41;
    public double upPos = 0.6;

    public double grabPos = 0.4;
    public double unGrabPos = 0.8;

    public Wobble_Arm(HardwareMap hardwareMap){
         angler = hardwareMap.get(Servo.class, "wobble_angler");
         gripper = hardwareMap.get(Servo.class, "wobble_gripper");
    }

    public void down(){
        angler.setPosition(downPos);
    }

    public void up(){
        angler.setPosition(upPos);
    }

    public void grab(){
        gripper.setPosition(grabPos);
    }
    public void unGrab(){
        gripper.setPosition(unGrabPos);
    }

    //checks if the servo is done moving to a position
    public boolean isDone(double position, Servo daServo){
        return (daServo.getPosition() - position) < 0.1;
    }
}
