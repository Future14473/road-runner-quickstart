package org.firstinspires.ftc.teamcode.ourOpModes.robotParts;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Wobble_Arm {
    Servo angler;
    Servo gripper;

    public Wobble_Arm(HardwareMap hardwareMap){
         angler = hardwareMap.get(Servo.class, "wobble_angler");
         gripper = hardwareMap.get(Servo.class, "wobble_gripper");
    }

    public void down(){
        angler.setPosition(0.15);
    }

    public void releaseWobble(){
        gripper.setPosition(1.5);
        //move back up
        angler.setPosition(0.5);
    }

    public void grab(){
        gripper.setPosition(0.3);
    }

}

