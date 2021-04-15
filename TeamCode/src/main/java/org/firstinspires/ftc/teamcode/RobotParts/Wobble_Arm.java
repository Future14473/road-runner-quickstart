package org.firstinspires.ftc.teamcode.RobotParts;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.ourOpModes.resources.Timing;

public class Wobble_Arm {
    public Servo angler;
    Servo gripper;
    Timing timer;
    public Boolean isGrabbing = false; //todo make not public

    public Wobble_Arm(HardwareMap hardwareMap, LinearOpMode opMode){
        angler = hardwareMap.get(Servo.class, "wobble_angler");
        angler.setDirection(Servo.Direction.REVERSE);
        gripper = hardwareMap.get(Servo.class, "wobble_gripper");
        timer = new Timing(opMode);
    }

    public void up(){ angler.setPosition(0.5); } //b button
    // 0.44 goes all the way into the robot

    public void down(){angler.setPosition(0.75);} // a button


    public void automaticReleaseWobble(){
        unGrab();
        timer.safeDelay(200);
        up();
    }

    public void autonomousInit(){
        grab();
        timer.safeDelay(200);
        up();
    }

    public void safeReleaseWobble(){
        unGrab();
        timer.safeDelay(400);
        up();
    }



    public void unGrab(){
        gripper.setPosition(1.5);
    }

    public void grab() {
        gripper.setPosition(0.51);
    }

    public double getAnglerPosition(){return angler.getPosition();}
    public double getGripperPosition(){return gripper.getPosition();}


}


