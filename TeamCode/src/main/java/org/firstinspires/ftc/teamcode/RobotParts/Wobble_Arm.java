package org.firstinspires.ftc.teamcode.RobotParts;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.ourOpModes.resources.Timing;

public class Wobble_Arm {
    public DcMotorEx angler;
    Servo gripper;
    Timing timer;
    public Boolean isGrabbing = false; //todo make not public

    public Wobble_Arm(HardwareMap hardwareMap, LinearOpMode opMode){
        angler = hardwareMap.get(DcMotorEx.class, "wobble_angler");
        //angler.setTargetPosition(0);
        //angler.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //angler.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        gripper = hardwareMap.get(Servo.class, "wobble_gripper");
        timer = new Timing(opMode);
    }

    public void up(){ angler.setTargetPosition(0); } //b button
    // 0.44 goes all the way into the robot

    public void down(){ angler.setTargetPosition(700); } // a button

    public void automaticReleaseWobble(){
        unGrab();
        timer.execAsync(this::up, 200);
        up();
    }

    public void autonomousInit(){
        grab();
        timer.execAsync(this::up, 200);
    }

    public void safeReleaseWobble(){
        unGrab();
        timer.execAsync(this::up, 400);
    }

    public void unGrab(){
        gripper.setPosition(1.5);
    }

    public void grab() {
        gripper.setPosition(0.51);
    }

    public double getAnglerPosition(){return angler.getCurrentPosition();}
    public double getGripperPosition(){return gripper.getPosition();}


}


