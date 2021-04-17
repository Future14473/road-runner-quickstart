package org.firstinspires.ftc.teamcode.RobotParts;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.ourOpModes.resources.Timing;

public class Wobble_Arm {
    public DcMotorEx angler;
    Servo gripper;
    Timing timer;
    public Boolean isGrabbing = false; //todo make not public

    public Wobble_Arm(HardwareMap hardwareMap, LinearOpMode opMode){
        angler = hardwareMap.get(DcMotorEx.class, "wobble_angler");
        angler.setTargetPosition(0);
        angler.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        angler.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(15, 3, 0, 0));
        angler.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        //angler.setDirection(DcMotorSimple.Direction.REVERSE);


        gripper = hardwareMap.get(Servo.class, "wobble_gripper");
        timer = new Timing(opMode);
    }

    int upPos = -190, downPos = -360;
    public void up(){
//        angler.setPower(
//                0.5 * (upPos - getAnglerPosition())
//        );
        angler.setTargetPosition(upPos);
        angler.setPower(0.6);
    } //b button
    // 0.44 goes all the way into the robot

    public void down(){
//        angler.setPower(
//                0.5 * (downPos - getAnglerPosition())
//        );
        angler.setTargetPosition(downPos);
        angler.setPower(0.6);

    } // a button

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
        gripper.setPosition(1);
    }

    public void grab() {
        gripper.setPosition(0.4);
    }

    public double getAnglerPosition(){return angler.getCurrentPosition();}
    public double getGripperPosition(){return gripper.getPosition();}


}


