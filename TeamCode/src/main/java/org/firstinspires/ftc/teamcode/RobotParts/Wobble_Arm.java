package org.firstinspires.ftc.teamcode.RobotParts;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.ourOpModes.resources.Timing;

@Config
public class Wobble_Arm {
//    DcMotorEx angler;
    Servo gripper;
    Timing timer;

    public static int upPos = -190, downPos = -360;
    public static double grabPos = 0.2, unGrabPos = 1;

    public Wobble_Arm(HardwareMap hardwareMap, LinearOpMode opMode){
//        angler = hardwareMap.get(DcMotorEx.class, "wobble_angler");
//        angler.setTargetPosition(0);
//        angler.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        angler.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(15, 3, 6, 0));
//        angler.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        gripper = hardwareMap.get(Servo.class, "wobble_gripper");
        timer = new Timing(opMode);
    }

    public void up(){
//        angler.setTargetPosition(upPos);
//        angler.setPower(0.6);
    }
    // 0.44 goes all the way into the robot

    public void home(){
//        angler.setTargetPosition(-50);
//        angler.setPower(0.6);
    }

    public void down(){
//        angler.setTargetPosition(downPos);
//        angler.setPower(0.6);
    }

    public void automaticReleaseWobble(){
        unGrab();
        timer.execAsync(this::up, 200);
        up();
    }

    public void unGrab(){
        gripper.setPosition(unGrabPos);
    }

    public void grab() {
        gripper.setPosition(grabPos);
    }

//    public double getAnglerPosition(){return angler.getCurrentPosition();}

    public double getGripperPosition(){return gripper.getPosition();}

}


