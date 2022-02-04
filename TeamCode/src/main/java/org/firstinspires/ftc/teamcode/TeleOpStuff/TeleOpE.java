package org.firstinspires.ftc.teamcode.TeleOpStuff;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.TeleOpStuff.Components.DT;

@TeleOp(name = "AAA Teleop", group = "Teleop")
public class TeleOpE extends LinearOpMode {

    DT driveTrain;

    public void runOpMode() throws InterruptedException{
        driveTrain = new DT(hardwareMap, this);

        waitForStart();

        while(opModeIsActive()){
            updateDT();
        }
    }

    private void updateDT(){
        final double SENSITIVITY;
        if(gamepad1.right_bumper) SENSITIVITY = 0.3;
        else SENSITIVITY = 1.0;
        driveTrain.setVelocity(gamepad1.right_stick_y * SENSITIVITY * driveTrain.MAX_VELOCITY,
                gamepad1.right_stick_x * SENSITIVITY * driveTrain.MAX_VELOCITY * 2 / driveTrain.DT_WIDTH);
    }
}

