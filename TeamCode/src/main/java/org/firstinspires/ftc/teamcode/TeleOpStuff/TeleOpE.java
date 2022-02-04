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
    public void runOpMode() throws InterruptedException{
        DT driveTrain = new DT(hardwareMap, this);

        waitForStart();

        while(opModeIsActive()){

        }
    }

    private void updateDT(){

    }
}

