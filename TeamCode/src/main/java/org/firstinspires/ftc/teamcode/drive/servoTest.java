package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Config
@Disabled
@Autonomous(group = "!")
public class servoTest extends LinearOpMode {
    public static double movement = 0.0;




    @Override
    public void runOpMode(){

        Servo servo1 = hardwareMap.get(Servo.class, "servo1");

        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive()){



            servo1.setPosition(movement);
        }
    }
}
