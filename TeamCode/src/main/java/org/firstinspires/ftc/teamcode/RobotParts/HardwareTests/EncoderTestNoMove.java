package org.firstinspires.ftc.teamcode.RobotParts.HardwareTests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotParts.EncoderMecanum;

@Config
@TeleOp (group = "!")

public class EncoderTestNoMove extends LinearOpMode {
    public static double forwardIn = 1;
    public static double strafeIn = 0;
    public static double turnDegrees = 0;
    @Override
    public void runOpMode() throws InterruptedException {

        EncoderMecanum drivetrain = new EncoderMecanum(hardwareMap, telemetry);
        waitForStart();
//        drivetrain.moveInches(forwardIn,strafeIn,turnDegrees);
        while (opModeIsActive()) {
            drivetrain.telemetryEncoderPositions();
            telemetry.update();
        }
    }
}

