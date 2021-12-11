package org.firstinspires.ftc.teamcode.Mechanisms.HardwareTests;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Mechanisms.EncoderMecanum;

@Config
@TeleOp(group = "Hardware Tests")
public class EncoderTest extends LinearOpMode {
    public static double forwardIn = 1;
    public static double strafeIn = 0;
    public static double turnDegrees = 0;
    @Override
    public void runOpMode() throws InterruptedException {

        EncoderMecanum drivetrain = new EncoderMecanum(hardwareMap, telemetry);
        drivetrain.setMotorsToPowerMode();
        waitForStart();

        while (opModeIsActive()) {
            drivetrain.movePower(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
            drivetrain.telemetryEncoderPositions();
            telemetry.update();
        }
        drivetrain.setMotorsToEncoderMode();
                drivetrain.moveInches(3,0,0);
    };
}

