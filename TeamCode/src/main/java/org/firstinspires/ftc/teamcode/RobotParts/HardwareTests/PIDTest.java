package org.firstinspires.ftc.teamcode.RobotParts.HardwareTests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.RobotParts.EncoderMecanum;

@Config
@TeleOp (group = "aaa")

public class PIDTest extends LinearOpMode {
    public static double forwardIn = 1;
    public static double strafeIn = 0;
    public static double turnDegrees = 0;
    @Override
    public void runOpMode() throws InterruptedException {

        EncoderMecanum drivetrain = new EncoderMecanum(hardwareMap, telemetry);
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("PIDF - lF",drivetrain.leftFront.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION));
            telemetry.addData("PIDF - lR",drivetrain.leftRear.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION));
            telemetry.addData("PIDF - rF",drivetrain.rightFront.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION));
            telemetry.addData("PIDF - rR",drivetrain.rightRear.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION));
            telemetry.update();
        }
    }
}

