package org.firstinspires.ftc.teamcode.ourOpModes.unit_tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.ourOpModes.robotParts.Mecanum;

@TeleOp(group = "drive")
public class IntakeTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor intake = hardwareMap.get(DcMotor.class, "intake");

        telemetry.addData("Hold Down Gamepad 1 right stick y ", "to move");


        waitForStart();

        while (!isStopRequested()) {
            intake.setPower(gamepad1.right_stick_y);
        }
    }
}