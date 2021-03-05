package org.firstinspires.ftc.teamcode.ourOpModes.unit_tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(group = "drive")
public class Intake_and_TacoTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor taco = hardwareMap.get(DcMotor.class, "taco");
        DcMotor intake = hardwareMap.get(DcMotor.class, "intake");

        telemetry.addData("Hold Down Gamepad 1 left stick y ", "to move intake");
        telemetry.addData("Hold Down Gamepad 1 right stick y ", "to move taco");

        waitForStart();

        while (!isStopRequested()) {
            intake.setPower(gamepad1.left_stick_y);
            taco.setPower(gamepad1.right_stick_y);
        }
    }
}