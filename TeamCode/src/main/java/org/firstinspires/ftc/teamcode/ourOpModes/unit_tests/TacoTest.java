package org.firstinspires.ftc.teamcode.ourOpModes.unit_tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(group = "drive")
public class TacoTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor taco = hardwareMap.get(DcMotor.class, "taco");

        telemetry.addData("Hold Down Gamepad 1 right stick y ", "to move");
        telemetry.update();
        waitForStart();

        while (!isStopRequested()) {
            taco.setPower(gamepad1.right_stick_y);
        }
    }
}