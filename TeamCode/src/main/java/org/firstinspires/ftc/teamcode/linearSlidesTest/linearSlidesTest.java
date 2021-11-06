package org.firstinspires.ftc.teamcode.linearSlidesTest;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(group = "linearSlidesTest")

public class linearSlidesTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx linearSlide = hardwareMap.get(DcMotorEx.class, "linearSlide");
        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        int linearSlidePosition;
        waitForStart();
        linearSlide.setPower(0.2);
        while (opModeIsActive()) {
            linearSlidePosition = linearSlide.getCurrentPosition();
            if (gamepad1.dpad_up) {
                linearSlide.setTargetPosition(linearSlidePosition + 20);
            }
            if (gamepad1.dpad_down) {
                linearSlide.setTargetPosition(linearSlidePosition - 20);
            }
            telemetry.addData("Linear Slide Position", linearSlidePosition);
            telemetry.update();
        }
    }
}