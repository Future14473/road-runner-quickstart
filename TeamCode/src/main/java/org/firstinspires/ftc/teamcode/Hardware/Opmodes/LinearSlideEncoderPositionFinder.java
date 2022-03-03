package org.firstinspires.ftc.teamcode.Hardware.Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
@Disabled
public class LinearSlideEncoderPositionFinder extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx front = hardwareMap.get(DcMotorEx.class, "frontSlide");
        DcMotorEx back = hardwareMap.get(DcMotorEx.class, "backSlide");
        front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();
        while (opModeIsActive()){

            telemetry.addData("Linear Slide Position Front", front.getCurrentPosition());
            telemetry.addData("Linear Slide Position Back", back.getCurrentPosition());
            telemetry.update();
        }
    }
}
