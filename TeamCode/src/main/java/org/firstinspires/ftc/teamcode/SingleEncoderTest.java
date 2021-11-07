package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(group = "linearSlidesTest")
@Config
public class SingleEncoderTest extends LinearOpMode {
    public static int velocity = 1220;
    public static int deltaMovement = 100;
    DcMotorEx linearSlide;
    DcMotorEx linearSlide2;
    @Override
    public void runOpMode() throws InterruptedException {

        linearSlide = hardwareMap.get(DcMotorEx.class, "linearSlide");
        linearSlide2 = hardwareMap.get(DcMotorEx.class, "linearSlide2");
//        DcMotorEx linearSlide2 = hardwareMap.get(DcMotorEx.class, "linearSlide2");


        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //linearSlide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //weird thing since it is giving an error for needing to set a position before you start
//        linearSlide.setTargetPosition(0);
//        linearSlide.setTargetPosition(0);

//        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        linearSlide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        telemetry.addData("Current Position", linearSlide.getCurrentPosition());
        telemetry.addData("Current Position", linearSlide2.getCurrentPosition());
        telemetry.update();
        waitForStart();
        linearSlide.setTargetPosition(-287);
        linearSlide2.setTargetPosition(-290);
        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlide.setVelocity(200);
        linearSlide2.setVelocity(200);
        while(opModeIsActive()){
            telemetry.addData("velocity", linearSlide.getVelocity());
            telemetry.addData("position", linearSlide.getCurrentPosition());
            telemetry.addData("is at target", !linearSlide.isBusy());
            telemetry.addData("velocity", linearSlide2.getVelocity());
            telemetry.addData("position", linearSlide2.getCurrentPosition());
            telemetry.addData("is at target", !linearSlide2.isBusy());
            telemetry.update();

        }



//            linearSlide.setVelocity(0);

        }
    }
