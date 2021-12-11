package org.firstinspires.ftc.teamcode.Mechanisms.HardwareTests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(group = "linearSlidesTest")
@Config
public class FindPositionLinearSlide extends LinearOpMode {
    public static int velocity = 1220;
    public static int deltaMovement = 100;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx linearSlide = hardwareMap.get(DcMotorEx.class, "linearSlide2");
//        DcMotorEx linearSlide2 = hardwareMap.get(DcMotorEx.class, "linearSlide2");


        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //linearSlide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //weird thing since it is giving an error for needing to set a position before you start
//        linearSlide.setTargetPosition(0);
//        linearSlide.setTargetPosition(0);

//        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        linearSlide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        telemetry.addData("Current Position", linearSlide.getCurrentPosition());
        telemetry.update();
        waitForStart();
        while (opModeIsActive()){
            telemetry.addData("Currentposition", linearSlide.getCurrentPosition());
            telemetry.update();
        }
        // -287, -290
//        linearSlide.setVelocity(1000);
//        linearSlide.setTargetPosition(1000);
//        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        while(linearSlide.isBusy())



//            linearSlide.setVelocity(0);
            telemetry.addData("Linear Slide Velocity", linearSlide.getVelocity());
//            telemetry.addData("Linear Slide Position", linearSlidePosition);
            telemetry.update();
        }
    }
