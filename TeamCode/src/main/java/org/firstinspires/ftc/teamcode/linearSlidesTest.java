//package org.firstinspires.ftc.teamcode;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//
//@TeleOp(group = "linearSlidesTest")
//@Config
//public class linearSlidesTest extends LinearOpMode {
//    public static int velocity = 1220;
//    public static int deltaMovement = 100;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        DcMotorEx linearSlide = hardwareMap.get(DcMotorEx.class, "linearSlide");
//        DcMotorEx linearSlide2 = hardwareMap.get(DcMotorEx.class, "linearSlide2");
//
//
//        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        linearSlide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        //weird thing since it is giving an error for needing to set a position before you start
////        linearSlide.setTargetPosition(0);
////        linearSlide.setTargetPosition(0);
//
////        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////        linearSlide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        int linearSlidePosition = 0;
//        waitForStart();
////        linearSlide.setPower(0.2);
//        while (opModeIsActive()) {
//            linearSlidePosition = linearSlide.getCurrentPosition();
//            if (gamepad1.dpad_up) {
//                telemetry.addData("Info", "Dpad Up");
//                linearSlide.setVelocity(velocity);
//                linearSlide2.setVelocity(velocity);
//                linearSlidePosition+=deltaMovement;
//                linearSlide2.setTargetPosition(linearSlidePosition);
//                linearSlide.setTargetPosition(linearSlidePosition);
//            }
//            if (gamepad1.dpad_down) {
//                telemetry.addData("Info", "Dpad Down");
//                linearSlide.setVelocity(velocity);
//                linearSlide2.setVelocity(velocity);
//                linearSlidePosition-=deltaMovement;
//                linearSlide2.setTargetPosition(linearSlidePosition);
//                linearSlide.setTargetPosition(linearSlidePosition);
//            }
//
//
//            linearSlide.setVelocity(0);
//            telemetry.addData("Out of the if statements, ", "asdf");
//            telemetry.addData("Linear Slide Velocity", linearSlide.getVelocity());
//            telemetry.addData("Linear Slide Position", linearSlidePosition);
//            telemetry.update();
//        }
//    }
//}