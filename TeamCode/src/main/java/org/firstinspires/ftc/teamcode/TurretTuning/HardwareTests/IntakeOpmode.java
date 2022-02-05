package org.firstinspires.ftc.teamcode.TurretTuning.HardwareTests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.TurretTuning.Intake;


@TeleOp(group = "1 Teleop")
    public class IntakeOpmode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Intake intake = new Intake(hardwareMap);
        waitForStart();
        while (opModeIsActive()){
            intake.setNoodlePower(gamepad1.right_trigger);
            intake.setNoodlePower(-gamepad1.left_trigger);
            if (gamepad1.right_trigger > 0 && gamepad1.left_trigger > 0) {
                intake.setNoodlePower(0);
            }
            
//            if (gamepad1.right_trigger > 0) {
//                intake.inNoodles();
//            } if (gamepad1.left_trigger > 0) {
//                intake.outNoodles();
//            } if (gamepad1.right_trigger == 0 && gamepad1.left_trigger == 0) {
//                intake.stopNoodles();
            }


    }
}