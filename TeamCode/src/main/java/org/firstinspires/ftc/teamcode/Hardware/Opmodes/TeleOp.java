package org.firstinspires.ftc.teamcode.Hardware.Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.Intake.Intake;
import org.firstinspires.ftc.teamcode.Hardware.Outtake.Outtake;
import org.firstinspires.ftc.teamcode.Hardware.Outtake.Slides;

import org.firstinspires.ftc.teamcode.Hardware.Outtake.Linkages;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Outtake outtake = new Outtake(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.x) {
                outtake.slides.extendHigh();
            }
            if (gamepad1.a) {
                outtake.slides.retract();
            }
            if (gamepad1.y) {
                outtake.linkages.extend();
            }
            if (gamepad1.b) {
                outtake.linkages.retract();
            }
            if (gamepad1.dpad_up) {
                outtake.linkages.DumperOut();
            }
            if (gamepad1.dpad_down) {
                outtake.linkages.DumperIn();
            }

            if (gamepad1.left_trigger > 0){
                intake.out();
            }

            if (gamepad1.right_trigger > 0){
                intake.in();
            }
        }
    }
}
