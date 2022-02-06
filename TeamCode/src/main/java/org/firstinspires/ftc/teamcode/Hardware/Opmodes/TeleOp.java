package org.firstinspires.ftc.teamcode.Hardware.Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.Intake.Intake;
import org.firstinspires.ftc.teamcode.Hardware.Outtake.Outtake;
import org.firstinspires.ftc.teamcode.Hardware.Outtake.Slides;

import org.firstinspires.ftc.teamcode.Hardware.Outtake.Linkages;
import org.firstinspires.ftc.teamcode.Hardware.Turret.LazySusan;
import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Outtake outtake = new Outtake(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        LazySusan lazySusan = new LazySusan(hardwareMap);
        SampleTankDrive tankDrive = new SampleTankDrive(hardwareMap);
        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.dpad_up) {
                outtake.slides.extendHigh();
            }
            if (gamepad1.dpad_down) {
                outtake.slides.retract();
            }
            if (gamepad1.dpad_right) {
                outtake.linkages.extend();
            }
            if (gamepad1.dpad_left) { // make retract and down all in one method later
                outtake.linkages.retract();
            }
            if (gamepad1.x) {
                outtake.linkages.DumperOut();
            }
            if (gamepad1.y) {
                outtake.linkages.DumperIn();
            }
            if(gamepad1.b){
                lazySusan.rotateToDegrees(90);
            }
            if(gamepad1.a){
                lazySusan.rotateToDegrees(-90);
            }
            if(gamepad1.right_bumper){
                lazySusan.rotateToDegrees(0);
            }

           if (gamepad1.right_trigger > 0){
                intake.in();
            } else if (gamepad1.left_trigger > 0){
                intake.out();
            } else{
                intake.stop();
            }

           if (gamepad1.left_stick_button){
               intake.up();
           }
           if (gamepad1.left_bumper){
                intake.drop();
           }

           tankDrive.setPowerDir(gamepad1.left_stick_y, gamepad1.left_stick_x);
        }
    }
}
