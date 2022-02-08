package org.firstinspires.ftc.teamcode.Hardware.Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.Duck.Duck;
import org.firstinspires.ftc.teamcode.Hardware.Intake.Intake;
import org.firstinspires.ftc.teamcode.Hardware.Outtake.Outtake;
import org.firstinspires.ftc.teamcode.Hardware.Turret.LazySusan;
import org.firstinspires.ftc.teamcode.Hardware.util.Timer;
import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class Teleop extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Outtake outtake = new Outtake(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        LazySusan lazySusan = new LazySusan(hardwareMap);
        SampleTankDrive tankDrive = new SampleTankDrive(hardwareMap);
        Duck duck = new Duck(hardwareMap);
        Timer timer = new Timer(this);

        outtake.linkages.flipHalfDumper();
        intake.drop();

        waitForStart();

        new Thread( () -> {
            while (opModeIsActive()) {
                tankDrive.setPowerDir(gamepad2.right_stick_y, -gamepad2.left_stick_x);
            }
        }).start();

        while (opModeIsActive()) {

           if (gamepad1.right_trigger > 0){
                intake.in();
            } else if (gamepad1.left_trigger > 0){
                intake.out();
            } else{
                intake.stop();
            }

        if (gamepad1.dpad_up) {
            outtake.linkages.dumperIn();
            outtake.linkages.dumperIn();
            timer.safeDelay(200);
            outtake.slides.extendHigh();
        }

        if (gamepad1.dpad_right) {
            lazySusan.rotateToDegrees(90);
            outtake.linkages.extend();
        }
        if (gamepad1.dpad_left) { // make retract and down all in one method later
            lazySusan.rotateToDegrees(-90);
            outtake.linkages.extend();
        }
        if(gamepad1.right_bumper) {
            lazySusan.rotateToDegrees(180);
            outtake.linkages.extend();
        }
        if (gamepad1.dpad_down) {
            outtake.linkages.dumperOut();
            timer.safeDelay(500);
            lazySusan.rotateToDegrees(0);
            outtake.linkages.flipHalfDumper();
            outtake.linkages.retract();
            timer.safeDelay(1000);
            outtake.slides.retract();
        }

        if (gamepad1.right_stick_x > 0){
            lazySusan.turnRightIncrement();
        }
            if (gamepad1.right_stick_x < 0){
                lazySusan.turnLeftIncrement();
            }

            if (gamepad1.right_stick_button){
               duck.setBlueSpeed();
           }
           duck.move();


        }
    }
}
