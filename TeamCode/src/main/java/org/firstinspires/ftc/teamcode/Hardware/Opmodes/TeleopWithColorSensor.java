package org.firstinspires.ftc.teamcode.Hardware.Opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.ColorSensor.ColorSensor;
import org.firstinspires.ftc.teamcode.Hardware.Duck.Duck;
import org.firstinspires.ftc.teamcode.Hardware.Intake.Intake;
import org.firstinspires.ftc.teamcode.Hardware.Outtake.Outtake;
import org.firstinspires.ftc.teamcode.Hardware.Turret.LazySusan;
import org.firstinspires.ftc.teamcode.Hardware.util.Timer;
import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleopWithColorSensor extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        Outtake outtake = new Outtake(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        LazySusan lazySusan = new LazySusan(hardwareMap);
        SampleTankDrive tankDrive = new SampleTankDrive(hardwareMap);
        Duck duck = new Duck(hardwareMap);
        Timer timer = new Timer(this);
        ColorSensor colorSensor = new ColorSensor(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        outtake.linkages.flipHalfDumper();
        intake.drop();

        waitForStart();



        new Thread( () -> {
            while (opModeIsActive()) {
                tankDrive.setPowerDir(gamepad2.left_stick_y, -gamepad2.right_stick_x);
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

           if (gamepad1.left_bumper){
               outtake.linkages.dumperIn();
           }
           if(gamepad1.right_bumper){
               outtake.linkages.flipHalfDumper();
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
            if(gamepad1.y) {
                lazySusan.rotateToDegrees(180);
                outtake.linkages.extend();
            }
            if (gamepad1.dpad_down) {
                outtake.linkages.dumperOut();
                timer.safeDelay(500);
                outtake.linkages.retract();
                outtake.linkages.flipHalfDumper();
                timer.safeDelay(500);
                lazySusan.rotateToDegrees(0);
                timer.safeDelay(1000);
                outtake.slides.retract();
            }

        //manual linkage
//            if(gamepad1.left_stick_y > 0){
//                outtake.linkages.increment();
//            }
//            if(gamepad1.left_stick_y < 0){
//                outtake.linkages.decrement();
//            }

            if (gamepad1.right_stick_x > 0){
                lazySusan.turnRightIncrement();
            }
            if (gamepad1.right_stick_x < 0){
                lazySusan.turnLeftIncrement();
            }

            if (gamepad2.right_bumper){
                duck.setBlue(1);
           }
            if (gamepad2.left_bumper){
                duck.setRed(1);
            }
            if (gamepad2.a){
                duck.setStop();
            }


        telemetry.addData("Dumper isFilled", colorSensor.isBlock() ? "filled" : "empty");
            telemetry.addData("Dumper Block", colorSensor.getColor());
        telemetry.addData("Turret Angle", lazySusan.getDegrees());
        telemetry.addData("Drive Velocity", tankDrive.getVelos());
        telemetry.update();
        }
    }
}
