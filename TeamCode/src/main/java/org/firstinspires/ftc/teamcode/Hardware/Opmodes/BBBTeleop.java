package org.firstinspires.ftc.teamcode.Hardware.Opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.Outtake.BoxSensor;
import org.firstinspires.ftc.teamcode.Hardware.Duck.Duck;
import org.firstinspires.ftc.teamcode.Hardware.Intake.Intake;
import org.firstinspires.ftc.teamcode.Hardware.Outtake.Linkages;
import org.firstinspires.ftc.teamcode.Hardware.Outtake.Turret;
import org.firstinspires.ftc.teamcode.Hardware.Outtake.LazySusan;
import org.firstinspires.ftc.teamcode.Hardware.util.Timer;
import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class BBBTeleop extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        Turret turret = new Turret(hardwareMap, this);
        Intake intake = new Intake(hardwareMap);
        LazySusan lazySusan = new LazySusan(hardwareMap);
        SampleTankDrive tankDrive = new SampleTankDrive(hardwareMap);
        Duck duck = new Duck(hardwareMap);
        Timer timer = new Timer(this);
        BoxSensor colorSensor = new BoxSensor(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        boolean rightBumper1PrevState = false, rightBumperCurrState, game1aPrevState = false;

        turret.readyToIntake();

        intake.drop();

        waitForStart();

        new Thread( () -> {
            while (opModeIsActive()) {
                if(gamepad2.left_trigger > 0.5) {
                    tankDrive.setPowerDir(gamepad2.left_stick_y, gamepad2.right_stick_x * 0.85);
                } else if (gamepad2.right_trigger > 0.5){
                    tankDrive.setPowerDir(gamepad2.left_stick_y * 0.25, gamepad2.right_stick_x * 0.25);
                } else {
                    tankDrive.setPowerDir(gamepad2.left_stick_y * 0.65, gamepad2.right_stick_x * 0.65);
                }
            }
        }).start();

        while (opModeIsActive()) {

            if (game1aPrevState != gamepad1.a){
                if (gamepad1.a){
                    turret.isShared = !turret.isShared;
                }
            }
            game1aPrevState = gamepad1.a;
            telemetry.addData("isShared", turret.isShared);

           if (gamepad1.right_trigger > 0) {
               intake.smartIn(turret, timer);
           }
           else if (gamepad1.left_trigger > 0){
                intake.out();
            } else {
               intake.stop();
           }


            if (gamepad1.dpad_up) {
                turret.up();
            }

            if (gamepad1.dpad_right) {
                turret.right();
            }
            if (gamepad1.dpad_left) { // make retract and down all in one method later
                turret.left();
            }
            if (gamepad1.dpad_down){
                turret.down();
            }
            if(gamepad1.y) {
                turret.back();
            }

            if ((gamepad1.right_stick_x > 0) && !turret.isDown()){
                lazySusan.turnRightIncrement();
            }
            if ((gamepad1.right_stick_x < 0) && !turret.isDown()){
                lazySusan.turnLeftIncrement();
            }

            if (gamepad1.x){
                turret.rightSharedHub();
            }
            if (gamepad1.b){
                turret.leftSharedHub();
            }

            if (gamepad1.left_bumper){
                turret.letGoEmergency();
                intake.out();
                timer.safeDelay(800);
            }

            rightBumperCurrState = gamepad1.right_bumper;
            // if the button wasn't just pressed
            if (rightBumperCurrState != rightBumper1PrevState) {
                // and you really did press right bumper
                if (rightBumperCurrState == true) {
                    turret.toggleLinkages();
                }
            }

            rightBumper1PrevState = rightBumperCurrState;

            duck.setStop();
            if (gamepad2.right_bumper){
                duck.setBlue();
            }
            if (gamepad2.left_bumper){
                duck.setRed();
            }
            duck.move();





        telemetry.addData("Dumper isFilled", colorSensor.hasBlock() ? "filled" : "empty");
        telemetry.addData("Dumper Block", colorSensor.getColor());
        telemetry.addData("Toggle Pos", Linkages.toggleIndex);
        telemetry.addData("Turret Angle", lazySusan.getDegrees());
        telemetry.addData("Slide Height", turret.getHeight());
        telemetry.update();
        }
    }
}
