package org.firstinspires.ftc.teamcode.Hardware.Opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.Outtake.BoxSensor;
import org.firstinspires.ftc.teamcode.Hardware.Duck.Duck;
import org.firstinspires.ftc.teamcode.Hardware.Intake.Intake;
import org.firstinspires.ftc.teamcode.Hardware.Outtake.Turret;
import org.firstinspires.ftc.teamcode.Hardware.Outtake.LazySusan;
import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleopWithColorSensor extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        Turret turret = new Turret(hardwareMap, this);
        Intake intake = new Intake(hardwareMap);
        LazySusan lazySusan = new LazySusan(hardwareMap);
        SampleTankDrive tankDrive = new SampleTankDrive(hardwareMap);
        Duck duck = new Duck(hardwareMap);
        BoxSensor colorSensor = new BoxSensor(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        turret.readyToIntake();

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
               turret.closeDumper();
           }
           if(gamepad1.right_bumper){
               turret.readyToIntake();
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

            if (gamepad1.right_stick_x > 0){
                lazySusan.turnRightIncrement();
            }
            if (gamepad1.right_stick_x < 0){
                lazySusan.turnLeftIncrement();
            }

            duck.setStop();
            if (gamepad2.right_bumper){
                duck.setBlue();
            }
            if (gamepad2.left_bumper){
                duck.setRed();
            }
            duck.move();


        telemetry.addData("Dumper isFilled", colorSensor.isBlock() ? "filled" : "empty");
            telemetry.addData("Dumper Block", colorSensor.getColor());
        telemetry.addData("Turret Angle", lazySusan.getDegrees());
        telemetry.addData("Drive Velocity", tankDrive.getVelos());
        telemetry.update();
        }
    }
}
