package org.firstinspires.ftc.teamcode.Hardware.Opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.Duck.Duck;
import org.firstinspires.ftc.teamcode.Hardware.Intake.Intake;
import org.firstinspires.ftc.teamcode.Hardware.Outtake.Outtake;
import org.firstinspires.ftc.teamcode.Hardware.Turret.LazySusan;
import org.firstinspires.ftc.teamcode.Hardware.util.Timer;
import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class Teleop extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        Outtake outtake = new Outtake(hardwareMap, this);
        Intake intake = new Intake(hardwareMap);
        LazySusan lazySusan = new LazySusan(hardwareMap);
        SampleTankDrive tankDrive = new SampleTankDrive(hardwareMap);
        Duck duck = new Duck(hardwareMap);
        Timer timer = new Timer(this);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        outtake.readyToIntake();

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
                outtake.closeDumper();
            }
            if(gamepad1.right_bumper){
                outtake.readyToIntake();
            }


            if (gamepad1.dpad_up) {
                outtake.up();
            }

            if (gamepad1.dpad_right) {
                outtake.right();
            }
            if (gamepad1.dpad_left) {
                outtake.left();
            }
            if(gamepad1.y) {
                outtake.back();
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

        telemetry.addData("Turret Angle", lazySusan.getDegrees());
        telemetry.addData("Drive Velocity", tankDrive.getVelos());
        telemetry.update();
        }
    }
}
