package org.firstinspires.ftc.teamcode.RobotParts;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(group = "1 Teleop")
public class AdvancedTeleop extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Intake intake = new Intake(hardwareMap);
        Output output = new Output(hardwareMap);
        Capstone capstone = new Capstone(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        AutomaticDumper automaticDumper = new AutomaticDumper(intake, output);

        Duck duck = new Duck(hardwareMap);
        RetractableOdo retractableOdo = new RetractableOdo(hardwareMap);
        DcMotor noodles;
        noodles = hardwareMap.get(DcMotor.class, "noodles");


        waitForStart();
        retractableOdo.upOdo();
        while (opModeIsActive()){

//            intake.setNoodlePower(gamepad2.right_trigger - gamepad2.left_trigger);


            if (gamepad2.a){
                intake.slideOut();
                intake.inNoodles();

            }
            if (gamepad2.b){
                intake.slideIn();
                intake.stopNoodles();
                intake.transferOutake();
                intake.transferIntake();
            }
            if (gamepad2.x){
                output.extend();
                output.flipHalfDumper();
            }
            if (gamepad2.y){
                output.retract();
                output.flipInDumper();
            }
            if (gamepad2.dpad_down){
                output.flipInDumper();
            }
            duck.setStopSpeed();
            if (gamepad2.dpad_right){
                duck.setBlueSpeed();
                telemetry.addData("Duck Status", "Blue");
            }
            if (gamepad2.dpad_left){
                duck.setRedSpeed();
                telemetry.addData("Duck Status", "Red");
            }
            if (gamepad2.dpad_up){
                capstone.grab();
            }
            if (gamepad2.dpad_down) {
                capstone.place();
            }


            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );


            telemetry.update();
        }
    }
}
