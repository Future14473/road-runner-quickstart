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
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        AutomaticDumper automaticDumper = new AutomaticDumper(intake, output);

        Duck duck = new Duck(hardwareMap);
        RetractableOdo retractableOdo = new RetractableOdo(hardwareMap);
        DcMotor noodles;

        waitForStart();
        retractableOdo.upOdo();
        noodles = hardwareMap.get(DcMotor.class, "noodles");
        while (opModeIsActive()){

//            intake.setNoodlePower(gamepad2.right_trigger - gamepad2.left_trigger);

            if (gamepad2.right_bumper){
                intake.slideOutInNoodles();
                if (noodles.getCurrentPosition() > 1){
                    intake.slideIn();
                }
            }

            if(gamepad2.left_bumper){
                automaticDumper.retractDump();
            }

            if (gamepad2.y){
                output.flipOutDumper();
            }
            if (gamepad2.x){
                output.flipHalfDumper();
            }
            if(gamepad2.a){
                output.retractFlipIn();
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
