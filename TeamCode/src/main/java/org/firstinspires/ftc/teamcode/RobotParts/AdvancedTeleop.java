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
        Capstone capstone = new Capstone(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Cycler cycler = new Cycler(new Intake(hardwareMap), new Output(hardwareMap), this);
        Duck duck = new Duck(hardwareMap);
        RetractableOdo retractableOdo = new RetractableOdo(hardwareMap);

        waitForStart();
        retractableOdo.upOdo();
        while (opModeIsActive()){

            //TOP DRIVER
            //____________________________________________________
            if(gamepad2.x){
                cycler.intakeOut();
            }

            if (gamepad2.y){
                cycler.retractIntakeTransfer();
            }

            if(gamepad2.b){
                cycler.dumperOutPrep();
            }

            if(gamepad2.a){
                cycler.dumpRetract();
            }

            // DUCK _________________________________________________________________
            duck.setStopSpeed();
            if (gamepad2.dpad_right){
                duck.setBlueSpeed();
                telemetry.addData("Duck Status", "Blue");
            }
            if (gamepad2.dpad_left){
                duck.setRedSpeed();
                telemetry.addData("Duck Status", "Red");
            }

            // CAPSTONE______________________________________________________________
            if (gamepad2.dpad_up){
                capstone.grab();
            }
            if (gamepad2.dpad_down) {
                capstone.place();
            }

            //BOTTOM DRIVER____________________________________________________________
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
