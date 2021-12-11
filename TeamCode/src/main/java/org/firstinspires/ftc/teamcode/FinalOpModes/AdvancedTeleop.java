package org.firstinspires.ftc.teamcode.FinalOpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Mechanisms.Capstone;
import org.firstinspires.ftc.teamcode.Mechanisms.Cycler;
import org.firstinspires.ftc.teamcode.Mechanisms.Duck;
import org.firstinspires.ftc.teamcode.Mechanisms.Intake;
import org.firstinspires.ftc.teamcode.Mechanisms.Output;
import org.firstinspires.ftc.teamcode.Mechanisms.RetractableOdo;
import org.firstinspires.ftc.teamcode.z_drive.SampleMecanumDrive;

@TeleOp(group = "1 Teleop")
public class AdvancedTeleop extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Intake intake = new Intake(hardwareMap);
        Capstone capstone = new Capstone(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Cycler cycler = new Cycler(intake, new Output(hardwareMap), this);
        Duck duck = new Duck(hardwareMap);
        RetractableOdo retractableOdo = new RetractableOdo(hardwareMap);

        waitForStart();
        retractableOdo.upOdo();
        //drivetrain in thread so that you can use the automated cycling and still control the drivetrain
        new Thread( () -> {
            while(opModeIsActive()){
                //BOTTOM DRIVER____________________________________________________________
                //In-Line if statement that slows down the drivetrain if right bumper pressed

                drive.setDrivePower(
                        new Pose2d(
                                gamepad1.right_bumper ? -gamepad1.left_stick_y * 0.5 : -gamepad1.left_stick_y,
                                gamepad1.right_bumper ? -gamepad1.left_stick_x * 0.8: -gamepad1.left_stick_x,
                                gamepad1.right_bumper ? -gamepad1.right_stick_x * 0.5: -gamepad1.right_stick_x
                        )
                );
            }
        }).start();

        while (opModeIsActive()){
            //TOP DRIVER

            // UPDATED Advanced Teleop
            //Cycler____________________________________________________
            if(gamepad2.right_bumper){
                intake.inNoodlesUp();
            }
            if (gamepad2.left_bumper){
                intake.outNoodlesUp();
            }
            if((gamepad2.right_bumper == false) && (gamepad2.left_bumper == false)){
                intake.flipInTeleop();
                intake.setStopNoodlesSpeed();
            }
            intake.moveNoodles();

            //emergency move the transfer down
            if(gamepad2.x){
                intake.transferIntake();
            }

            if (gamepad2.y){
                cycler.transferIntakeToDumper();
            }

            if(gamepad2.b){
                intake.flipOutTeleop();
                intake.transferOutake();
                cycler.dumperOutPrepHigh();
                intake.transferIntake();
                intake.flipInTeleop();
            }

            if(gamepad2.a){
                cycler.dumpRetract();
            }

            // DUCK _________________________________________________________________
            duck.setStopSpeed();

            // right blue speed, left red speed
            duck.setDuckPowerVar(gamepad2.left_trigger - gamepad2.right_trigger);
            duck.setSpeed();

            // CAPSTONE______________________________________________________________
            if (gamepad2.dpad_left){
                capstone.down();
                telemetry.addData("Capstone Position", "grab");
            }

            if (gamepad2.dpad_up){
                capstone.holdUp();
                telemetry.addData("Capstone Position", "hold Up");
            }

            if (gamepad2.dpad_right){
                capstone.preCap();
                telemetry.addData("Capstone Position", "preCap");
            }

            if (gamepad2.dpad_down){
                capstone.cap();
                telemetry.addData("Capstone Position", "cap");
            }


            telemetry.update();
        }
    }
}
