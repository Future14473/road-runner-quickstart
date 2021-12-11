package org.firstinspires.ftc.teamcode.Mechanisms.HardwareTests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Mechanisms.Intake;
import org.firstinspires.ftc.teamcode.util.Timer;

import java.sql.Time;

@TeleOp(group = "A Hardware Tests")
public class IntakeTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Intake intake = new Intake(hardwareMap, new Timer(this));
        waitForStart();

        while (opModeIsActive()){
            if(gamepad1.right_bumper){
                intake.inNoodlesUp();
            }

            if(gamepad1.left_bumper){
                intake.outNoodlesUp();
            }

            if(gamepad1.x){
                intake.transferOutake();
                telemetry.addData("Transfer Status", "outtake");
            }

            if(gamepad1.y){
                intake.transferIntake();
            }

            if (gamepad1.a){
                intake.flipInTeleop();
            }

            if (gamepad1.b){
                intake.flipOutTeleop();
            }

            if (gamepad1.dpad_up){
                intake.flipOutAuto();
            }

            telemetry.addData("slidePower", gamepad1.right_stick_y);

            telemetry.update();
        }
    }
}
