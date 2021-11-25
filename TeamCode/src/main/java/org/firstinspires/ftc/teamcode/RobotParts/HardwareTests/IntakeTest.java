package org.firstinspires.ftc.teamcode.RobotParts.HardwareTests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotParts.Intake;
import org.firstinspires.ftc.teamcode.RobotParts.Output;

@TeleOp(group = "1 Teleop")
public class IntakeTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Intake intake = new Intake(hardwareMap);
        waitForStart();

        while (opModeIsActive()){
            intake.setNoodlePower(gamepad1.right_trigger - gamepad1.left_trigger);
            telemetry.addData("Intake Positions", intake.getSlidePosition());
            if(gamepad1.right_bumper){
                intake.slideOut();
            }

            if(gamepad1.left_bumper){
                intake.slideIn();
                telemetry.addData("Transfer Status", "intake");
            }

            if(gamepad1.x){
                intake.transferOutake();
                telemetry.addData("Transfer Status", "outtake");
            }

            if(gamepad1.y){
                intake.transferIntake();
            }

            if (gamepad1.a){
                intake.flipIn();
            }

            if (gamepad1.b){
                intake.flipOut();
            }

            intake.setSlideSpeed(gamepad1.right_stick_y);
            telemetry.addData("slidePower", gamepad1.right_stick_y);

            telemetry.update();
        }
    }
}
