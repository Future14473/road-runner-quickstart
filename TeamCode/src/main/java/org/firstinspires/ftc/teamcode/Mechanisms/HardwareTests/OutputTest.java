package org.firstinspires.ftc.teamcode.Mechanisms.HardwareTests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Mechanisms.Output;

@TeleOp(group = "A ardware Tests")
public class OutputTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Output output = new Output(hardwareMap);
        waitForStart();

        while (opModeIsActive()){
            if(gamepad1.x){
                output.flipInDumper();
                telemetry.addData("Dumper Status", "Flip In");
            }

            if (gamepad1.b){
                output.flipHalfDumper();
                telemetry.addData("Dumper Status", "Flip In");
            }
            if(gamepad1.y){
                output.flipOutDumper();
                telemetry.addData("Dumper Status", "Flip Out");
            }

            if(gamepad1.right_bumper){
                output.extendHigh();
            }

            if(gamepad1.left_bumper){
                output.retract();
            }

            if (gamepad1.dpad_up){
                output.extendMid();
            }

            if (gamepad1.dpad_down){
                output.extendLow();
            }

            telemetry.update();
        }
    }
}
