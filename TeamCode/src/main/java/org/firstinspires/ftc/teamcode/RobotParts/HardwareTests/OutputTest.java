package org.firstinspires.ftc.teamcode.RobotParts.HardwareTests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotParts.Output;
import org.firstinspires.ftc.teamcode.RobotParts.RetractableOdo;

@TeleOp(group = "1 Teleop")
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

            if(gamepad1.y){
                output.flipOutDumper();
                telemetry.addData("Dumper Status", "Flip Out");
            }

            if(gamepad1.right_bumper){
                output.extend();
            }

            if(gamepad1.left_bumper){
                output.retract();
            }

            telemetry.update();
        }
    }
}
