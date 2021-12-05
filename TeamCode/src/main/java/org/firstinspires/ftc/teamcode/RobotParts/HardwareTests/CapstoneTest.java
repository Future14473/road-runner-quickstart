package org.firstinspires.ftc.teamcode.RobotParts.HardwareTests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotParts.Capstone;

@TeleOp(group = "1 Teleop")
public class CapstoneTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Capstone capstone = new Capstone(hardwareMap);
        waitForStart();

        while (opModeIsActive()){
            if (gamepad2.x){
                capstone.down();
                telemetry.addData("Capstone Position", "grab");
            }

            if (gamepad2.y){
                capstone.highestLimit();
                telemetry.addData("Capstone Position", "place");
            }

            if (gamepad2.b){
                capstone.cap();
                telemetry.addData("Capstone Position", "cap");
            }

            if (gamepad2.a){
                capstone.holdUp();
                telemetry.addData("Capstone Position", "hold Up");
            }


            if (gamepad2.right_bumper){
                capstone.preCap();
                telemetry.addData("Capstone Position", "preCap");
            }


            telemetry.update();
        }
    }
}
