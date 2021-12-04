package org.firstinspires.ftc.teamcode.RobotParts.HardwareTests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotParts.Capstone;
import org.firstinspires.ftc.teamcode.RobotParts.Intake;

@TeleOp(group = "1 Teleop")
public class CapstoneTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Capstone capstone = new Capstone(hardwareMap);
        waitForStart();

        while (opModeIsActive()){
            if (gamepad2.x){
                capstone.grab();
                telemetry.addData("Capstone Position", "grab");
            }

            if (gamepad2.y){
                capstone.place();
                telemetry.addData("Capstone Position", "place");
            }



            telemetry.update();
        }
    }
}
