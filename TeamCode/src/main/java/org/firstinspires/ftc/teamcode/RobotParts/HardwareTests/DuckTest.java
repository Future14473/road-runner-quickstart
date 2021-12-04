package org.firstinspires.ftc.teamcode.RobotParts.HardwareTests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotParts.Capstone;
import org.firstinspires.ftc.teamcode.RobotParts.Duck;

@TeleOp(group = "1 Teleop")
public class DuckTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Duck duck = new Duck(hardwareMap);
        waitForStart();

        while (opModeIsActive()){
            duck.setStopSpeed();
            if (gamepad2.dpad_right){
                duck.setBlueSpeed();
                telemetry.addData("Duck Status", "Blue");
            }
            if (gamepad2.dpad_left){
                duck.setRedSpeed();
                telemetry.addData("Duck Status", "Red");
            }
            duck.setSpeed();
            telemetry.update();
        }
    }
}
