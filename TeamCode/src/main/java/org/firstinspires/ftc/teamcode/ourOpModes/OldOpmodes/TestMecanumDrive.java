package org.firstinspires.ftc.teamcode.ourOpModes.OldOpmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotParts.Mecanum;

@TeleOp(group = "drive")
@Disabled
public class TestMecanumDrive extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Mecanum drive =  new Mecanum(hardwareMap);
        waitForStart();

        while (!isStopRequested()) {
           drive.drive(10,10,0);
        }
    }
}