package org.firstinspires.ftc.teamcode.Hardware.Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class DriveOnly extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        SampleTankDrive tankDrive = new SampleTankDrive(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {



           tankDrive.setPowerDir(gamepad1.right_stick_y, -gamepad1.left_stick_x);
        }
    }
}
