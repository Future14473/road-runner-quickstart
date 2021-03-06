package org.firstinspires.ftc.teamcode.ourOpModes.unit_tests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.TwoWheelTrackingLocalizer;
import org.firstinspires.ftc.teamcode.ourMovementLib.Follower;
import org.firstinspires.ftc.teamcode.ourMovementLib.PathPoint;
import org.firstinspires.ftc.teamcode.ourOpModes.robotParts.Mecanum;

@TeleOp(group = "drive")
public class TestMecanumDrive extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Mecanum drive =  new Mecanum(hardwareMap);
        telemetry.addData("Rotation is Gamepad 1 x joystick, ", "Translation is Gamepad 1 x and y joystick");
        telemetry.update();
        waitForStart();

        while (!isStopRequested()) {
            telemetry.addData("Running", "mecanum driving only");
            telemetry.update();
           drive.drive(gamepad1.right_stick_x, -gamepad1.right_stick_y, gamepad1.left_stick_x);
        }
    }
}