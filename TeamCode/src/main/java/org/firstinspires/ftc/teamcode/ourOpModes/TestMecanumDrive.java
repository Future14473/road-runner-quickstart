package org.firstinspires.ftc.teamcode.ourOpModes;

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
        waitForStart();

        while (!isStopRequested()) {
           drive.drive(10,10,0);
        }
    }
}