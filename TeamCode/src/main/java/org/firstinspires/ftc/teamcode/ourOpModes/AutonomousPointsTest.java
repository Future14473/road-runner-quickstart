package org.firstinspires.ftc.teamcode.ourOpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Roadrunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Roadrunner.TwoWheelTrackingLocalizer;
import org.firstinspires.ftc.teamcode.ourMovementLib.Follower;
import org.firstinspires.ftc.teamcode.ourOpModes.resources.IMU;
@TeleOp(group = "teleop")
public class AutonomousPointsTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        TwoWheelTrackingLocalizer odometry = new TwoWheelTrackingLocalizer(hardwareMap, new SampleMecanumDrive(hardwareMap));
        IMU imu = new IMU(hardwareMap, telemetry);

        Follower follower = new Follower(new SampleMecanumDrive(hardwareMap), odometry, this, telemetry, gamepad1, imu);
        waitForStart();

        while (opModeIsActive()){
            if (gamepad1.b){
                //go to starting point
                follower.goTo(0,0,0);
            }

            if (gamepad1.dpad_up){
                follower.goTo(30,0,0);
            }

            if (gamepad1.dpad_down){
                follower.goTo(0,24,0);
            }

            if (gamepad1.dpad_right){
                follower.goTo(24,24,0);
            }

            if (gamepad1.dpad_left){
                follower.goTo(0,0,0);
            }

        }
    }
}
