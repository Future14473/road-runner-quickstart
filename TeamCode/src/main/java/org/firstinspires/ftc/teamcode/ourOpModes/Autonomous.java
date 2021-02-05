package org.firstinspires.ftc.teamcode.ourOpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.TwoWheelTrackingLocalizer;
import org.firstinspires.ftc.teamcode.ourMovementLib.Follower;
import org.firstinspires.ftc.teamcode.ourMovementLib.paths.DaPath;
import org.firstinspires.ftc.teamcode.ourOpModes.resources.Timing;
import org.firstinspires.ftc.teamcode.ourOpModes.robotParts.Mecanum;

@TeleOp(name="Auto", group="Teleop")
public class Autonomous extends LinearOpMode {

    Timing timer = new Timing(Autonomous.this);
    Follower follower;
    TwoWheelTrackingLocalizer odometry;

    @Override
    public void runOpMode() throws InterruptedException {
        odometry = new TwoWheelTrackingLocalizer(hardwareMap, new SampleMecanumDrive(hardwareMap));
        follower = new Follower(
                new Mecanum(hardwareMap),
                odometry,
                Autonomous.this,
                telemetry);

        waitForStart();


        while (opModeIsActive()){
            telemetry.update();
            if (gamepad1.x){
                follower.goTo(DaPath.forward);
            }
            if (gamepad1.y){
                follower.goTo(DaPath.strafe);
            }
            if (gamepad1.a){
                follower.goTo(DaPath.turn);
            }
        }
    }


}
