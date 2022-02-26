package org.firstinspires.ftc.teamcode.Hardware.Opmodes;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.Duck.Duck;
import org.firstinspires.ftc.teamcode.Hardware.Intake.Intake;
import org.firstinspires.ftc.teamcode.Hardware.Outtake.Turret;
import org.firstinspires.ftc.teamcode.Hardware.util.Timer;
import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;

@TeleOp
@Config

public class turnTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleTankDrive drive = new SampleTankDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(0,0,0));
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.y) {
                drive.turn(Math.toRadians(-90));
            }
            if (gamepad1.b){
                drive.turn(Math.toRadians(90));
            }
            if (gamepad1.x) {
//                telemetry.addData("Turn distance",drive.turnTo(Math.toRadians(0)));
                drive.turnTo(Math.toRadians(0));
            }
            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("Turn distance",drive.turnAngle);
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
             }
        }
    }
