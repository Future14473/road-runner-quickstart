package org.firstinspires.ftc.teamcode.ourOpModes;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Roadrunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Roadrunner.TwoWheelTrackingLocalizer;
import org.firstinspires.ftc.teamcode.RobotParts.Shooter;
import org.firstinspires.ftc.teamcode.RobotParts.ShooterFlicker;
import org.firstinspires.ftc.teamcode.ourOpModes.resources.RotationUtil;
import org.firstinspires.ftc.teamcode.ourOpModes.resources.Timing;

@TeleOp (name = "AAA Auto")
public class Auto extends LinearOpMode {

    SampleMecanumDrive drive;
    Timing timer;
    ShooterFlicker flicker;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap, telemetry);
        timer = new Timing(this);
        flicker = new ShooterFlicker(hardwareMap, this, telemetry);
        Shooter shooter = new Shooter(hardwareMap);


        waitForStart();

        Pose2d startPose = new Pose2d(-54.5, 20, 0);
        drive.setPoseEstimate(startPose);

        while (opModeIsActive()){
            // POWER SHOTS
            shooter.setPowerShotSpeed();
            shooter.setSpeed();
            if (gamepad1.y){
                shootPowerShots();
            }
        }
    }

    void goTo(double x, double y, double heading) {
        Pose2d p = drive.getPoseEstimate();
        double pnAngle = p.getHeading() <= Math.PI ? p.getHeading() : p.getHeading() - 2 * Math.PI;
        boolean reverse = Math.abs(pnAngle) < Math.PI / 2 && drive.getPoseEstimate().getX() > x;
        ;
        Trajectory destination = drive.trajectoryBuilder(drive.getPoseEstimate(), reverse)
                .splineTo(new Vector2d(x, y), reverse ? Math.PI + heading : heading)
                .build();

        drive.followTrajectory(destination);
    }

    void shootPowerShots() {
        // go to the power shot 1 point
        goTo(-3.78, 15.42, Math.toRadians(17.27));
        flicker.singleFlick();

        turnStrong( Math.toRadians( 9.5));;
        timer.safeDelay(500);
        flicker.singleFlick();


        turnStrong(  Math.toRadians(6.5));
        timer.safeDelay(500);
        flicker.singleFlick();
    }

    void turnStrong(double targetDir) {
        PIDFController pid = new PIDFController(new PIDCoefficients(10, 2, 4), 0, 0);
        pid.setTargetPosition(0);
        double heading;
        while ((heading = drive.getIMU().getHeading() + 10) < Math.toRadians(5) && opModeIsActive()) {
            double pwr = pid.update(-RotationUtil.turnLeftOrRight(heading, targetDir, Math.PI * 2));
            telemetry.addData("PID power", pwr);
            DRIVE(0, 0, -pwr, drive);
        }
    }
    public void DRIVE(double forward, double sideways, double rotate, SampleMecanumDrive drivetrain) {
        drivetrain.setWeightedDrivePower(
                new Pose2d(
                        forward,
                        -sideways, // note the drivetrain wheels are reversed so sideways is positive right
                        -rotate * 2
                )
        );
    }

//    void powerShot1() {
//        goTo(-3.78, 15.42, Math.toRadians(17.27));
//    }
//
//    void powerShot2() {
//        goTo(-3.78, 15.42, Math.toRadians(9.5));
//    }
//
//    void powerShot3() {
//        goTo(-3.78, 15.42, Math.toRadians(6.5));
//    }
}
