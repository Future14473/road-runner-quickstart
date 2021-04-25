package org.firstinspires.ftc.teamcode.ourOpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Roadrunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RobotParts.RingCollector;
import org.firstinspires.ftc.teamcode.RobotParts.Shooter;
import org.firstinspires.ftc.teamcode.RobotParts.ShooterFlicker;
import org.firstinspires.ftc.teamcode.RobotParts.SideStyx;
import org.firstinspires.ftc.teamcode.RobotParts.Toggleable;
import org.firstinspires.ftc.teamcode.RobotParts.VuforiaPhone;
import org.firstinspires.ftc.teamcode.RobotParts.Wobble_Arm;
import org.firstinspires.ftc.teamcode.ourOpModes.resources.IMU;
import org.firstinspires.ftc.teamcode.ourOpModes.resources.Pathing;
import org.firstinspires.ftc.teamcode.ourOpModes.resources.RotationUtil;

@TeleOp(name = "AAA Teleop", group = "Teleop")
public class Teleop extends LinearOpMode {
    IMU imu;
    SampleMecanumDrive drive;
    double headingZero = 0;

    Wobble_Arm wobble_arm;
    RingCollector ringCollector;
    ShooterFlicker flicker;
    SideStyx styx;
    Shooter shooter;

    boolean debug_disable_shooter = false;

    VuforiaPhone vuforiaPhone;

    public void runOpMode() throws InterruptedException {

        init_camera();

        DirtyGlobalVariables.isInAuto = false;
        DirtyGlobalVariables.vuforia = vuforiaPhone;
        DirtyGlobalVariables.vuforia.beginTracking();
        


        ringCollector = new RingCollector(hardwareMap);
        wobble_arm = new Wobble_Arm(hardwareMap, Teleop.this);
        flicker = new ShooterFlicker(hardwareMap, this, telemetry);
        styx = new SideStyx(hardwareMap, telemetry);
        shooter = new Shooter(hardwareMap);

        drive = new SampleMecanumDrive(hardwareMap, telemetry);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setPoseEstimate(new Pose2d(-54.5, 20, 0));

        imu = drive.getIMU();

        shooter.setHighGoalSpeedTeleop();
        flicker.flickIn();
        styx.allDown();

        Pathing pathing = new Pathing(drive);

        Toggleable toggleShooter = new Toggleable(()-> debug_disable_shooter = !debug_disable_shooter);
        Toggleable speedUp = new Toggleable(shooter::increaseSpeed);
        Toggleable speedDown = new Toggleable(shooter::decreaseSpeed);
        Toggleable tripleFlick = new Toggleable(()->
            new Thread(()->flicker.flickThrice(shooter)).start()
        );
        Toggleable singleFlick = new Toggleable(()->
                new Thread(()->flicker.singleFlick()).start()
        );

        waitForStart();

        while (opModeIsActive()) {
            // High Goal
            if (gamepad1.dpad_up)
                pathing.highGoal();
            // Collection
            if (gamepad1.dpad_right)
                pathing.collectPos();

            //Back to Home
            if (gamepad1.dpad_down)
                goTo(-60.8, 16.92, Math.toRadians(0));

            // Power Shots
            if(gamepad1.y)
                pathing.powerShot1();

            if(gamepad1.x) // 11.5 degs
                pathing.powerShot2();

            if(gamepad1.a) // 2.5 degs
                pathing.powerShot3();

            if (gamepad2.dpad_left)
                shooter.setHighGoalSpeed();

            if (gamepad2.dpad_right)
                shooter.setPowerShotSpeed();

            speedUp.toggle(gamepad2.dpad_up);
            speedDown.toggle(gamepad2.dpad_down);
            toggleShooter.toggle(gamepad1.right_stick_button);
            tripleFlick.toggle(gamepad2.left_bumper);
            singleFlick.toggle(gamepad2.left_bumper);

            if (debug_disable_shooter)
                shooter.stopHard();
            else
                shooter.setSpeed();

            ringCollector.collect(gamepad2.left_trigger - gamepad2.right_trigger);

            if (gamepad2.right_bumper)
                styx.allUp();
            else
                styx.allDown();

            if (gamepad2.a)
                wobble_arm.down();

            if (gamepad2.x)
                wobble_arm.grab();

            if (gamepad2.y)
                wobble_arm.unGrab();

            if (gamepad2.b)
                wobble_arm.up();

            drivetrain_controls();

            drive.update();

//            telemetry.addData("wobble arm pos", wobble_arm.getAnglerPosition());
            telemetry.addData("Shooter Velocity", shooter.getShooterVelocity());

            DirtyGlobalVariables.telemetry.update();
        }
    }

    void init_camera(){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().
                getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        int[] viewportContainerIds = {cameraMonitorViewId};

        vuforiaPhone = new VuforiaPhone(hardwareMap, viewportContainerIds);
    }

    void drivetrain_controls(){
        double y = -gamepad1.right_stick_y;
        double x = gamepad1.right_stick_x;

        double targetDir = -Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 2;

        if(Math.abs(targetDir) < Math.toRadians(20))
            targetDir = 0;

        double magnitude = Math.hypot(gamepad1.left_stick_y, gamepad1.left_stick_x);
        double turnPwr = RotationUtil.turnLeftOrRight(imu.getHeading(), targetDir + headingZero, Math.PI * 2);


        if (gamepad1.left_bumper) {
            turnPwr = gamepad1.left_stick_x;
            x *= 0.3;
        }

        if (gamepad1.right_trigger > 0 || gamepad1.left_trigger > 0)
            x *= 0.3;

        DRIVE(y, x, magnitude > 0.5 ? -turnPwr : 0, drive);
    }


    void goTo(double x, double y, double heading){
        Trajectory destination = drive.trajectoryBuilder(drive.getPoseEstimate())
                .splineToSplineHeading(new Pose2d(x, y, heading), heading)
                .build();

        drive.followTrajectory(destination);
    }

    /*
     * For POSITIVE forward parameter: go forward
     * For POSITIVE sideways parameter: go right
     * For POSITIVE rotate parameter: turn right
     */
    public void DRIVE(double forward, double sideways, double rotate, SampleMecanumDrive drivetrain) {
        drivetrain.setWeightedDrivePower(
                new Pose2d(
                        forward,
                        -sideways, // note the drivetrain wheels are reversed so sideways is positive right
                        -rotate * 2
                )
        );
    }
}
