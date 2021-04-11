package org.firstinspires.ftc.teamcode.ourOpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Bluetooth.BluetoothConvenient;
import org.firstinspires.ftc.teamcode.Roadrunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RobotParts.Shooter;
import org.firstinspires.ftc.teamcode.RobotParts.ShooterFlicker;
import org.firstinspires.ftc.teamcode.RobotParts.Wobble_Arm;
import org.firstinspires.ftc.teamcode.ourOpModes.resources.IMU;
import org.firstinspires.ftc.teamcode.ourOpModes.resources.RotationUtil;
import org.firstinspires.ftc.teamcode.ourOpModes.robotParts.RingCollector;

@TeleOp(name="AAA Teleop", group="Teleop")
//@Disabled
//use DriveWheelIMULocalization for the same functionality instead
public class Teleop extends LinearOpMode
{
    // Declare OpMode members.
    //Mecanum MecanumDrive;

    IMU imu;
    SampleMecanumDrive drive;
    double headingZero = 0;

    BluetoothConvenient BT;

    public void runOpMode() throws InterruptedException {
        //BT = new BluetoothConvenient(telemetry, hardwareMap, this);

        //MecanumDrive = new Mecanum(hardwareMap);
        RingCollector ringCollector = new RingCollector(hardwareMap);
        Wobble_Arm wobble_arm = new Wobble_Arm(hardwareMap, Teleop.this);
        ShooterFlicker flicker = new ShooterFlicker(hardwareMap, this, telemetry);
        Shooter shooter = new Shooter(hardwareMap);

        //Reset wobble arm to up position
        wobble_arm.automaticReleaseWobble();
        flicker.flickIn();

        drive = new SampleMecanumDrive(hardwareMap, telemetry);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        imu = drive.getIMU();

        // TRAJECTORY STUFF
        // We want to start the bot at x: 10, y: -8, heading: 90 degrees
        Pose2d startPose = new Pose2d(-60.8, 16.92, 0);
        drive.setPoseEstimate(startPose);

        Trajectory toHighGoal;
        Trajectory toCollection;
        Trajectory toStart;

        boolean debug_disable_shooter = true;

        waitForStart();

        while (opModeIsActive()){

            Pose2d p = drive.getPoseEstimate();
            telemetry.addData("Current Position", p);
            //BT.bluetoothClient.send(String.format("\\xyrplot %.2f %.2f %.2f\n", -p.getY()/12.0 + 6, p.getX()/12.0 + 6 , p.getHeading()));

            if (gamepad1.dpad_up){
                toHighGoal = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .splineTo(new Vector2d(-6.5, 27.4), Math.toRadians(12.5))
                        .build();

                drive.followTrajectory(toHighGoal);
            }

            if (gamepad1.dpad_right){
                boolean reverse = Math.abs(drive.getPoseEstimate().getHeading()) < Math.PI/2;
                toCollection = drive.trajectoryBuilder(drive.getPoseEstimate(), reverse)
                        .splineTo(new Vector2d(2.9, 24.9), 0)
                        .build();

                drive.followTrajectory(toCollection);
            }

            if (gamepad1.dpad_down){
                boolean reverse = Math.abs(drive.getPoseEstimate().getHeading()) < Math.PI/2;
                toStart = drive.trajectoryBuilder(drive.getPoseEstimate(), reverse)
                        .splineTo(new Vector2d(-60.8, 21.92), reverse?Math.PI:0)
                        .build();

                drive.followTrajectory(toStart);
            }

            drive.update();

            double y = -gamepad1.right_stick_y;
            double x = gamepad1.right_stick_x;

            // absolute turning
            double targetDir = -Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 2;
            double magnitude = Math.hypot(gamepad1.left_stick_y, gamepad1.left_stick_x);
            double turnPwr = RotationUtil.turnLeftOrRight(imu.getHeading(), targetDir + headingZero, Math.PI * 2);

            if(gamepad2.dpad_left){
                shooter.setHighGoalSpeed();
            }

            if(gamepad2.dpad_right){
                shooter.setPowerShotSpeed();
            }

            if(gamepad2.dpad_up){
                shooter.increaseSpeed();
            }
            else if(gamepad2.dpad_down){
                shooter.decreaseSpeed();
            }

            if(gamepad1.right_stick_button)
                debug_disable_shooter = !debug_disable_shooter;

            if(debug_disable_shooter)
                shooter.stop();
            else
                shooter.setSpeed();

            if (! (gamepad1.right_trigger > 0 || gamepad1.left_trigger > 0) ){
                x*= 1.0/3;
//                y*= 1.0/3;
            }

            if(gamepad2.right_stick_button){
                // high goal
                //follower.goTo(-4, 22, -0.31);
            }

            DRIVE(y, x, (magnitude > 0.5 && Math.abs(turnPwr) > 0.08) ? -turnPwr/2 : 0, drive);

            ringCollector.collect(gamepad2.left_trigger - gamepad2.right_trigger);

            if (gamepad2.left_bumper){
                flicker.autoFlick();
            }
            if (gamepad2.right_bumper){
                flicker.singleFlick();
            }

            if (gamepad1.a) {
                wobble_arm.down();
            }
            if (gamepad1.b) {
                wobble_arm.up();
            }
            if (gamepad1.right_bumper) {
                wobble_arm.grab();
            }
            if (gamepad1.left_bumper) {
                wobble_arm.unGrab();
            }

            if(gamepad1.dpad_up)
            {
                headingZero = imu.getHeading();
            }

//            telemetry.addData("Flicker Position", flicker.getPosition());
//            telemetry.addData("Is Flick In", (MathStuff.isEqual(flicker.getPosition(), flicker.flickIn)));
//            telemetry.addData("Is Flick Out", (MathStuff.isEqual(flicker.getPosition(), flicker.flickOut)));
//            telemetry.addData("IsGrabbing? ", wobble_arm.isGrabbing);
//            telemetry.addData("Angler Postion:", wobble_arm.getAnglerPosition());
//            telemetry.addData("Gripper Postion:", wobble_arm.getGripperPosition());

            telemetry.addData("Shooter Velocity", shooter.getShooterVelocity());
            telemetry.addData("Target Velocity", shooter.getTargetVelocity());
            telemetry.update();
        }


        BT.bluetoothClient.endHostSession();
        BT.interpreter.input.forceEnd = true;
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


