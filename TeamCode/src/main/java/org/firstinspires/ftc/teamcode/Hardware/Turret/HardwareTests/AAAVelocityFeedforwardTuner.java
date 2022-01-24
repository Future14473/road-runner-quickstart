package org.firstinspires.ftc.teamcode.Hardware.Turret.HardwareTests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.kinematics.Kinematics;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.Hardware.Turret.LazySusan;

import static org.firstinspires.ftc.teamcode.Hardware.Turret.TurretConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.Hardware.Turret.TurretConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.Hardware.Turret.TurretConstants.kA;
import static org.firstinspires.ftc.teamcode.Hardware.Turret.TurretConstants.kStatic;
import static org.firstinspires.ftc.teamcode.Hardware.Turret.TurretConstants.kV;

// importing important constants from Turret Constants

/*
 * This routine is designed to tune the open-loop feedforward coefficients. Although it may seem unnecessary,
 * tuning these coefficients is just as important as the positional parameters. Like the other
 * manual tuning routines, this op mode relies heavily upon the dashboard. To access the dashboard,
 * connect your computer to the RC's WiFi network. In your browser, navigate to
 * https://192.168.49.1:8080/dash if you're using the RC phone or https://192.168.43.1:8080/dash if
 * you are using the Control Hub. Once you've successfully connected, start the program, and your
 * robot will begin moving forward and backward according to a motion profile. Your job is to graph
 * the velocity errors over time and adjust the feedforward coefficients. Once you've found a
 * satisfactory set of gains, add them to the appropriate fields in the DriveConstants.java file.
 *
 * Pressing Y/Δ (Xbox/PS4) will pause the tuning process and enter driver override, allowing the
 * user to reset the position of the bot in the event that it drifts off the path.
 * Pressing B/O (Xbox/PS4) will cede control back to the tuning process.
 */
@Config
@Autonomous(group = "drive")
public class AAAVelocityFeedforwardTuner extends LinearOpMode {
    public static double DEGREES = 45;

    private FtcDashboard dashboard = FtcDashboard.getInstance();

    private LazySusan lazySusan;

    enum Mode {
        DRIVER_MODE,
        TUNING_MODE
    }

    private Mode mode;

    private static MotionProfile generateProfile(boolean movingForward) {
        MotionState start = new MotionState(movingForward ? 0 : DEGREES, 0, 0, 0);
        MotionState goal = new MotionState(movingForward ? DEGREES : 0, 0, 0, 0);
        return MotionProfileGenerator.generateSimpleMotionProfile(start, goal, MAX_VEL, MAX_ACCEL);
    }

    @Override
    public void runOpMode() {
        // creating the lazy susan
        lazySusan = new LazySusan(hardwareMap);

        // returns if lazy susan is running using with encoders
        if (lazySusan.getRunMode()== DcMotorEx.RunMode.RUN_USING_ENCODERS) {
            RobotLog.setGlobalErrorMsg("Feedforward constants usually don't need to be tuned " +
                    "when using the built-in drive motor velocity PID.");
        }

        // creates telemetry
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        // set the denum mode to tuning mode since this class is for feedforward tuning
        mode = Mode.TUNING_MODE;

        //set up clock for timing
        NanoClock clock = NanoClock.system();

        //update telemetry
        telemetry.addLine("Ready!");
        telemetry.update();
        telemetry.clearAll();

        // wait for start
        waitForStart();

        boolean movingForwards = true;
        MotionProfile activeProfile = generateProfile(true);
        double profileStart = clock.seconds();


        while (!isStopRequested()) {
            telemetry.addData("mode", mode);

            switch (mode) {
                case TUNING_MODE:
                    if (gamepad1.y) {
                        mode = Mode.DRIVER_MODE;
                    }

                    // calculate and set the motor power
                    double profileTime = clock.seconds() - profileStart;

                    if (profileTime > activeProfile.duration()) {
                        // generate a new profile
                        movingForwards = !movingForwards;
                        activeProfile = generateProfile(movingForwards);
                        profileStart = clock.seconds();
                    }

                    MotionState motionState = activeProfile.get(profileTime);
//                    double targetPower = Kinematics.calculateMotorFeedforward(motionState.getV(), motionState.getA(), kV, kA, kStatic);
//                    lazySusan.setPower(targetPower);
                    lazySusan.setVelocity((int) motionState.getV());
                    lazySusan.rotateToDegrees(movingForwards ? 45 : 0);
                    double currentVelo = lazySusan.getVelo();


                    // update telemetry
                    //telemetry.addData("setPower ", targetVelocity);
//                    telemetry.addData("targetPower", targetPower);
                    telemetry.addData("current position", lazySusan.getDegrees());
                    telemetry.addData("target position", lazySusan.getTargetDegrees());
                    telemetry.addData("targetVelocity", motionState.getV());
                    telemetry.addData("measuredVelocity", currentVelo);
                    telemetry.addData("error", motionState.getV() - currentVelo);
                    break;
                case DRIVER_MODE:
                    if (gamepad1.b) {
                        mode = Mode.TUNING_MODE;
                        movingForwards = true;
                        activeProfile = generateProfile(movingForwards);
                        profileStart = clock.seconds();
                    }

//                    drive.setWeightedDrivePower(
//                            new Pose2d(
//                                    -gamepad1.left_stick_y,
//                                    -gamepad1.left_stick_x,
//                                    -gamepad1.right_stick_x
//                            )
//                    );
                    break;
            }

            telemetry.update();
        } // end of while
    }
}