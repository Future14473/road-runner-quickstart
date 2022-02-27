package org.firstinspires.ftc.teamcode.Hardware.Opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.Outtake.BoxSensor;
import org.firstinspires.ftc.teamcode.Hardware.Duck.Duck;
import org.firstinspires.ftc.teamcode.Hardware.Intake.Intake;
import org.firstinspires.ftc.teamcode.Hardware.Outtake.Linkages;
import org.firstinspires.ftc.teamcode.Hardware.Outtake.Turret;
import org.firstinspires.ftc.teamcode.Hardware.Outtake.LazySusan;
import org.firstinspires.ftc.teamcode.Hardware.util.Timer;
import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class HubTrackingTest extends LinearOpMode {

    public static double XcurM =0;
    public static double YcurM =0;
    public static double HcurM =0;

    @Override
    public void runOpMode() throws InterruptedException {
        Turret turret = new Turret(hardwareMap, this);

        LazySusan lazySusan = new LazySusan(hardwareMap);
        SampleTankDrive tankDrive = new SampleTankDrive(hardwareMap);

        Timer timer = new Timer(this);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        boolean rightBumper1PrevState = false, rightBumperCurrState;

        turret.readyToIntake();

        waitForStart();



        new Thread( () -> {
            while (opModeIsActive()) {
                tankDrive.setPowerDir(-gamepad2.left_stick_y, gamepad2.right_stick_x * (gamepad2.right_bumper ? 1.0 : 0.85));
            }
        }).start();


        /// _____________________________________ INITIALIZE ROBOT LOCATION AT CENTER OF FIELD _____________________
        Pose2d start = new Pose2d(0,0,0);
        tankDrive.setPoseEstimate(start);

        // lift the slides all the way up
        turret.preloadUp();

        // jank variable declaration




        /// _____________________________________ INITIALIZE ROBOT LOCATION AT CENTER OF FIELD _____________________



        while (opModeIsActive()) {

                // so the robot should already be able to drive around
                // the only thing left is for the robot to be able to have the arm swing around to point to the hub

                // issue how do i turn the getpose2d function into 3 individual booleans to pass into the point to func
                // last two booleans are the locations pointed to --> the hub location



                turret.pointTo(XcurM,YcurM,HcurM,-12,24);

                telemetry.addData("Toggle Pos", Linkages.toggleIndex);
                telemetry.addData("Turret Angle", lazySusan.getDegrees());
                telemetry.addData("Slide Height", turret.getHeight());
                telemetry.update();

        }
    }
}
