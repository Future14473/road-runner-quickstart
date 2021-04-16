package org.firstinspires.ftc.teamcode.ourOpModes;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Bluetooth.BluetoothConvenient;
import org.firstinspires.ftc.teamcode.ComputerVision.Detection;
import org.firstinspires.ftc.teamcode.Roadrunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RobotParts.Shooter;
import org.firstinspires.ftc.teamcode.RobotParts.ShooterFlicker;
import org.firstinspires.ftc.teamcode.RobotParts.SideStyx;
import org.firstinspires.ftc.teamcode.RobotParts.VuforiaPhone;
import org.firstinspires.ftc.teamcode.RobotParts.Wobble_Arm;
import org.firstinspires.ftc.teamcode.ourOpModes.resources.IMU;
import org.firstinspires.ftc.teamcode.ourOpModes.resources.RotationUtil;
import org.firstinspires.ftc.teamcode.ourOpModes.resources.Timing;
import org.firstinspires.ftc.teamcode.RobotParts.RingCollector;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.Objects;

@TeleOp(name = "AAA Auto", group = "Teleop")
//@Disabled
//use DriveWheelIMULocalization for the same functionality instead
public class Auto extends LinearOpMode {
    // Declare OpMode members.
    //Mecanum MecanumDrive;

    IMU imu;
    SampleMecanumDrive drive;
    double headingZero = 0;

//    BluetoothConvenient BT;

    Wobble_Arm wobble_arm;

    public void runOpMode() throws InterruptedException {
//        BT = new BluetoothConvenient(telemetry, hardwareMap, this);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().
                getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        int[] viewportContainerIds = OpenCvCameraFactory.getInstance().splitLayoutForMultipleViewports(cameraMonitorViewId,
                2, OpenCvCameraFactory.ViewportSplitMethod.HORIZONTALLY);

        OpenCvCamera webcam = OpenCvCameraFactory.getInstance().
                createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), viewportContainerIds[1]);

        Detection detector = new Detection(telemetry);

        VuforiaPhone vuforiaPhone = new VuforiaPhone(hardwareMap, viewportContainerIds);

        webcam.setPipeline(detector);


        DirtyGlobalVariables.vuforia = vuforiaPhone;
        DirtyGlobalVariables.vuforia.beginTracking();

//        Opening and Streaming from Camera

        webcam.openCameraDeviceAsync(() -> {
            webcam.startStreaming(352, 288, OpenCvCameraRotation.UPRIGHT);
        });

        RingCollector ringCollector = new RingCollector(hardwareMap);
        wobble_arm = new Wobble_Arm(hardwareMap, this);
        ShooterFlicker flicker = new ShooterFlicker(hardwareMap, this, telemetry);
        SideStyx styx = new SideStyx(hardwareMap, telemetry);

        Shooter shooter = new Shooter(hardwareMap);
        shooter.stop();

        //Reset wobble arm to up position
//        wobble_arm.automaticReleaseWobble();
        flicker.flickIn();

        drive = new SampleMecanumDrive(hardwareMap, telemetry);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        imu = drive.getIMU();

        // TRAJECTORY STUFF
        // We want to start the bot at x: 10, y: -8, heading: 90 degrees
        Pose2d startPose = new Pose2d(-54.5, 20, 0);
        styx.allDown();
        drive.setPoseEstimate(startPose);

        boolean debug_disable_shooter = true;

        waitForStart();
        webcam.stopStreaming();
        new Thread(()->{
            while (opModeIsActive()) {
                shooter.setSpeed();
                //telemetry.addData("Shooter Velocity", shooter.getShooterVelocity());
            }
        }).start();

        int whichPowerShot = 0;
        /*
        if(detector.stack == 0){
            goTo(46,13.2,0.3);
        }
        else if(detector.stack == 1){
            goTo(35,18,0.42);
        }
        else{

        }
        */




        while (opModeIsActive()) {
            Pose2d p = drive.getPoseEstimate();
            //double pnAngle = p.getHeading() <= Math.PI ? p.getHeading(): p.getHeading() - 2* Math.PI;
//            DirtyGlobalVariables.telemetry.addData("Current Position", p);
//            BT.bluetoothClient.send(String.format("\\xyrplot %.2f %.2f %.2f\n", -p.getY()/12.0 + 6, p.getX()/12.0 + 6 , p.getHeading()));

            if (gamepad1.right_trigger > 0.2){
                goTo(-24,28,Math.PI);
            }

            if (gamepad1.a){
                goTo(27,42,0);
            }
            if (gamepad1.b){
                goTo(51,18,0);
            }
            if (gamepad1.right_bumper){
                goTo(68,42,0);
            }

            if (gamepad1.dpad_up)
                goTo(-8, 34.0, Math.toRadians(15.5));

            if (gamepad1.dpad_right)
                goTo(2.9, 24.9, Math.toRadians(0));

            if (gamepad1.dpad_down)
                goTo(-60.8, 16.92, Math.toRadians(0));

            if(gamepad1.y) {
                shooter.setPowerShotSpeed();

                goTo(-3.78, 15.42, Math.toRadians(17.27));
                flicker.singleFlick();

                goTo(-3.78, 15.42, Math.toRadians(11.5));
                flicker.singleFlick();

                goTo(-3.78, 15.42, Math.toRadians(6.5));
                flicker.singleFlick();

            }

//            if (gamepad1.)

            if(gamepad1.x){
                shooter.stop();

                if(whichPowerShot == 0){
                    turnStrong(17.27);

                }if(whichPowerShot == 1){
                    turnStrong(11.5);

                }if(whichPowerShot == 2){
                    turnStrong(6.5);

                }
                whichPowerShot++;
                whichPowerShot %= 3;
            }

            drive.update();

            double y = -gamepad1.right_stick_y;
            double x = gamepad1.right_stick_x;

            // absolute turning
            double targetDir = -Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 2;
            double magnitude = Math.hypot(gamepad1.left_stick_y, gamepad1.left_stick_x);
            double turnPwr = RotationUtil.turnLeftOrRight(imu.getHeading(), targetDir + headingZero, Math.PI * 2);
            if (gamepad1.left_bumper) {
                turnPwr = gamepad1.left_stick_x;
                x *= 0.3;
            }

            if (gamepad2.dpad_left) {
                shooter.setHighGoalSpeed();
            }

            if (gamepad2.dpad_right) {
                shooter.setPowerShotSpeed();
            }

            if (gamepad2.dpad_up) {
                shooter.increaseSpeed();
            } else if (gamepad2.dpad_down) {
                shooter.decreaseSpeed();
            }

            if(gamepad1.right_stick_button){
                debug_disable_shooter = !debug_disable_shooter;
            }

//            if (debug_disable_shooter)
//                shooter.stop();
//            else
//                shooter.setSpeed();


            DRIVE(y, x, (magnitude > 0.5 && Math.abs(turnPwr) > 0.08) ? -turnPwr / 2 : 0, drive);

            ringCollector.collect(gamepad2.left_trigger + gamepad2.right_trigger);

            if (gamepad2.left_bumper) {
                flicker.autoFlick();
            }
            if (gamepad2.right_bumper) {
                styx.allUp();
            } else {
                styx.allDown();
            }

            if (gamepad2.a) {
                wobble_arm.down();
            }
            if (gamepad2.x) {
                wobble_arm.grab();
            }


            if (gamepad2.y) {
                wobble_arm.unGrab();
            }

            if (gamepad2.b) {
                wobble_arm.up();
            }

//            if (gamepad1.dpad_up) {
//                headingZero = imu.getHeading();
//            }
//            telemetry.addData("Flicker Position", flicker.getPosition());
//            telemetry.addData("Is Flick In", (MathStuff.isEqual(flicker.getPosition(), flicker.flickIn)));
//            telemetry.addData("Is Flick Out", (MathStuff.isEqual(flicker.getPosition(), flicker.flickOut)));
//            telemetry.addData("IsGrabbing? ", wobble_arm.isGrabbing);
//            telemetry.addData("Angler Postion:", wobble_arm.getAnglerPosition());
//            telemetry.addData("Gripper Postion:", wobble_arm.getGripperPosition());

            //telemetry.addData("Target Velocity", shooter.getTargetVelocity());
            DirtyGlobalVariables.telemetry.update();
        }

    }

    public void goto_forth(int x, int y, int heading){
        goTo(x/10.0, y/10.0, Math.toRadians(heading));
    }

    void goTo(double x, double y, double heading){
        Pose2d p = drive.getPoseEstimate();
        double pnAngle = p.getHeading() <= Math.PI ? p.getHeading(): p.getHeading() - 2* Math.PI;
        boolean reverse = Math.abs(pnAngle) < Math.PI / 2 && drive.getPoseEstimate().getX() > x;
        ;
        Trajectory destination = drive.trajectoryBuilder(drive.getPoseEstimate(), reverse)
                .splineTo(new Vector2d(x, y), reverse ? Math.PI + heading: heading)
                .build();

        drive.followTrajectory(destination);
    }
    void turnStrong(double targetDir){
        PIDFController pid = new PIDFController(new PIDCoefficients(10, 2, 4), 0, 0);
        pid.setTargetPosition(0);
        double toTurn;
        while (
                (toTurn = RotationUtil.turnLeftOrRight(imu.getHeading(), targetDir, Math.PI*2)) > Math.toRadians(5) && opModeIsActive()){
            double pwr = pid.update(-toTurn);
            telemetry.addData("turn power", pwr);
            DRIVE(0, 0, -pwr, drive);
        }

    }

//    void turnTo(double heading){
//        double curr_heading = Objects.requireNonNull(drive.getPoseEstimate().getHeading(), "Curr heading is null");
//        double delta_heading = Objects.requireNonNull((RotationUtil.turnLeftOrRight(curr_heading, Math.toRadians(heading), Math.PI*2)),"delta heading null");
//        Trajectory destination = drive.trajectoryBuilder(drive.getPoseEstimate())
//                .splineTo(new Vector2d(x, y), reverse ? Math.PI + heading: heading)
//                .build();
//        Trajectory direction =
//                drive.trajectoryBuilder(drive.getPoseEstimate())
//                        .splineTo(
//                        new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), delta_heading)
//                )
//
//                        .build();
//
//        drive.turn(delta_heading);
//    }

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


