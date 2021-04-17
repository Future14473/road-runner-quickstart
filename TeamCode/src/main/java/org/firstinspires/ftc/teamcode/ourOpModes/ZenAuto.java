package org.firstinspires.ftc.teamcode.ourOpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Bluetooth.BluetoothConvenient;
import org.firstinspires.ftc.teamcode.ComputerVision.Detection;
import org.firstinspires.ftc.teamcode.Roadrunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RobotParts.Shooter;
import org.firstinspires.ftc.teamcode.RobotParts.ShooterFlicker;
import org.firstinspires.ftc.teamcode.RobotParts.SideStyx;
import org.firstinspires.ftc.teamcode.RobotParts.Toggleable;
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

@Autonomous(name = "ZenAuto", group = "Autonomous")
//@Disabled
//use DriveWheelIMULocalization for the same functionality instead
public class ZenAuto extends LinearOpMode {
    // Declare OpMode members.

    IMU imu;
    SampleMecanumDrive drive;
    double headingZero = 0;

    Wobble_Arm wobble_arm;

    public void runOpMode() throws InterruptedException {

        //setup RC for display
        int cameraMonitorViewId = hardwareMap.appContext.getResources().
                getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        int[] viewportContainerIds = OpenCvCameraFactory.getInstance().splitLayoutForMultipleViewports(cameraMonitorViewId,
                2, OpenCvCameraFactory.ViewportSplitMethod.HORIZONTALLY);

        //OpenCV Setup
        OpenCvCamera webcam = OpenCvCameraFactory.getInstance().
                createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), viewportContainerIds[1]);

        Detection detector = new Detection(telemetry);

        webcam.setPipeline(detector);

        webcam.openCameraDeviceAsync(() -> {
            webcam.startStreaming(352, 288, OpenCvCameraRotation.UPRIGHT);
        });

        //Vuforia Setup
        VuforiaPhone vuforiaPhone = new VuforiaPhone(hardwareMap, viewportContainerIds);

        FtcDashboard.getInstance().startCameraStream(webcam, 0);
        DirtyGlobalVariables.vuforia = vuforiaPhone;
        DirtyGlobalVariables.vuforia.beginTracking();

        //Robot Part Setup
        RingCollector ringCollector = new RingCollector(hardwareMap);
        wobble_arm = new Wobble_Arm(hardwareMap, this);
        ShooterFlicker flicker = new ShooterFlicker(hardwareMap, this, telemetry);
        SideStyx styx = new SideStyx(hardwareMap, telemetry);

        Shooter shooter = new Shooter(hardwareMap);
        Timing timer = new Timing(this);

        //Reset wobble arm to up position
//        wobble_arm.automaticReleaseWobble();
        flicker.flickIn();
        wobble_arm.grab();


        drive = new SampleMecanumDrive(hardwareMap, telemetry);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        imu = drive.getIMU();

        Toggleable speedUp = new Toggleable(shooter::increaseSpeed);
        Toggleable speedDown = new Toggleable(shooter::decreaseSpeed);

        // TRAJECTORY STUFF
        // We want to start the bot at x: 10, y: -8, heading: 90 degrees
        Pose2d startPose = new Pose2d(-54.5, 20, 0);
        styx.allDown();
        drive.setPoseEstimate(startPose);

        boolean debug_disable_shooter = true;

        waitForStart();

        webcam.stopStreaming();
        ringCollector.collect(1);
        wobble_arm.up();

        new Thread(()->{
            while (opModeIsActive()) {
                shooter.setSpeed();
//                drive.update();
                //telemetry.addData("Shooter Velocity", shooter.getShooterVelocity());
            }
        }).start();

        int whichPowerShot = 0;

        Pose2d p = drive.getPoseEstimate();

        //High Goal Shooting
        goTo(-8, 31.0, Math.toRadians(20));
        flicker.autoFlick();

        for(int i = 0; i<5;i++){
            drive.update();
        }

        telemetry.addData("stack height", detector.stack);
        telemetry.update();
        if(detector.stack == 0){
            goTo(27,50,0);        }
        else if(detector.stack == 1){
            goTo(36,25,0);
        }
        else{
            goTo(63,50,0);
        }
        wobble_arm.down();
        timer.safeDelay(500);
        wobble_arm.automaticReleaseWobble();

        goTo(-1, 37, Math.PI);
        //Ring collect
        goTo(-26,37, Math.PI);

        drive.update();

        telemetry.addData("Shooter Target Vel", shooter.getTargetVelocity());

        DirtyGlobalVariables.telemetry.update();
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


