package org.firstinspires.ftc.teamcode.Hardware.Opmodes.OLDDONOTUSE;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.AprilTag.AprilBoundBoxPipeline;
import org.firstinspires.ftc.teamcode.Hardware.Duck.Duck;
import org.firstinspires.ftc.teamcode.Hardware.Intake.Intake;
import org.firstinspires.ftc.teamcode.Hardware.Outtake.Turret;
import org.firstinspires.ftc.teamcode.Hardware.util.Timer;
import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp
@Config
@Disabled
public class BackupAutoBlueDuckFlyMarOne extends LinearOpMode {
    // pre x: -20   , pre y: 54.5, pre H: 290
    // start x -36-5.5  start y: 70 startH: 270
    public static double preloadX = -29, preloadY = 49, preloadH = 270,
                            startX = -35.5, startY = 70, startH = Math.toRadians(270),
                            duckX = -55, duckY = 65.5, duckH = 180,
                            scoreDuckX = -30, scoreDuckY = 50, scoreDuckH = 0,
                            preParkX = 20, preParkY = 43,
                            parkX = 55, parkY = 37;
    public static long duckWait = 3300;
    public static double duckPower = 0.7;
//    public volatile boolean isPreloadUp = false, isPreloadMid = false, isPreloadLow = false,
//            isPreloadDown = false, isPreloadDownLow = false,
//            isDuckScorePrepRed = false, isDuckScorePrepBlue = false,
//            isDown = false, isUp = false;

    @Override
    public void runOpMode() throws InterruptedException {
        // Hardware Setup
        SampleTankDrive drive = new SampleTankDrive(hardwareMap);
        Turret turret = new Turret(hardwareMap, this);
        Duck duck = new Duck(hardwareMap);
        Timer timer = new Timer(this);
        Intake intake = new Intake(hardwareMap);

        // Computer Vision Setup
        AprilBoundBoxPipeline cv = new AprilBoundBoxPipeline(0.166, 578.272, 578.272, 402.145, 221.506, telemetry);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        camera.setPipeline(cv);
        AprilBoundBoxPipeline.Location location;
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        FtcDashboard.getInstance().startCameraStream(camera, 0);
        telemetry.setMsTransmissionInterval(50);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
//                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
                camera.startStreaming(1280,720, OpenCvCameraRotation.UPRIGHT);
            }
            

            @Override
            public void onError(int errorCode)
            {

            }
        });

        // Trajectory Setup
        Pose2d start = new Pose2d(startX,startY,startH);
        Trajectory preload, duckPath, alignDuck, scoreDuck, park;
        preload = drive.trajectoryBuilder(start)
                .splineTo(new Vector2d(preloadX, preloadY), Math.toRadians(preloadH))
                .build();

        // Position Setup
        drive.setPoseEstimate(start);
        turret.closeDumper();
        intake.drop();
        // todo make this power based
        turret.resetTurretZero();

//        while ((cv.getLocation() == null) && opModeIsActive()){
//            telemetry.addData("Looking For April Tag, Value is", cv.location);
//            telemetry.update();
//        }

        timer.safeDelay(3000);
        if (cv.getLocation() == AprilBoundBoxPipeline.Location.LEFT) {
            telemetry.addData("Position", "Lefts");
        }
        if (cv.getLocation() == AprilBoundBoxPipeline.Location.MIDDLE) {
            telemetry.addData("Position", "Middle");
        }
        if (cv.getLocation() == AprilBoundBoxPipeline.Location.RIGHT) {
            telemetry.addData("Position", "Right");
        }
        if (cv.getLocation() == null) {
            telemetry.addData("Position", "Null");
        }
        telemetry.addData("In ", "Init");
        telemetry.update();

        waitForStart();
        intake.setPower(-0.6);
        camera.closeCameraDevice();


        // drive from start to preload
        drive.followTrajectory(preload);
        drive.turnTo(Math.toRadians(preloadH));
        // Preload
        // decide the preload up pos
        if (cv.getLocation() == AprilBoundBoxPipeline.Location.LEFT) {
            turret.preloadLowBlue();
        }
        if (cv.getLocation() == AprilBoundBoxPipeline.Location.MIDDLE) {
            turret.preloadMidBlue();
        }
        if (cv.getLocation() == AprilBoundBoxPipeline.Location.RIGHT) {
            turret.preloadUpBlue();
        }
        if (cv.getLocation() == null){
            turret.preloadUpBlue();
        }
        turret.preloadDown();
        intake.stop();



        // build the duck path
        duckPath = drive.trajectoryBuilder(drive.getPoseEstimate(), true)
                .splineTo(new Vector2d(duckX, duckY), Math.toRadians(duckH))
                .build();


        // Duck Drop
        drive.followTrajectory(duckPath);
        drive.turn(Math.toRadians(10));
        alignDuck = drive.trajectoryBuilder(drive.getPoseEstimate())
                .back(1.5)
                .build();
        drive.followTrajectory(alignDuck);
//        drive.turnTo(0);
//        duck.setBlue();
        duck.setPower(duckPower);
//        duck.move();
        timer.safeDelay(duckWait);

        //Pickup Duck
        intake.in();
        drive.turnToDuckCollect(Math.toRadians(90),turret);
        drive.turnToDuckCollect(Math.toRadians(180), turret);
        if (turret.hasBlock()) {
            turret.closeDumper();
        }
        drive.turnTo(Math.toRadians(0));
        turret.closeDumper();
        intake.stop();
        duck.setStop(); duck.move();

        scoreDuck = drive.trajectoryBuilder(drive.getPoseEstimate())
                .splineTo(new Vector2d(scoreDuckX, scoreDuckY), Math.toRadians(scoreDuckH))
                .build();

        // Score Duck
        drive.followTrajectory(scoreDuck);

        drive.turnTo(Math.toRadians(0));
//        drive.turn(Math.toRadians(-30));
//        turret.duckScorePrepBlueAsync();
//        isDuckScorePrepBlue = true;
//        new Thread ( () -> {
//            turret.duckScorePrepBlue();
//        }).start();
        turret.duckScorePrepBlue();
        // duck drop
        //turret.pointTo(drive.getPoseEstimate().getX(),drive.getPoseEstimate().getY(),drive.getPoseEstimate().getHeading(),-12,24);
//        turret.downAsync();
//        isDown = true;
//        new Thread ( () -> {
//            turret.down();
//        }).start();
        turret.down();

        park = drive.trajectoryBuilder(drive.getPoseEstimate())
                .splineTo(new Vector2d(preParkX, preParkY), Math.toRadians(0))
                .splineTo(new Vector2d(parkX, parkY), Math.toRadians(0))
                .build();

        //park
        drive.followTrajectory(park);
    }
}