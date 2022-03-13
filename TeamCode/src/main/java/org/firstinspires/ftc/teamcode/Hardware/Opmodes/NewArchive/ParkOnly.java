package org.firstinspires.ftc.teamcode.Hardware.Opmodes.NewArchive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.ComputerVision.BlueCapstonePipeline;
import org.firstinspires.ftc.teamcode.Hardware.Duck.Duck;
import org.firstinspires.ftc.teamcode.Hardware.Intake.Intake;
import org.firstinspires.ftc.teamcode.Hardware.Outtake.Turret;
import org.firstinspires.ftc.teamcode.Hardware.util.Timer;
import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous
@Config
public class ParkOnly extends LinearOpMode {
    OpenCvWebcam camera;
    public static double preloadX = -29, preloadY = 49, preloadH = 270,
                            startX = -35.5, startY = 70, startH = Math.toRadians(270),
                            duckX = -54.5, duckY = 64.5, duckH = 182,
                            preScoreDuckX = -37, preScoreDuckY = 65, preScoreDuckH = 290,
                            scoreDuckX = -23, scoreDuckY = 50, scoreDuckH = 0,
                            alignDuckTurn = -17,
                            preParkX = 10, preParkY = 55, preParkH = 0,
                            parkX = 55, parkY = 55, parkH = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        // Hardware Setup
        SampleTankDrive drive = new SampleTankDrive(hardwareMap);
        Turret turret = new Turret(hardwareMap, this);
        Duck duck = new Duck(hardwareMap);
        Timer timer = new Timer(this);
        Intake intake = new Intake(hardwareMap);

        // Computer Vision Setup
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        
//        AprilBoundBoxPipeline cv = new AprilBoundBoxPipeline(0.166, 578.272, 578.272, 402.145, 221.506, telemetry);
        BlueCapstonePipeline cv = new BlueCapstonePipeline(telemetry);
        camera.setPipeline(cv);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {@Override public void onOpened() { //                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            camera.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT); } @Override public void onError(int errorCode) { }});

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        FtcDashboard.getInstance().startCameraStream(camera, 0);

        timer.safeDelay(3000);
        if (cv.getLocation() == null) { telemetry.addData("Capstone Position", "Null"); }
        else {
            switch (cv.getLocation()) {
                case RIGHT:
                    telemetry.addData("Position", "Middle");
                    break;
                case LEFT:
                    telemetry.addData("Position", "Left");
                    break;
                case OUT_OF_FRAME:
                    telemetry.addData("Position", "Right");
                    break;
            }
        }
        telemetry.addData("Voltage", drive.batteryVoltageSensor.getVoltage());

        // Trajectory Setup
        Pose2d start = new Pose2d(startX,startY,startH);
//        Trajectory preload = drive.trajectoryBuilder(start)
//                .splineTo(new Vector2d(preloadX, preloadY), Math.toRadians(preloadH))
//                .build();
//        Trajectory duckPath = drive.trajectoryBuilder(preload.end(), true)
//                .splineTo(new Vector2d(duckX, duckY), Math.toRadians(duckH))
//                .build();
////        Trajectory alignDuck = drive.trajectoryBuilder(duckPath.end())
////                .back(1.5)
////                .build();
//        Trajectory scoreDuck = drive.trajectoryBuilder(duckPath.end())
//                .splineTo(new Vector2d(preScoreDuckX, preScoreDuckY), Math.toRadians(preScoreDuckH))
//                .splineTo(new Vector2d(scoreDuckX, scoreDuckY), Math.toRadians(scoreDuckH))
//                .build();
//        Trajectory park = drive.trajectoryBuilder(scoreDuck.end())
//                .splineTo(new Vector2d(preParkX, preParkY), Math.toRadians(preParkH))
//                .splineTo(new Vector2d(parkX, parkY), Math.toRadians(parkH))
//                .build();
//        Trajectory preParkLeft = drive.trajectoryBuilder(scoreDuck.end())
//                .splineTo(new Vector2d(preParkX, preParkY), Math.toRadians(preParkH))
//                .build();
//        Trajectory parkLeft = drive.trajectoryBuilder(preParkLeft.end())
//                .splineTo(new Vector2d(parkX, parkY), Math.toRadians(parkH))
//                .build();
        Trajectory parkOnly = drive.trajectoryBuilder(new Pose2d())
                .forward(35)
                .build();

        // Position Setup
//        drive.setPoseEstimate(start);
        turret.closeDumper();

        // todo make this power based
        turret.resetTurretZero();

        telemetry.addData("Status ", "Not Ready to Start");
        telemetry.update();
        waitForStart();
        intake.drop();
        camera.stopStreaming();
        drive.followTrajectory(parkOnly);
//        intake.setPower(-0.6);
//        camera.closeCameraDevice();
        // TODO: 3/2/22 see if this takes out the crashing issue

/*
        // Preload
        drive.followTrajectory(preload);
        drive.turnTo(Math.toRadians(preloadH));
        
//         decide the preload up pos
        switch (cv.getLocation()) {
            case RIGHT:
                turret.preloadMidBlue();
                turret.preloadDown();
                break;
            case LEFT:
                turret.preloadLowBlue();
                turret.preloadDownLowBlue();
                break;
            case OUT_OF_FRAME:
                turret.preloadUpBlue();
                turret.preloadDown();
                break;
        }

        intake.stop();

        // Duck Drop
        drive.followTrajectory(duckPath);
        drive.turn(Math.toRadians(alignDuckTurn));
//        drive.followTrajectory(alignDuck);
        duck.autoDuckBlue(timer);
//        duck.setPower(duckPower);
//        timer.safeDelay(duckWait);

        //Pickup Duck
        intake.in();
        drive.turnToDuckCollect(Math.toRadians(90),turret);
        drive.turnToDuckCollect(Math.toRadians(200), turret);
        if (turret.hasBlock()) {
            turret.closeDumper();
        }
        drive.turnToDuckCollect(Math.toRadians(90), turret);
        drive.turnTo(Math.toRadians(0));
        duck.setStop(); duck.move();

        // Score Duck
        drive.followTrajectoryCloseDump(scoreDuck, turret);
        intake.stop();
        drive.turnTo(Math.toRadians(0));
        turret.duckScorePrepBlue();
        turret.down();

        //park
        intake.out();
        drive.turn(Math.toRadians(15));
        if (cv.getLocation() == BlueCapstonePipeline.Location.LEFT){
            drive.followTrajectory(preParkLeft);
            drive.turn(Math.toRadians(15));
            drive.followTrajectory(parkLeft);
//            drive.setPowerDir(1.0,0);
//            timer.safeDelay(1000);
        } else {
            drive.followTrajectory(park);
        }
//        drive.setPowerDir(1.0,0);
//        timer.safeDelay(1000);*/
    }
}