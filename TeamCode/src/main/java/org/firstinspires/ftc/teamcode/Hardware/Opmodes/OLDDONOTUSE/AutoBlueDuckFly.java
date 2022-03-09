package org.firstinspires.ftc.teamcode.Hardware.Opmodes.OLDDONOTUSE;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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

@Autonomous
@Config
//@Disabled
public class AutoBlueDuckFly extends LinearOpMode {
    public static double preloadX = -29, preloadY = 49, preloadH = 270,
                            startX = -35.5, startY = 70, startH = Math.toRadians(270),
                            duckX = -53.5, duckY = 66.5, duckH = 180,
                            preScoreDuckX = -35, preScoreDuckY = 65, preScoreDuckH = 290,
                            scoreDuckX = -25, scoreDuckY = 53, scoreDuckH = 0,

                            preParkX = 20, preParkY = 43,
                            parkX = 55, parkY = 43;
    public static long duckWait = 3300;
    public static double duckPower = 0.7;

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
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        
        AprilBoundBoxPipeline cv = new AprilBoundBoxPipeline(0.166, 578.272, 578.272, 402.145, 221.506, telemetry);
        camera.setPipeline(cv);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {@Override public void onOpened() { //                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            camera.startStreaming(1280,720, OpenCvCameraRotation.UPRIGHT); } @Override public void onError(int errorCode) { }});
        
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        FtcDashboard.getInstance().startCameraStream(camera, 0);

        timer.safeDelay(3000);
        if (cv.getLocation() == null) { telemetry.addData("Capstone Position", "Null"); }
        else {
            switch (cv.getLocation()) {
                case RIGHT:
                    telemetry.addData("Position", "Right");
                    break;
                case LEFT:
                    telemetry.addData("Position", "Left");
                    break;
                case MIDDLE:
                    telemetry.addData("Position", "Middle");
                    break;
            }
        }
        telemetry.addData("Voltage", drive.batteryVoltageSensor.getVoltage());
        telemetry.addData("your", "mom test "); // TODO: 3/2/22 get rid of this later 
        

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

        telemetry.addData("Status ", "Ready to Start");
        telemetry.update();
        waitForStart();
        
        intake.setPower(-0.6);
//        camera.closeCameraDevice();
        // TODO: 3/2/22 see if this takes out the crashing issue


        // Preload
        drive.followTrajectory(preload);
        drive.turnTo(Math.toRadians(preloadH));
        
        // decide the preload up pos
        if (cv.getLocation() == null){  turret.preloadUpBlue(); }
        else {
            switch (cv.getLocation()) {
                case RIGHT:
                    turret.preloadUpBlue();
                    break;
                case LEFT:
                    turret.preloadLowBlue();
                    break;
                case MIDDLE:
                    turret.preloadMidBlue();
            }
        }
        
        turret.preloadDown();
        intake.stop();

        // Duck Drop
        duckPath = drive.trajectoryBuilder(drive.getPoseEstimate(), true)
                .splineTo(new Vector2d(duckX, duckY), Math.toRadians(duckH))
                .build();
        drive.followTrajectory(duckPath);
        drive.turn(Math.toRadians(10));
        alignDuck = drive.trajectoryBuilder(drive.getPoseEstimate())
                .back(1.5)
                .build();
        drive.followTrajectory(alignDuck);
        duck.autoDuckBlue(timer);
//        duck.setPower(duckPower);
//        timer.safeDelay(duckWait);

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

        // Score Duck
        scoreDuck = drive.trajectoryBuilder(drive.getPoseEstimate())
                .splineTo(new Vector2d(preScoreDuckX, preScoreDuckY), Math.toRadians(preScoreDuckH))
                .splineTo(new Vector2d(scoreDuckX, scoreDuckY), Math.toRadians(scoreDuckH))
                .build();
        drive.followTrajectory(scoreDuck);
        drive.turnTo(Math.toRadians(0));
        turret.duckScorePrepBlue();
        turret.down();

        //park
        park = drive.trajectoryBuilder(drive.getPoseEstimate())
                .splineTo(new Vector2d(preParkX, preParkY), Math.toRadians(0))
                .splineTo(new Vector2d(parkX, parkY), Math.toRadians(0))
                .build();
        drive.followTrajectory(park);
//        drive.setPowerDir(1.0,0);
//        timer.safeDelay(1000);
    }
}