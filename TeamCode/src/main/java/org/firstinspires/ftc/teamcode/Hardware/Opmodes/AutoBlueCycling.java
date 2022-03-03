package org.firstinspires.ftc.teamcode.Hardware.Opmodes;

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
@Disabled
public class AutoBlueCycling extends LinearOpMode {
    OpenCvWebcam camera;
    public static double
            startX = -35.5+48, startY = 70, startH = Math.toRadians(270),
            preloadX = startX, preloadY = startY - 16, preloadH = 270,
            preWharehouseX = 11, preWharehouseY = 72, preWhareHouseH = 0,
            whareHouseX = 62, whareHouseY = 72, whareHouseH = 0,
            scoreX = 62, scoreY = 62, scoreH = 235;

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
        Trajectory preload = drive.trajectoryBuilder(start)
                .splineTo(new Vector2d(preloadX, preloadY), Math.toRadians(preloadH))
                .build();
        Trajectory firstToBlocks = drive.trajectoryBuilder(preload.end())
                .splineTo(new Vector2d(preWharehouseX, preWharehouseY), Math.toRadians(preWhareHouseH))
                .splineTo(new Vector2d(whareHouseX, whareHouseY), Math.toRadians(whareHouseH))
                .build();
        Trajectory toScore = drive.trajectoryBuilder(firstToBlocks.end(), true)
                .splineTo(new Vector2d(preWharehouseX, preWharehouseY), Math.toRadians(preWhareHouseH))
                .splineTo(new Vector2d(scoreX, scoreY), Math.toRadians(scoreH))
                .build();
        Trajectory toBlocksCycling = drive.trajectoryBuilder(toScore.end())
                .splineTo(new Vector2d(preWharehouseX, preWharehouseY), Math.toRadians(preWhareHouseH))
                .splineTo(new Vector2d(whareHouseX, whareHouseY), Math.toRadians(whareHouseH))
                .build();

        // Position Setup
        drive.setPoseEstimate(start);
        turret.closeDumper();
        intake.drop();
        // todo make this power based
        turret.resetTurretZero();

        telemetry.addData("Status ", "Not Ready to Start");
        telemetry.update();
        waitForStart();
        camera.stopStreaming();
        
//        intake.setPower(-0.6);
//        camera.closeCameraDevice();
        // TODO: 3/2/22 see if this takes out the crashing issue


        // Preload
        drive.followTrajectory(preload);
        drive.turnTo(Math.toRadians(preloadH));
        
//         decide the preload up pos, take red because it's flipped
        switch (cv.getLocation()) {
            case RIGHT:
                turret.preloadMidRed();
                turret.preloadDown();
                break;
            case LEFT:
                turret.preloadLowRed();
                turret.preloadDownLowRed();
                break;
            case OUT_OF_FRAME:
                turret.preloadUpRed();
                turret.preloadDown();
                break;
        }

//        drive.turnTo(Math.toRadians(90));

        // INTAKE first block
        intake.smartIn(turret,timer);
        drive.followTrajectoryBreakBlock(firstToBlocks, turret);

        //Score and go back to wharehouse
        // TODO: 3/3/22 for loop
        drive.followTrajectoryAsyncTask(toScore, () -> turret.right());
        drive.followTrajectoryAsyncTaskBreakBlock(toBlocksCycling, turret, () -> turret.down());
    }
}