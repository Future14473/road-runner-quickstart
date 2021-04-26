package org.firstinspires.ftc.teamcode.ourOpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.ComputerVision.Detection;
import org.firstinspires.ftc.teamcode.Roadrunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RobotParts.Shooter;
import org.firstinspires.ftc.teamcode.RobotParts.ShooterFlicker;
import org.firstinspires.ftc.teamcode.RobotParts.SideStyx;
import org.firstinspires.ftc.teamcode.RobotParts.VuforiaPhone;
import org.firstinspires.ftc.teamcode.RobotParts.Wobble_Arm;
import org.firstinspires.ftc.teamcode.ourOpModes.resources.IMU;
import org.firstinspires.ftc.teamcode.ourOpModes.resources.Pathing;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "AAA Qing Auto", group = "Autonomous")
@Config
public class QingAuto extends LinearOpMode {

    SampleMecanumDrive drive;
    OpenCvCamera webcam;
    Detection detector;
    VuforiaPhone vuforiaPhone;

    Wobble_Arm wobble_arm;
    ShooterFlicker flicker;
    SideStyx styx;
    Shooter shooter;

    public static double
            box_close_x = 16,
            box_close_y = 49,

            box_medium_x = 46,
            box_medium_y = 23,

            box_far_x = 63,
            box_far_y = 47;

    private void init_camera(){
        //setup RC for display
        int cameraMonitorViewId = hardwareMap.appContext.getResources().
                getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        int[] viewportContainerIds = OpenCvCameraFactory.getInstance().splitLayoutForMultipleViewports(cameraMonitorViewId,
                2, OpenCvCameraFactory.ViewportSplitMethod.HORIZONTALLY);

        //OpenCV Setup
        webcam = OpenCvCameraFactory.getInstance().
                createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), viewportContainerIds[1]);

        detector = new Detection(telemetry);

        webcam.setPipeline(detector);

        webcam.openCameraDeviceAsync(() -> {
            webcam.startStreaming(352, 288, OpenCvCameraRotation.UPRIGHT);
        });

        //Vuforia Setup
        vuforiaPhone = new VuforiaPhone(hardwareMap, viewportContainerIds);

        FtcDashboard.getInstance().startCameraStream(webcam, 0);
        DirtyGlobalVariables.vuforia = vuforiaPhone;
        DirtyGlobalVariables.vuforia.beginTracking();
    }

    state current_state;
    enum state {
        TO_HIGH_GOAL, SHOOTING, BOXES, PARKING, IDLE
    }

    public void runOpMode() throws InterruptedException {
        telemetry.setAutoClear(true);

        DirtyGlobalVariables.isInAuto = true;

        init_camera();

        // init hardware objects
        wobble_arm = new Wobble_Arm(hardwareMap, this);
        flicker = new ShooterFlicker(hardwareMap, this, telemetry);
        styx = new SideStyx(hardwareMap, telemetry);
        shooter = new Shooter(hardwareMap);

        // init hardware positions
        flicker.flickIn();
        wobble_arm.grab();
        styx.allDown();

        // init drive
        drive = new SampleMecanumDrive(hardwareMap, telemetry);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // init pose
        Pose2d startPose = new Pose2d(-54.5, 20, 0);
        drive.setPoseEstimate(startPose);

        Pathing pathing = new Pathing(drive);

        waitForStart();

        webcam.stopStreaming();
        wobble_arm.up();

        current_state = state.TO_HIGH_GOAL;
        while (opModeIsActive() && !isStopRequested()) {

            Pathing.startTeleopPosition = drive.getPoseEstimate();
            telemetry.addData("Teleop Start Position", Pathing.startTeleopPosition);
            shooter.setSpeed();
            if (!drive.isBusy())
                drive.update();

            telemetry.addData("State", current_state.toString());
            telemetry.addData("stack height", detector.stack);
            telemetry.addData("Shooter Speed", shooter.getShooterVelocity());
            telemetry.update();

            switch (current_state) {
                case TO_HIGH_GOAL:
                    pathing.goToSplineHeading(-7, 24, Math.toRadians(24));
                    current_state = state.SHOOTING;
                    break;
                case SHOOTING:
//                    flicker.flickThrice(shooter);
                    flicker.fastTriFlick(shooter);
                    current_state = state.BOXES;
                    break;
                case BOXES:
                    boxes();
                    current_state = state.PARKING;
                    break;
                case PARKING:
                    pathing.goToLineConstant(17, 24, 0);
                    current_state = state.IDLE;
                    break;
                case IDLE:
                    DirtyGlobalVariables.isInAuto = false;
                    wobble_arm.home();
                    shooter.stop();

                    return;
            }
        }
    }

    void boxes(){
        switch(detector.stack){
        case 0:
            goTo(box_close_x,box_close_y, 0);
            break;
        case 1:
            goTo(box_medium_x, box_medium_y, 0);
            break;
        default:
            goTo(box_far_x, box_far_y, 0);
            break;
        }

        wobble_arm.down();
        delay(1000);
        wobble_arm.automaticReleaseWobble();
        delay(1000);
    }

    void goTo(double x, double y, double heading){
        Trajectory destination = drive.trajectoryBuilder(drive.getPoseEstimate())
                .splineToSplineHeading(new Pose2d(x, y, heading), heading)
                .build();

        drive.followTrajectory(destination);
    }

    public void delay(long delay){
        long start = System.currentTimeMillis();
        while((System.currentTimeMillis() - start < delay) && opModeIsActive()){
            //wait
        }
    }
}


