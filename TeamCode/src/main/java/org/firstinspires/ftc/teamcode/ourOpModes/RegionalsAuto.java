package org.firstinspires.ftc.teamcode.ourOpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.ComputerVision.Detection;
import org.firstinspires.ftc.teamcode.Roadrunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RobotParts.RingCollector;
import org.firstinspires.ftc.teamcode.RobotParts.Shooter;
import org.firstinspires.ftc.teamcode.RobotParts.ShooterFlicker;
import org.firstinspires.ftc.teamcode.RobotParts.SideStyx;
import org.firstinspires.ftc.teamcode.RobotParts.VuforiaPhone;
import org.firstinspires.ftc.teamcode.RobotParts.Wobble_Arm;
import org.firstinspires.ftc.teamcode.ourOpModes.resources.Pathing;
import org.firstinspires.ftc.teamcode.ourOpModes.resources.Timing;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "AAA Regionals Auto", group = "Autonomous")
@Config
public class RegionalsAuto extends LinearOpMode {

    SampleMecanumDrive drive;
    Pathing pathing;

    OpenCvCamera webcam;
    Detection detector;
    VuforiaPhone vuforiaPhone;

    Wobble_Arm wobble_arm;
    ShooterFlicker flicker;
    SideStyx styx;
    Shooter shooter;

    public static double
            box_close_x = 25,
            box_close_y = 49,

            box_medium_x = 49,
            box_medium_y = 26,

            box_far_x = 68,
            box_far_y = 47,

            pre_collect_x = -33.5,
            pre_collect_y = 20,



            wobble_grab_x = pre_collect_x,
            wobble_grab_y = 38,

            recollect_x = -20.5,
            recollect_y = 42;
    public static boolean fast = true;

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
        TO_HIGH_GOAL, SHOOTING, BOXES, WOBBLE, RECOLLECT, PARKING, BOXES_AGAIN, IDLE, SHOOT_AGAIN,
    }

    public void runOpMode() throws InterruptedException {
        telemetry.setAutoClear(true);

        DirtyGlobalVariables.isInAuto = true;

        init_camera();

        // init hardware objects
        RingCollector collector = new RingCollector(hardwareMap);
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
        pathing = new Pathing(drive);

        // init pose
        Pose2d startPose = new Pose2d(-54.5, 20, 0);
        drive.setPoseEstimate(startPose);

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
                    pathing.goToSplineHeading(-3, 24, Math.toRadians(24));
                    current_state = state.SHOOTING;
                    break;
                case SHOOTING:
//                    flicker.flickThrice(shooter);
                    if(fast){
                        flicker.fastTriFlick(shooter);
                    }
                    else {
                        flicker.flickThrice(shooter);
                    }
                    current_state = state.BOXES;
                    shooter.stop();
                    break;
                case BOXES:
                    boxes();
                    current_state = state.WOBBLE;
                    break;
                case WOBBLE:
                    pathing.goToLineWobbleDown(pre_collect_x, pre_collect_y, 0, 0.5, wobble_arm);
                    pathing.goToLine(wobble_grab_x, wobble_grab_y, 0);
                    shooter.setHighGoalSpeed(); //get ready for second shoot
                    delay(300);
                    wobble_arm.grab();
//                    delay(300);
//                    wobble_arm.up();
                    current_state = state.RECOLLECT;
                    break;
                case RECOLLECT:
                    collector.collect(1);
                    pathing.goToLine(recollect_x, recollect_y, 0);

                    if (detector.stack == 0){
                        current_state = state.BOXES_AGAIN;
                    } else {
                        current_state = state.SHOOT_AGAIN;
                    }
                    break;
                case SHOOT_AGAIN:
                    pathing.goToLine(-3, 24, Math.toRadians(24));
                    flicker.fastTriFlick(shooter);
                    current_state = state.BOXES_AGAIN;
                    shooter.stop();
                    break;
                case BOXES_AGAIN:
                    boxes2();
                    current_state = state.PARKING;
                    break;
                case PARKING:
                    pathing.goToLineConstant(17, 35, 0);
                    current_state = state.IDLE;
                    break;
                case IDLE:
                    DirtyGlobalVariables.isInAuto = false;
                    collector.stop();
                    wobble_arm.home();
                    shooter.stop();
                    return;
            }
        }
    }

    void boxes(){
        switch(detector.stack){
        case 0:
            pathing.goToLineWobbleDown(box_close_x,box_close_y, 0, 0.5, wobble_arm);
            break;
        case 1:
            pathing.goToLineWobbleDown(box_medium_x,box_medium_y, 0, 0.5, wobble_arm);
            break;
        default:
            pathing.goToLineWobbleDown(box_far_x,box_far_y, 0, 0.5, wobble_arm);
            break;
        }

        wobble_arm.unGrab();
        new Thread(()->{
            delay(200);
            wobble_arm.up();
        }
        ).start();
    }

    void boxes2(){
        switch(detector.stack){
            case 0:
                pathing.goToLineWobbleDown(box_close_x-5,box_close_y, 0, 0.9, wobble_arm);
                break;
            case 1:
                pathing.goToLineWobbleDown(box_medium_x-3,box_medium_y, 0, 0.9, wobble_arm);
                break;
            default:
                pathing.goToLineWobbleDown(box_far_x,box_far_y, 0, 0.9, wobble_arm);
                break;
        }

        wobble_arm.unGrab();
        new Thread(()->{
            delay(100);
            wobble_arm.up();
        }
        ).start();
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


