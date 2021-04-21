package org.firstinspires.ftc.teamcode.ourOpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
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
import org.firstinspires.ftc.teamcode.RobotParts.Toggleable;
import org.firstinspires.ftc.teamcode.RobotParts.VuforiaPhone;
import org.firstinspires.ftc.teamcode.RobotParts.Wobble_Arm;
import org.firstinspires.ftc.teamcode.ourOpModes.resources.IMU;
import org.firstinspires.ftc.teamcode.ourOpModes.resources.RotationUtil;
import org.firstinspires.ftc.teamcode.ourOpModes.resources.Timing;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "AAA Qing Auto", group = "Autonomous")
//@Disabled
//use DriveWheelIMULocalization for the same functionality instead
@Config
public class QingAuto extends LinearOpMode {
    // Declare OpMode members.

    IMU imu;
    SampleMecanumDrive drive;
    double headingZero = 0;

    Wobble_Arm wobble_arm;

    public static double high_goal_x = -10;
    public static double high_goal_y = 57;

    public static double box_close_x = 25;
    public static double box_close_y = 50;

    public static double box_medium_x = 46;
    public static double box_medium_y = 30;

    public static double box_far_x = 63;
    public static double box_far_y = 47;

    public static double before_stack_x = 5;
    public static double before_stack_y = 44;

    public static double grab_wobble_x = -46;
    public static double grab_wobble_y = 30;

    public static int wobble_close_delay = 900,
    wobble_middle_delay = 1200,
    wobble_far_delay = 2000;

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
        //Timing timer = new Timing(this);

        //Reset wobble arm to up position
//        wobble_arm.automaticReleaseWobble();
        flicker.flickIn();
        wobble_arm.grab();


        drive = new SampleMecanumDrive(hardwareMap, telemetry);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        imu = drive.getIMU();

        // TRAJECTORY STUFF
        // We want to start the bot at x: 10, y: -8, heading: 90 degrees
        Pose2d startPose = new Pose2d(-54.5, 20, 0);
        styx.allDown();
        drive.setPoseEstimate(startPose);

        waitForStart();

        webcam.stopStreaming();
        ringCollector.collect(1);
        wobble_arm.up();

        new Thread(()->{
            while (opModeIsActive()) {
                shooter.setSpeed();
                telemetry.setAutoClear(true);
                if(!drive.following)
                    drive.update();
                //telemetry.addData("Shooter Velocity", shooter.getShooterVelocity());
            }
        }).start();

        int whichPowerShot = 0;

        Pose2d p = drive.getPoseEstimate();

        //High Goal Shooting
        goTo(high_goal_x, high_goal_y, Math.toRadians(20));
        flicker.autoFlick();

        telemetry.addData("stack height", detector.stack);
        telemetry.update();


        boxes(detector);


        //vuforia scan position
//        goTo(before_stack_x, before_stack_y, Math.toRadians(20));
//
//        wobble_arm.down();
//        wobble_arm.unGrab();
//
//        //grab wobble position
//        goTo(grab_wobble_x,grab_wobble_y, 0);
//
//        wobble_arm.grab();
//        wobble_arm.up();
//
//        boxes(detector);

        goTo(17, 24, 0);
    }

    void boxes(Detection detector){
        if(detector.stack == 0){
            goTo(box_close_x,box_close_y,0);

            new Thread(()->{

                Timing timer = new Timing(this);
                //timer.safeDelay(wobble_close_delay);
                wobble_arm.down();
                timer.safeDelay(1000);
                wobble_arm.automaticReleaseWobble();

            }).start();

        }
        else if(detector.stack == 1){
            goTo(box_medium_x,box_medium_y,0);

            new Thread(()->{

                Timing timer = new Timing(this);
                //timer.safeDelay(wobble_middle_delay);
                wobble_arm.down();
                timer.safeDelay(500);
                wobble_arm.automaticReleaseWobble();

            }).start();
        }
        else{
            goTo(box_far_x, box_far_y,0);

            new Thread(()->{

                Timing timer = new Timing(this);
                //timer.safeDelay(wobble_far_delay);
                wobble_arm.down();
                timer.safeDelay(500);
                wobble_arm.automaticReleaseWobble();

            }).start();

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
        Trajectory destination = drive.trajectoryBuilder(drive.getPoseEstimate())
                .splineToConstantHeading(new Vector2d(x, y), heading)
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


