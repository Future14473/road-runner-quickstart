package org.firstinspires.ftc.teamcode.ComputerVision;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.firstinspires.ftc.teamcode.Roadrunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Follower.Follower;
import org.firstinspires.ftc.teamcode.ourOpModes.VuforiaPhone;
import org.firstinspires.ftc.teamcode.ourOpModes.resources.IMU;
import org.firstinspires.ftc.teamcode.ourOpModes.resources.Timing;
import org.firstinspires.ftc.teamcode.RobotParts.Wobble_Arm;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "RingFindingTest", group = "Auto")
public class RingFindingTest extends LinearOpMode {
    DcMotorEx shooter, taco;
    DcMotor intake;
    CRServo shooter_roller1, shooter_roller2;
    Wobble_Arm wobble_arm;
    Timing timer = new Timing(this);
    IMU imu;
    int sweepDir = 1;

    Detection detector;
    int cameraMonitorViewId;
    OpenCvCamera webcam;

    @Override
    public void runOpMode() throws InterruptedException {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().
                getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvCamera webcam = OpenCvCameraFactory.getInstance().
                createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        Detection detector = new Detection(telemetry);


        webcam.setPipeline(detector);

        //Opening and Streaming from Camera

        webcam.openCameraDeviceAsync(() -> {
            webcam.startStreaming(352, 288, OpenCvCameraRotation.UPRIGHT);
        });


        VuforiaPhone vuforia = new VuforiaPhone(hardwareMap, telemetry);
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        taco = hardwareMap.get(DcMotorEx.class, "taco");
        intake = hardwareMap.get(DcMotor.class, "intake");

        shooter_roller1 = hardwareMap.get(CRServo.class, "shooter_roller1");
        shooter_roller2 = hardwareMap.get(CRServo.class, "shooter_roller2");
        shooter_roller2.setDirection(DcMotorSimple.Direction.REVERSE);
        wobble_arm = new Wobble_Arm(hardwareMap, RingFindingTest.this);

        //intake.setPower(1.0);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        IMU imu = new IMU(hardwareMap, telemetry);

        Follower follower = new Follower(drive, vuforia, this, telemetry, gamepad1, imu);


        telemetry.addData("Autonomous", "Hold A for manual control");
        telemetry.update();



        waitForStart();

        while(opModeIsActive()){
            double targetHeading = imu.getHeading() - detector.angle;
            for(int j = 2; j > 0; j--) {
                while(detector.distance == 0 && !gamepad1.b){
                    follower.goToHeading(0.15 * sweepDir);
                    sweepDir *= -1;
                    timer.safeDelay(1000);
                }
                targetHeading = imu.getHeading() - detector.angle;
                for (int i = 0; i < 3; i++) {
                    follower.goToHeading(targetHeading);
                    timer.safeDelay(200);
                }
                follower.DRIVE_MAINTAIN_HEADING(0.4, 0, imu.getHeading(),
                        detector.distance * 53/j, imu);
            }
            timer.safeDelay(200);
        }

        webcam.stopStreaming();
    }

}
