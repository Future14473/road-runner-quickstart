//package org.firstinspires.ftc.teamcode.ComputerVision;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//
//public class TestCVUG extends LinearOpMode {
//    public void runOpMode() throws InterruptedException {
//
//        //setup RC for display
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().
//                getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        int[] viewportContainerIds = OpenCvCameraFactory.getInstance().splitLayoutForMultipleViewports(cameraMonitorViewId,
//                2, OpenCvCameraFactory.ViewportSplitMethod.HORIZONTALLY);
//
//        //OpenCV Setup
//        OpenCvCamera webcam = OpenCvCameraFactory.getInstance().
//                createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), viewportContainerIds[1]);
//
//        Detection detector = new Detection(telemetry);
//
//        webcam.setPipeline(detector);
//
//        webcam.openCameraDeviceAsync(() -> {
//            webcam.startStreaming(352, 288, OpenCvCameraRotation.UPRIGHT);
//        });
//
//        FtcDashboard.getInstance().startCameraStream(webcam, 0);
//
//        waitForStart();
//
//
//
//
//        webcam.stopStreaming();
//
//
//        boxes(detector);
//
//
//        //vuforia scan position
//        goTo(before_stack_x, before_stack_y, Math.toRadians(20));
//
//        wobble_arm.down();
//        wobble_arm.unGrab();
//
//        //grab wobble position
//        goTo(grab_wobble_x,grab_wobble_y, 0);
//
//        wobble_arm.grab();
//        new Timing(this).safeDelay(500);
//        wobble_arm.up();
//        new Timing(this).safeDelay(500);
//
//        boxes(detector);
//
//        wobble_arm.home();
//
//        goTo(12, 24, 0);
//
//        new Timing(this).safeDelay(1000);
//
//
//    }
//}
