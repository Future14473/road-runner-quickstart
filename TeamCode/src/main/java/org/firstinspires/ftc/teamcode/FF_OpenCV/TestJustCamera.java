package org.firstinspires.ftc.teamcode.FF_OpenCV;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(group = "AAA")
public class TestJustCamera extends LinearOpMode {
    OpenCvCamera phoneCam;
    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().
                getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().
                createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        JustCameraStream cameraStream = new JustCameraStream(telemetry);
        phoneCam.setPipeline(cameraStream);
//        phoneCam.openCameraDeviceAsync( () -> {
//            phoneCam.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
//        });

        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }

        });
//        FtcDashboard.getInstance().startCameraStream(phoneCam, 0);
        waitForStart();
        while (opModeIsActive()){
            telemetry.addData("running", true);
        }
    }
}
