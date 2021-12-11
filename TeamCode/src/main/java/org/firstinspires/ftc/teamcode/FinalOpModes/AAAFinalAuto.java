/*
 * Copyright (c) 2019 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.FinalOpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.ComputerVision.CapstonePipeline;
import org.firstinspires.ftc.teamcode.Mechanisms.Cycler;
import org.firstinspires.ftc.teamcode.Mechanisms.Duck;
import org.firstinspires.ftc.teamcode.Mechanisms.EncoderMecanum;
import org.firstinspires.ftc.teamcode.Mechanisms.Intake;
import org.firstinspires.ftc.teamcode.Mechanisms.Output;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
import org.firstinspires.ftc.teamcode.util.Timer;


@Autonomous
@Config
public class AAAFinalAuto extends LinearOpMode
{

    public static double
            duck1Y = 12.5, duck1X = -4,
            wobbleX = -40, wobbleY = -5,
            wobbleHighYOffset = -2,
            wobbleLowYOffset = 2,
            wobbleMidYOffset = -1,
            preParkX = -20;

    @Override
    public void runOpMode() {
        Duck duck = new Duck(hardwareMap);
        Timer timer = new Timer(this);
        Intake intake = new Intake(hardwareMap, timer);
        Cycler cycler = new Cycler(intake, new Output(hardwareMap), this);
        CapstonePipeline capstonePipeline = new CapstonePipeline(telemetry);

        //Hardware Setup
        EncoderMecanum encoderMecanum = new EncoderMecanum(hardwareMap, telemetry);


        // CV Setup
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvWebcam webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        webcam.setPipeline(capstonePipeline);
        FtcDashboard.getInstance().startCameraStream(webcam, 0);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
        telemetry.addData("DO NOT ", "INITIALIZE YET");
        telemetry.update();
        intake.flipOutAuto();

        waitForStart();
        CapstonePipeline.Location location = capstonePipeline.getLocation();

        webcam.stopStreaming();

        //remember it is forward, strafe so Y,X

        // Duck____________________________
        encoderMecanum.moveInchesConstantHeading(duck1Y, duck1X);

        duck.setAutoSpeed();
        duck.setSpeed();
        timer.safeDelay(Duck.autoDelayTime);
        duck.setStopSpeed();
        duck.setSpeed();

        // Wobble__________________________
        switch (location){
            case LEFT:
                cycler.dumperOutPrepLow();
                encoderMecanum.moveInchesConstantHeading(wobbleY + wobbleMidYOffset, wobbleX);
                break;
            case RIGHT:
                cycler.dumperOutPrepHigh();
                encoderMecanum.moveInchesConstantHeading(wobbleY + wobbleHighYOffset, wobbleX);
                break;
            case MIDDLE:
                cycler.dumperOutPrepMiddle();
                encoderMecanum.moveInchesConstantHeading(wobbleY + wobbleLowYOffset, wobbleX);
                break;
        }
        cycler.dumpRetractAuto();

        // PARK ____________________
        encoderMecanum.moveInches(wobbleY+wobbleLowYOffset, preParkX, 180);
    }
}