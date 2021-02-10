package org.firstinspires.ftc.teamcode.ourOpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;

@TeleOp(name="Vuforia Nav", group ="Concept")
public class VuforiaOpMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        Vuforia vuforia = new Vuforia(hardwareMap, telemetry);

        waitForStart();

        vuforia.beingTracking();

        while (!isStopRequested()){
            OpenGLMatrix location = vuforia.getLocation();
            vuforia.printLocation(location);
        }

        vuforia.stopTracking();
    }
}
