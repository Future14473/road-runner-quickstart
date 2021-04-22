package org.firstinspires.ftc.teamcode.RobotParts;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ourOpModes.resources.Timing;

public class ShooterFlicker {
    Servo flicker;
    Timing timer;
    LinearOpMode opMode;
    Telemetry telemetry;

    public ShooterFlicker(HardwareMap hardwareMap, LinearOpMode opMode, Telemetry telemetry) {
        flicker = hardwareMap.get(Servo.class, "flicker");
        timer = new Timing(opMode);
        this.opMode = opMode;
        this.telemetry = telemetry;
    }

    public double flickIn = 0.52, flickOut = 0.36;

    public void flickIn() { // left bumper
        flicker.setPosition(flickIn);
    }

    public void flickOut() { // right bumper
        flicker.setPosition(flickOut);
    }

    public double getPosition() {
        return flicker.getPosition();
    }

    // try to add a overcomplicated wait block
//    private void waitTillIn() {
//        while (!(MathStuff.isEqual(flicker.getPosition(), flickIn))
//                && opMode.opModeIsActive()) {
//            telemetry.addData("Stuck in ", "wait till in");
//            telemetry.update();
//            Log.e("Stuck in ", "wait till in");
//        }
//
//    }
//
//    private void waitTillOut() {
//        while (!(MathStuff.isEqual(flicker.getPosition(), flickOut))
//                && opMode.opModeIsActive()) {
//            telemetry.addData("Stuck in ", "wait till out");
//            telemetry.update();
//            Log.e("Stuck in ", "wait till out");
//        }
//    }

    int waitTime = 1000;

    public void flickThrice() {
        for (int i = 0; i < 3; i++) {
            flickOut();
            timer.safeDelay(waitTime);
            flickIn();
            timer.safeDelay(waitTime);
        }
    }

    public void singleFlick(){
        flickOut();
        timer.safeDelay(waitTime);
        flickIn();
        timer.safeDelay(200);
    }

}

