package org.firstinspires.ftc.teamcode.Mechanisms.HardwareTests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp(group = "!")
public class SingleMotorTest extends LinearOpMode {
    public static String motorName = "output";
    public static int velocity = 1000;
    public static int targetPos = 384;
    @Override
    public void runOpMode() throws InterruptedException {
        // set target position, run to position, velocity
        waitForStart();

        DcMotorEx test_motor = hardwareMap.get(DcMotorEx.class, motorName);
        test_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        test_motor.setTargetPosition(targetPos);
        while (opModeIsActive()){
        test_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        test_motor.setVelocity(velocity);
        telemetry.addData("Posistion", test_motor.getCurrentPosition());
        telemetry.update();
        }


    }
}
