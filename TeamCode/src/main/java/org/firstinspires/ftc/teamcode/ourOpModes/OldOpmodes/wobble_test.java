package org.firstinspires.ftc.teamcode.ourOpModes.OldOpmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.RobotParts.Wobble_Arm;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(group = "teleop")
public class wobble_test extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx wobble_arm = hardwareMap.get(DcMotorEx.class, "wobble_angler");
        wobble_arm.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);


        waitForStart();
        while (opModeIsActive()){
            wobble_arm.setPower(10);
            telemetry.addData("Velocity", wobble_arm.getVelocity());
            telemetry.addData("Wobble angler", wobble_arm.getCurrentPosition());
            telemetry.update();

            if (gamepad1.a) {
                wobble_arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                wobble_arm.setTargetPosition(0);
            }
            if (gamepad1.b) {
                wobble_arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                wobble_arm.setTargetPosition(700);
            }

        }
    }
}
