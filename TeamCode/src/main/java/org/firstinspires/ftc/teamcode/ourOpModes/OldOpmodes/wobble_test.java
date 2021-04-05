package org.firstinspires.ftc.teamcode.ourOpModes.OldOpmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotParts.Wobble_Arm;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(group = "drive")
public class wobble_test extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Wobble_Arm wobble_arm = new Wobble_Arm(hardwareMap, this);

        waitForStart();
        while (opModeIsActive()){
            telemetry.addData("Wobble angler", wobble_arm.getAnglerPosition());
            telemetry.update();
            if (gamepad1.a) {
                wobble_arm.down();
            }
            if (gamepad1.b) {
                wobble_arm.up();
            }
            if (gamepad1.right_bumper) {
                wobble_arm.grab();
            }
            if (gamepad1.left_bumper) {
                wobble_arm.unGrab();
            }
        }
    }
}
