package org.firstinspires.ftc.teamcode.Hardware.Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.Intake.Intake;
import org.firstinspires.ftc.teamcode.Hardware.Outtake.Linkages;
import org.firstinspires.ftc.teamcode.Hardware.Outtake.Outtake;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class LinkageTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Linkages linkages = new Linkages(hardwareMap);
        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.x) {
                linkages.retractLeftExtender();
                telemetry.addData("retract, left", "!");
            }
            if (gamepad1.a) {
                linkages.retractRightExtender();
                telemetry.addData("retract, right", "!");
            }
            if (gamepad1.y) {
                linkages.extendRightExtender();
                telemetry.addData("extend, right", "!");
            }
            if (gamepad1.b) {
                linkages.extendLeftExtender();
                telemetry.addData("extend, left", "!");
            }
            telemetry.update();
        }
    }
}
