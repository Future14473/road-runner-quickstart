package org.firstinspires.ftc.teamcode.Mechanisms.HardwareTests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Mechanisms.RetractableOdo;

@TeleOp(group = "1 Teleop")
public class RetractableOdoTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RetractableOdo retractableOdo = new RetractableOdo(hardwareMap);
        waitForStart();

        while (opModeIsActive()){


            if (gamepad1.x){
                retractableOdo.upOdo();
                telemetry.addData("Retractable Odo Status ", "Up");
            }
            if (gamepad1.y){
                retractableOdo.downOdo();
                telemetry.addData("Retractable Odo Status ", "Down");
            }

            telemetry.addData("Retractable Odo Position Perp", retractableOdo.getPerpPos());
            telemetry.addData("Retractable Odo Position Parallel", retractableOdo.getParallelPos());

            telemetry.update();
        }
    }
}
