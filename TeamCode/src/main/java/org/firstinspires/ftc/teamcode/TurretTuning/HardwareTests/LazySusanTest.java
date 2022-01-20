package org.firstinspires.ftc.teamcode.TurretTuning.HardwareTests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.TurretTuning.Turret;

@TeleOp
public class LazySusanTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Turret turret = new Turret(hardwareMap);
        waitForStart();
        turret.rotateDegrees(90);
        while (opModeIsActive()){
            telemetry.addData("Turret Position", )
        }
    }
}
