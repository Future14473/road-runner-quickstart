package org.firstinspires.ftc.teamcode.Hardware.Turret.HardwareTests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Hardware.Turret.LazySusan;

@TeleOp
public class LazySusanRawTicksTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx lazySusan = hardwareMap.get(DcMotorEx.class, "lazySusan");
        waitForStart();
        while (opModeIsActive()){
            telemetry.addData("Turret Raw Position", lazySusan.getCurrentPosition());
            telemetry.update();
        }
    }
}
