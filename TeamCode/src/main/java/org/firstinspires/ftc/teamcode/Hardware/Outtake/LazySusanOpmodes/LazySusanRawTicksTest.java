package org.firstinspires.ftc.teamcode.Hardware.Outtake.LazySusanOpmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

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
