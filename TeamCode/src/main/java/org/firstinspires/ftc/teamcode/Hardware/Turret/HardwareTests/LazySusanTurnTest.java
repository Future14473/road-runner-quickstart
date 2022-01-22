package org.firstinspires.ftc.teamcode.Hardware.Turret.HardwareTests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.Turret.LazySusan;

@TeleOp
public class LazySusanTurnTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        LazySusan lazySusan = new LazySusan(hardwareMap);
        waitForStart();

        lazySusan.rotateToDegrees(90); //todo check if tick values wrap for dcmotor
        while (opModeIsActive()){
            telemetry.addData("Turret Position", lazySusan.getTicks());
            telemetry.addData("Turret Degrees", lazySusan.getDegrees());
            telemetry.addData("Turret Target Position", lazySusan.getTargetTicks());
            telemetry.update();
        }
    }
}
