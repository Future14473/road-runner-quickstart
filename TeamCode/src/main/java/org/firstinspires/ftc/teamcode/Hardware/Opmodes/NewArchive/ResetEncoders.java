package org.firstinspires.ftc.teamcode.Hardware.Opmodes.NewArchive;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.Outtake.Turret;
import org.firstinspires.ftc.teamcode.Hardware.util.Timer;

@TeleOp
@Disabled
public class ResetEncoders extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Turret turret = new Turret(hardwareMap,this);

        waitForStart();
        while (opModeIsActive()){
            turret.resetEncoders();
            telemetry.addData("Reset", "done");
            telemetry.update();
        }

    }
}
