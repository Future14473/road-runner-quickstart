package org.firstinspires.ftc.teamcode.Hardware.Opmodes.OLDDONOTUSE;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.Outtake.Turret;

@TeleOp
public class DebugSoftware extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Turret turret = new Turret(hardwareMap, this);

        waitForStart();
        turret.preloadLow();
        while (opModeIsActive()){

        }

    }
}
