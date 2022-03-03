package org.firstinspires.ftc.teamcode.Hardware.Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.Outtake.Dumper;
import org.firstinspires.ftc.teamcode.Hardware.Outtake.Linkages;
import org.firstinspires.ftc.teamcode.Hardware.Outtake.Turret;

@TeleOp
@Disabled
public class FullTurretTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Turret turret = new Turret(hardwareMap, this);
        Dumper dumper = new Dumper(hardwareMap);
        Linkages linkages = new Linkages(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.x) {
                dumper.intake();
            }
            if (gamepad1.y) {
                dumper.close();
            }
            if (gamepad1.a) {
                dumper.dump();
            }

            if (gamepad1.dpad_up) {
                linkages.extend();
            }
            if (gamepad1.dpad_down) {
                linkages.retract();
            }
            if (gamepad1.right_bumper) {
                linkages.increment();
            }
            if (gamepad1.left_bumper) {
                linkages.decrement();
            }
        }
    }
}
