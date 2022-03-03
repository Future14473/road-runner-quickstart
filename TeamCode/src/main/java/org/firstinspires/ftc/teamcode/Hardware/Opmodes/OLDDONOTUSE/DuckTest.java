package org.firstinspires.ftc.teamcode.Hardware.Opmodes.OLDDONOTUSE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.teamcode.Hardware.Duck.Duck;
import org.firstinspires.ftc.teamcode.Hardware.util.Timer;

@Autonomous
@Disabled
public class DuckTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Duck duck = new Duck(hardwareMap);
        waitForStart();
        duck.autoDuckBlue(new Timer(this));
    }
}
