package org.firstinspires.ftc.teamcode.Hardware.Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.Duck.Duck;
import org.firstinspires.ftc.teamcode.Hardware.util.Timer;

public class DuckTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Duck duck = new Duck(hardwareMap);
        waitForStart();
        duck.autoDuckBlue(new Timer(this));
    }
}
