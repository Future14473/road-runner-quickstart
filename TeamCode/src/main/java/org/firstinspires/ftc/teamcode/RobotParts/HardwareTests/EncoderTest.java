package org.firstinspires.ftc.teamcode.RobotParts.HardwareTests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotParts.EncoderMecanum;

@Config
public class EncoderTest extends LinearOpMode {
    public static double forwardIn = 1;
    public static double strafeIn = 1;
    public static double turnDegrees = 90;
    @Override
    public void runOpMode() throws InterruptedException {

        EncoderMecanum drivetrain = new EncoderMecanum(hardwareMap);
        drivetrain.moveInches(forwardIn,strafeIn,turnDegrees);
    }
}

