package org.firstinspires.ftc.teamcode.RobotParts;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ourOpModes.resources.Timing;

@Config
public class SideStyx {
    Servo shortStyx;
    Servo longStyx;
    LinearOpMode opMode;
    Telemetry telemetry;

    public SideStyx(HardwareMap hardwareMap, Telemetry telemetry) {
        shortStyx = hardwareMap.get(Servo.class, "shortStyx");
        longStyx = hardwareMap.get(Servo.class, "longStyx");
        this.telemetry = telemetry;
    }

    public void allUp() { // left bumper
        shortUp();
        longUp();
    }

    public void allDown() { // right bumper
        shortDown();
        longDown();
    }

    public static double down = -0.15, up = 0.5;

    public static double down_long = 0.4, up_long = 0.1;


    private void shortDown(){
        shortStyx.setPosition(0.5);
    }

    private void shortUp(){ shortStyx.setPosition(-0.15); }

    private void longDown(){
        longStyx.setPosition(down_long);
    }

    private void longUp(){ longStyx.setPosition(up_long); }
}

