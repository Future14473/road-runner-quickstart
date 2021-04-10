package org.firstinspires.ftc.teamcode.RobotParts;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ourOpModes.resources.Timing;

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

    public void shortUp(){
        shortStyx.setPosition(-0.5);
    }

    public void shortDown(){
        shortStyx.setPosition(1);
    }

    public void longUp(){
        longStyx.setPosition(0.5);
    }

    public void longDown(){
        longStyx.setPosition(-1);
    }
}

