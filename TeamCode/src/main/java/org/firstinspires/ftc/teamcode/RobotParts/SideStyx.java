package org.firstinspires.ftc.teamcode.RobotParts;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ourOpModes.resources.Timing;

@Config
public class SideStyx {
    public Servo shortStyx;
    public Servo longStyx;
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

    public static double down = 1, up = -0.3;

    public static double  down_long = 0.4, up_long = 0.1;


    public void shortDown(){
        shortStyx.setPosition(down);
    }

    public void shortUp(){ shortStyx.setPosition(up); }

    public void longDown(){
        longStyx.setPosition(down_long);
    }

    public void longUp(){ longStyx.setPosition(up_long); }
}

