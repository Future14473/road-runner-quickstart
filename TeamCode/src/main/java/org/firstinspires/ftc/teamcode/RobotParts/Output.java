package org.firstinspires.ftc.teamcode.RobotParts;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Output {
    Servo bottomDumper;
    Servo topDumper;

    public static double bottomDumperOutPos = -1.0;
    public static double bottomDumperInPos = 1.0;
    public static double topDumperOutPos = -1.0;
    public static double topDumperInPos = 1.0;
    public static double maxVelocity = 100;

    public Output(HardwareMap hardwareMap) {
        bottomDumper = hardwareMap.get(Servo.class, "bottomDumper");
        topDumper = hardwareMap.get(Servo.class, "topDumper");
    }
    public void flipInDumper() {
        bottomDumper.setPosition(bottomDumperInPos);
        bottomDumper.setDirection(Servo.Direction.REVERSE);
        topDumper.setPosition(topDumperInPos);
        topDumper.setDirection(Servo.Direction.REVERSE);
    }
    public void flipOutDumper() {
        bottomDumper.setPosition(bottomDumperOutPos);
        bottomDumper.setDirection(Servo.Direction.REVERSE);
        topDumper.setPosition(topDumperOutPos);
        topDumper.setDirection(Servo.Direction.REVERSE);

    }

    public double getTopDumperPos() {
        return topDumper.getPosition();
    }
    public double getBottomDumperPos () {
        return bottomDumper.getPosition();
    }
}


