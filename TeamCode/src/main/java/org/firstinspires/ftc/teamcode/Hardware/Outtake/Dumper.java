package org.firstinspires.ftc.teamcode.Hardware.Outtake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Dumper {
    Servo dumper;

    public static double dumperOutPos = 0.7;
    public static double dumperIntakePos = 0.5;
    public static double dumperInPos = 0.2;
    public Dumper(HardwareMap hardwareMap) {
        dumper = hardwareMap.get(Servo.class, "dumper");
    }

    public void dump() {
        dumper.setPosition(dumperOutPos);
    }

    public void intake() {
        dumper.setPosition(dumperIntakePos);
    }

    public void close() {
        dumper.setPosition(dumperInPos);
    }
}
