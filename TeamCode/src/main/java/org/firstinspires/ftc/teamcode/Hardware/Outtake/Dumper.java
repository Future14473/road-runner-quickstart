package org.firstinspires.ftc.teamcode.Hardware.Outtake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Dumper {
    Servo dumper;

    public static double dumperOutPos = 0.6;
    public static double dumperHalfPos = 0.45;
    public static double dumperInPos = 0.2;
    public Dumper(HardwareMap hardwareMap) {
        dumper = hardwareMap.get(Servo.class, "dumper");
    }

    public void flipOutDumper() {
        dumper.setPosition(dumperOutPos);
    }

    public void flipHalfDumper() {
        dumper.setPosition(dumperHalfPos);
    }

    public void flipInDumper() {
        dumper.setPosition(dumperInPos);
    }

    public void dumperIn() {
        flipInDumper();
    }

    public void dumperOut() {
        flipOutDumper();
    }
}
