package org.firstinspires.ftc.teamcode.Hardware.Outtake;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Hardware.Turret.LazySusan;

public class Outtake {
    public Linkages linkages;
    public Slides slides;
    public LazySusan lazySusan;

    public Outtake (HardwareMap hardwareMap){
        linkages = new Linkages(hardwareMap);
        slides = new Slides(hardwareMap);
        lazySusan = new LazySusan(hardwareMap);
    }
}
