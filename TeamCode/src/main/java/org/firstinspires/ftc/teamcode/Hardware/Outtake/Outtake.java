package org.firstinspires.ftc.teamcode.Hardware.Outtake;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Outtake {
    public Linkages linkages;
    public Slides slides;

    public Outtake (HardwareMap hardwareMap){
        linkages = new Linkages(hardwareMap);
        slides = new Slides(hardwareMap);
    }
}
