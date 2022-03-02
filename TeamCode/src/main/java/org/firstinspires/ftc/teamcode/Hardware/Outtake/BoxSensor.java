package org.firstinspires.ftc.teamcode.Hardware.Outtake;

import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class BoxSensor {
    NormalizedColorSensor colorSensor;
    NormalizedRGBA colors;
    SwitchableLight light;

    public static double emptyDistance = 6.0;
    public static double dumperHue = 180,
            goldHueHigh = 80, goldHueLow = 60,
            whiteHueLow = 150, whiteHueHigh = 170,
            duckHueHigh = 110, duckHueLow = 60;


    final float[] hsvValues = new float[3];
    float hue;

    float gain = 1;

    public BoxSensor(HardwareMap hardwareMap){
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");
        colorSensor.setGain(gain);
    }

    public boolean hasBlock(){
        return ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM) < emptyDistance;
    }

    public String getColor(){
        colors = colorSensor.getNormalizedColors();
        Color.colorToHSV(colors.toColor(), hsvValues);
        hue = hsvValues[0];

        if ((hue > goldHueLow) && (hue < goldHueHigh)) return "gold";
        if ((hue > whiteHueLow) && (hue < whiteHueHigh)) return "silver";
        if ((hue > duckHueLow) && (hue < duckHueHigh)) return "duck";
        return "none";
    }

}
