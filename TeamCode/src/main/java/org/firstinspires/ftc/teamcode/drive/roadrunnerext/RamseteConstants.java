package org.firstinspires.ftc.teamcode.drive.roadrunnerext;

import com.acmerobotics.dashboard.config.Config;

@Config
public class RamseteConstants {
    // b is horizontal error
    public static double b = 27.5;
    // zeta is dampening for horizontal error
    public static double zeta = 0.7;
    // phase lag for actual thing, proportional
    public static double kLinear = 0.15;
    // Ramsette doesn't account for heading error, this is for accounting for like
    // doesn't turn fast enough
//    public static double kHeading = 1.2;
    public static double kHeading = 0.7;
}
