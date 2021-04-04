package org.firstinspires.ftc.teamcode.ourOpModes.resources;

public class MathStuff {

    public static boolean isEqual(double position, double flickIn) {
        return Math.abs(position - flickIn) < 0.1;
    }
}
