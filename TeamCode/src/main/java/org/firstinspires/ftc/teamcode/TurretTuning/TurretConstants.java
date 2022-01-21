package org.firstinspires.ftc.teamcode.TurretTuning;

public class TurretConstants {
    static double motorTeeth = 14;
    static double motorToBevelTeeth = 28;
    static double normalGearToTurretTeeth = 30;
    static double turretTeeth = 225;
    static double maxRotationDegrees = 360;

    static double LAZY_SUSAN_TICKS_PER_REVOLUTION = 103.8;

    static double MOTOR_ROTATIONS_PER_TURRET_ROTATIONS =
            (turretTeeth/ normalGearToTurretTeeth) *(motorToBevelTeeth/motorTeeth);
            //(turretTeeth / (bevelToTurretTeeth / (motorToBevelTeeth / motorTeeth)));

    // math should be output/input
    public static int turretDegreesToTicks(double degrees){
        return (int)( (degrees / 360) //  turret  rotations
                * MOTOR_ROTATIONS_PER_TURRET_ROTATIONS * LAZY_SUSAN_TICKS_PER_REVOLUTION);
    }
}
