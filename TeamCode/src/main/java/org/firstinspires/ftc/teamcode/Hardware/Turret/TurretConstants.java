package org.firstinspires.ftc.teamcode.Hardware.Turret;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
public class TurretConstants {
    static double motorTeeth = 14;
    static double motorToBevelTeeth = 28;
    static double normalGearToTurretTeeth = 30;
    static double turretTeeth = 225;
    static double maxRotationDegrees = 360;

    // feedforward constants
    private static final double MAX_RPM = 1620;
    public static double kA = 0; // for the phase shift (if its too early or too late)
    public static double kV = 1.0 / rpmToVelocity(MAX_RPM); // for the proportional (if it is a certain distance off from target
    public static double kStatic = 0; // rarely used but is vertical shifts


    static double LAZY_SUSAN_TICKS_PER_REVOLUTION = 103.8;



    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(0, 0, 0,
            getMotorVelocityF(MAX_RPM / 60 * LAZY_SUSAN_TICKS_PER_REVOLUTION));

    static double MOTOR_ROTATIONS_PER_TURRET_ROTATIONS =
            (turretTeeth/ normalGearToTurretTeeth) *(motorToBevelTeeth/motorTeeth);
            //(turretTeeth / (bevelToTurretTeeth / (motorToBevelTeeth / motorTeeth)));

    public static double MAX_VEL = 30;
    public static double MAX_ACCEL = 30;
    public static double MAX_ANG_VEL = Math.toRadians(60);
    public static double MAX_ANG_ACCEL = Math.toRadians(60);

    // math should be output/input
    public static int turretDegreesToTicks(double degrees){
        return (int)( (degrees / 360) //  turret  rotations
                * MOTOR_ROTATIONS_PER_TURRET_ROTATIONS * LAZY_SUSAN_TICKS_PER_REVOLUTION);
    }

    public static double rpmToVelocity(double rpm) {
        return rpm * (1/MOTOR_ROTATIONS_PER_TURRET_ROTATIONS) / 60.0;
    }

    public static double getMotorVelocityF(double ticksPerSecond) {
        // see https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit#heading=h.61g9ixenznbx
        return 32767 / ticksPerSecond;
    }

}
