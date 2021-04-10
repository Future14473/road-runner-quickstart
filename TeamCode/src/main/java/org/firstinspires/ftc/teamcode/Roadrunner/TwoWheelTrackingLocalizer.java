package org.firstinspires.ftc.teamcode.Roadrunner;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.teamcode.LaserLocalization.laserLocalization;
import org.firstinspires.ftc.teamcode.RobotParts.VuforiaPhone;
import org.firstinspires.ftc.teamcode.ourOpModes.DirtyGlobalVariables;
import org.firstinspires.ftc.teamcode.util.Encoder;

import java.util.Arrays;
import java.util.List;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    ^
 *    |
 *    | ( x direction)
 *    |
 *    v
 *    <----( y direction )---->
 *        (forward)
 *    /--------------\
 *    |     ____     |
 *    |     ----     |    <- Perpendicular Wheel
 *    |           || |
 *    |           || |    <- Parallel Wheel
 *    |              |
 *    |              |
 *    \--------------/
 *
 */
public class TwoWheelTrackingLocalizer extends TwoTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = 0.68897638; // in
    public static double GEAR_RATIO = 1;

    public static double PARALLEL_X = -0.6; // X is the up and down direction
    public static double PARALLEL_Y = -7.45; // Y is the strafe direction

    public static double PERPENDICULAR_X = -6.5;
    public static double PERPENDICULAR_Y = -1.34;//-1.375;

    public static double X_MULTIPLIER = 1.0174; // Multiplier in the X direction
    public static double Y_MULTIPLIER = 1.0175; // Multiplier in the Y direction
    private static final float mmPerInch        = 25.4f;

    VuforiaPhone vuforia;
    laserLocalization lasers;

    // Parallel/Perpendicular to the forward axis
    // Parallel wheel is parallel to the forward axis
    // Perpendicular is perpendicular to the forward axis
    private Encoder parallelEncoder, perpendicularEncoder;

    private SampleMecanumDrive drive;

    public TwoWheelTrackingLocalizer(HardwareMap hardwareMap, SampleMecanumDrive drive) {
        super(Arrays.asList(
                new Pose2d(PARALLEL_X, PARALLEL_Y, 0),
                new Pose2d(PERPENDICULAR_X, PERPENDICULAR_Y, Math.toRadians(90))
        ));

        lasers = new laserLocalization(hardwareMap);
        vuforia = new VuforiaPhone(hardwareMap);
        vuforia.beginTracking();

        this.drive = drive;

        parallelEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "frontRight"));
        perpendicularEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "backRight"));

        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)

        perpendicularEncoder.setDirection(Encoder.Direction.REVERSE);
    }

    // Insert Vuforia and Laser position overrides
    @Override
    public void update() {
        double heading = drive.getIMU().getHeading();

        OpenGLMatrix vuLocation = vuforia.getLocation();

        boolean hasVuforia = vuLocation!=null;
        boolean hasLaser = lasers.isAccurate(heading);

        if(hasVuforia && !hasLaser){
            DirtyGlobalVariables.telemetry.addData("Localization", "using Vuforia");
            this.setPoseEstimate(vuforia.matrixToPose(vuLocation));
        }

        if(!hasVuforia && hasLaser){
            Pose2d laserPose = lasers.update(heading);
            if(laserPose != null){
                DirtyGlobalVariables.telemetry.addData("Localization", "using Lasers");
                this.setPoseEstimate(laserPose);
            }else{
                DirtyGlobalVariables.telemetry.addData("Localization", "using Wheels");
                super.update();
            }
        }

        if(hasVuforia && hasLaser){
            //*
//            Pose2d laserPose = lasers.update(heading); // do laser
//            if(laserPose != null){
//                DirtyGlobalVariables.telemetry.addData("Localization", "using Lasers");
//                this.setPoseEstimate(laserPose);
//            }else{
//                DirtyGlobalVariables.telemetry.addData("Localization", "using Wheels");
//                super.update();
//            }

            DirtyGlobalVariables.telemetry.addData("Localization", "using Vuforia");
            this.setPoseEstimate(vuforia.matrixToPose(vuLocation)); // do Vuforia

        }

        if(!hasVuforia && !hasLaser){
            super.update();
        }
    }

    //to [0, 360]
    double to360(double ang, double cycleSize){
        if(ang < 0){
            return cycleSize - to360(-ang, cycleSize);
        }else{
            return ang%cycleSize;
        }
    }


    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @Override
    public double getHeading() {
        return drive.getRawExternalHeading();
    }

    @Override
    public Double getHeadingVelocity() {
        return drive.getExternalHeadingVelocity();
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(parallelEncoder.getCurrentPosition()) * X_MULTIPLIER,
                encoderTicksToInches(perpendicularEncoder.getCurrentPosition()) * Y_MULTIPLIER
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        // TODO: If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
        //  competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
        //  compensation method

        return Arrays.asList(
                encoderTicksToInches(parallelEncoder.getCorrectedVelocity()) * X_MULTIPLIER,
                encoderTicksToInches(perpendicularEncoder.getCorrectedVelocity()) * Y_MULTIPLIER
        );
    }
}