package org.firstinspires.ftc.teamcode.Roadrunner;

import android.util.Log;

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

import java.lang.reflect.Field;
import java.util.Arrays;
import java.util.List;
import java.util.Objects;

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

        this.drive = drive;

        parallelEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "frontRight"));
        perpendicularEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "backRight"));

        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)

        perpendicularEncoder.setDirection(Encoder.Direction.REVERSE);
    }

    int vuforiaAvailableTimes = 0;
    // Vuforia and Laser position overrides
    @Override
    public void update() {
        double heading = drive.getIMU().getHeading();

        OpenGLMatrix vuLocation = DirtyGlobalVariables.vuforia.getLocation();
        if(vuLocation!=null)
            vuforiaAvailableTimes++;
        else
            vuforiaAvailableTimes = 0;

        boolean hasVuforia = vuLocation!=null && vuforiaAvailableTimes>5;

        // this only works when rr path is running, so not very useful
        //DirtyGlobalVariables.telemetry.addData("Pose Velocity", poseVel);

        DirtyGlobalVariables.telemetry.addData("In Path?", drive.following);

//        if(hasVuforia && !drive.following)
//            drive.getIMU().thisHeadingIsActually(heading, vuforia.matrixToPose(vuLocation).getHeading());

        if(drive.following){
//            if(hasVuforia){
//                doVuforia(vuLocation);
//            }else{
            DirtyGlobalVariables.telemetry.addData("Localization", "wheels");

            super.update();
//            }
        }else{
            if(hasVuforia){
                DirtyGlobalVariables.telemetry.addData("Localization", "vuforia");
                doVuforia(vuLocation);
            }else{
                DirtyGlobalVariables.telemetry.addData("Localization", "wheels");
                super.update();
                //doLaser(heading);
            }
        }
    }

    void setPoseEstimateForce(Pose2d pose){
        try {
            Field poseEstimate = this.getClass().getSuperclass().getDeclaredField("_poseEstimate");
            poseEstimate.setAccessible(true);
            poseEstimate.set(this, pose);
        } catch (NoSuchFieldException | IllegalAccessException e) {
            Log.d("EEEEEEEEERRRRRRRRRR", "reflection failed TwoWheelTrackingLocalizer line 133");
        }

    }

    void doVuforia(OpenGLMatrix vuLocation){
        DirtyGlobalVariables.telemetry.addData("Localization", "using Vuforia");
        Pose2d tempPose = DirtyGlobalVariables.vuforia.matrixToPose(vuLocation);
        this.setPoseEstimateForce(new Pose2d(tempPose.getX(), tempPose.getY(), this.getHeading()));
    }

    void doLaser(double heading){
        Pose2d laserPose = lasers.update(heading);
        if(laserPose != null && laserLocalization.isAccurate(heading)){
            DirtyGlobalVariables.telemetry.addData("Localization", "using Lasers");
            this.setPoseEstimateForce(laserPose);
        }else{
            DirtyGlobalVariables.telemetry.addData("Localization", "using Wheels");
            super.update();
        }
    }

    /*
    double poseVel = 0;
        if(drive.getPoseVelocity() != null)
            poseVel = Math.sqrt(
                        drive.getPoseVelocity().getX()*drive.getPoseVelocity().getX() +
                        drive.getPoseVelocity().getY()*drive.getPoseVelocity().getY() +
                        drive.getPoseVelocity().getHeading()*drive.getPoseVelocity().getHeading()
             );
     */
/*
    @Override
    public void update() {
        double heading = drive.getIMU().getHeading();

        DirtyGlobalVariables.telemetry.addData("Is following", drive.following);
        DirtyGlobalVariables.telemetry.addData("Is accurate", laserLocalization.isAccurate(heading));

        Pose2d laserPose = lasers.update(Objects.requireNonNull(heading, "Heading is null")); // do laser
        boolean hasLaser = !drive.following && laserLocalization.isAccurate(heading) && (laserPose != null);


        if(hasLaser) {
            DirtyGlobalVariables.telemetry.addData("Localization", "using Lasers");
            this.setPoseEstimate(Objects.requireNonNull(laserPose, "Laser Pose is null"));
        }
        else {
            DirtyGlobalVariables.telemetry.addData("Localization", "using WHEELS");
            super.update();
        }

//        DirtyGlobalVariables.telemetry.addData("Localization", "using WHEELS lasers disabled");
//        super.update();

    }*/

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