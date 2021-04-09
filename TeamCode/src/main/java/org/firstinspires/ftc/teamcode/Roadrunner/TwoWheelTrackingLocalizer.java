package org.firstinspires.ftc.teamcode.Roadrunner;

import android.graphics.RadialGradient;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.LaserLocalization.DistanceSensorAlt;
import org.firstinspires.ftc.teamcode.LaserLocalization.point;
import org.firstinspires.ftc.teamcode.LaserLocalization.scaleGraphics;
import org.firstinspires.ftc.teamcode.ourOpModes.VuforiaPhone;
import org.firstinspires.ftc.teamcode.ourOpModes.resources.RotationUtil;
import org.firstinspires.ftc.teamcode.util.Encoder;

import java.util.Arrays;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

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

    ModernRoboticsI2cRangeSensor range_left;
    ModernRoboticsI2cRangeSensor range_right;
    ModernRoboticsI2cRangeSensor range_front;
    ModernRoboticsI2cRangeSensor range_back;

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

        vuforia = new VuforiaPhone(hardwareMap);
        vuforia.beginTracking();

        range_left = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range_left");
        range_right = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range_right");
        range_front = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range_front");
        range_back = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range_back");

        this.drive = drive;

        parallelEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "frontRight"));
        perpendicularEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "backRight"));

        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)

        perpendicularEncoder.setDirection(Encoder.Direction.REVERSE);
    }

    // Insert Vuforia and Laser position overrides
    @Override
    public void update() {
        OpenGLMatrix vuLocation = vuforia.getLocation();
        if(vuLocation != null){
            VectorF vuTranslation = vuLocation.getTranslation();
            Orientation vuRotation = Orientation.getOrientation(vuLocation, EXTRINSIC, XYZ, RADIANS);
            Pose2d vuPose = new Pose2d(vuTranslation.get(0) / mmPerInch - 12 / mmPerInch,
                    vuTranslation.get(1) / mmPerInch, vuRotation.thirdAngle);
            this.setPoseEstimate(vuPose);
        }else {

            }else{
                super.update();
            }

        }
    }

    point getPosLaser() {
        double left = range_left.getDistance(DistanceUnit.INCH) / 12.0 / 0.915 + 7.5 / 12.0;
        double right = range_right.getDistance(DistanceUnit.INCH) / 12.0 / 0.915 + 7.5 / 12.0;
        double front = range_front.getDistance(DistanceUnit.INCH) / 12.0 / 0.915 + 5.15 / 12.0;
        double back = range_back.getDistance(DistanceUnit.INCH) / 12.0 / 0.915 + 9 / 12.0;

        double heading = imu.getHeading();

        DistanceSensorAlt.geom position = DistanceSensorAlt.calculate_location(left, right, front, back, heading, new scaleGraphics());

        if (position instanceof point) {
            //if [45, -135], reflect over y and x axes DON'T ASK ME WHY IT JUST WORKS
            heading = RotationUtil.mod(heading, 2 * Math.PI);
            if (heading > Math.toRadians(315) || heading < Math.toRadians(135))
                position.scale(4, 6, -1, -1);

            point p = (point) position;
            this.setPoseEstimate(new Pose2d(p.x, p.y, heading));
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