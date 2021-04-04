package org.firstinspires.ftc.teamcode.ourOpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Follower.Follower;
import org.firstinspires.ftc.teamcode.LaserLocalization.DistanceSensorMath;
import org.firstinspires.ftc.teamcode.LaserLocalization.intersection;
import org.firstinspires.ftc.teamcode.Roadrunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.ourOpModes.resources.IMU;

import java.util.ArrayList;
import java.util.List;


@TeleOp(name = "Localization test", group = "Sensor")
public class MRSensortest extends LinearOpMode {

    ModernRoboticsI2cRangeSensor range_left;
    ModernRoboticsI2cRangeSensor range_right;
    ModernRoboticsI2cRangeSensor range_front;
    ModernRoboticsI2cRangeSensor range_back;

    IMU imu;

    SampleMecanumDrive drive;

    @Override public void runOpMode() {

        // get a reference to our compass
        range_left = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range_left");
        range_right = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range_right");
        range_front = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range_front");
        range_back = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range_back");

        imu = new IMU(hardwareMap, telemetry);

        drive = new SampleMecanumDrive(hardwareMap);

        imu_calibration_procedure(-1, imu);

        // wait for the start button to be pressed
        waitForStart();

        new Follower(drive, null, this, telemetry, gamepad1, imu).goToHeading(0);

        while (opModeIsActive()) {
            //0.91 is experimentally determined deviation
            double left = range_left.getDistance(DistanceUnit.INCH) / 12.0 / 0.915 + 7.5 / 12.0;
            double right = range_right.getDistance(DistanceUnit.INCH) / 12.0 / 0.915 + 7.5 / 12.0;
            double front = range_front.getDistance(DistanceUnit.INCH) / 12.0 / 0.915 + 5.15 / 12.0;
            double back = range_back.getDistance(DistanceUnit.INCH) / 12.0 / 0.915 + 9 / 12.0;

            List<intersection> results = DistanceSensorMath.distanceToPosition(left, right, front, back);

            if(results.size() > 0)
                telemetry.addData("position", String.format("%.2f %.2f", results.get(0).a.x, results.get(0).a.y));
            else
                telemetry.addData("position", "undetermined");

            telemetry.addData("length", front + back);
            telemetry.addData("heading rad", Math.toDegrees(imu.getHeading()));

            telemetry.update();

            DRIVE(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x * 0.6);
        }
    }

    // param: the sum of the front and back distance sensors
    // return: the angle of the robot in rads, where 0 is forward
    double length_to_angle(double length){
        return Math.PI/2 + Math.atan(12.0 / Math.sqrt(length*length - 12*12));
    }

    /**
     * Uses the Lasers to calibrate the imu so that 0 is forward
     * Start the robot near the center of the field, about 30 degs from forward
     * @param start_ang -1 means left of "forward" direction. 1 means right of forward
     */
    void imu_calibration_procedure(int start_ang, IMU imu){
        List<Double> lengths_record = new ArrayList<>();
        double least_length = Double.MAX_VALUE;
        double angle_at_least_length = Double.NaN;

        DRIVE(0, 0, -Math.signum(start_ang) * 0.4);

        int increasing_trend = 0;
        while (increasing_trend < 13){
            double front = range_left.getDistance(DistanceUnit.INCH) / 12.0 / 0.915 + 7.5 / 12.0;
            double back = range_right.getDistance(DistanceUnit.INCH) / 12.0 / 0.915 + 7.5 / 12.0;

            // new minumum
            if(front + back < least_length){
                least_length = front + back;
                angle_at_least_length = imu.getHeading();
            }

            double former_average = average(lengths_record);

            lengths_record.add(front + back);
            if(lengths_record.size() > 20)
                lengths_record.remove(0);

            double new_average = average(lengths_record);

            if(new_average > former_average + 0.001)
                increasing_trend ++;

            telemetry.addData("IMU calibration: current min", least_length);
            telemetry.addData("IMU calibration: current avg", new_average);
            telemetry.addData("IMU calibration: increasing cnt", increasing_trend);
            telemetry.update();
        }
        DRIVE(0, 0, 0);

        if(least_length < 7.8 || least_length > 8.2)
            imu_calibration_procedure(1, imu);
        else
            imu.setPreviousHeadingTo(angle_at_least_length, 0);
    }

    double average(List<Double> l){
        double average = 0;
        for(double val : l)
            average += val * 1.0/l.size();
        return average;
    }

    /*
     * For POSITIVE forward parameter: go forward
     * For POSITIVE sideways parameter: go right
     * For POSITIVE rotate parameter: turn right
     */
    public void DRIVE(double forward, double sideways, double rotate) {
        drive.setWeightedDrivePower(
                new Pose2d(
                        forward,
                        sideways, // note the drivetrain wheels are reversed so sideways is positive right
                        -rotate * 0.7
                )
        );
    }

}

