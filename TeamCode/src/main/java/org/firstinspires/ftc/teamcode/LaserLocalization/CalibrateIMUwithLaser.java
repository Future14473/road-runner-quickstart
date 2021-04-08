package org.firstinspires.ftc.teamcode.LaserLocalization;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Roadrunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.ourOpModes.resources.IMU;

import java.util.ArrayList;
import java.util.List;

public class CalibrateIMUwithLaser {

    /**
     * Uses the Lasers to calibrate the imu so that 0 is exactly forward (facing goals)
     * Start the robot near the center of the field, about 30 degrees left of forward
     * @param start_ang -1 means left of "forward" direction. 1 means right of forward
     */
    public static void calibrate(int start_ang, IMU imu, ModernRoboticsI2cRangeSensor range_front, ModernRoboticsI2cRangeSensor range_back, Telemetry telemetry, SampleMecanumDrive drive){
        List<Double> lengths_record = new ArrayList<>();
        double least_length = Double.MAX_VALUE;
        double angle_at_least_length = Double.NaN;

        DRIVE(0, 0, -Math.signum(start_ang) * 0.4, drive);

        int increasing_trend = 0;
        while (increasing_trend < 13){
            double front = (range_back.getDistance(DistanceUnit.INCH) / 12.0)/ 0.915 + 5.15 / 12.0;
            double back = range_back.getDistance(DistanceUnit.INCH) / 12.0 / 0.915 + 8.85 / 12.0;

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
        DRIVE(0, 0, 0, drive);

//        if(least_length < 11.8 || least_length > 12.2)
//            calibrate(1, imu, range_front, range_back, telemetry, drive);
//        else
            imu.thisHeadingIsActually(angle_at_least_length, 0);
    }

    static double average(List<Double> l){
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
    public static void DRIVE(double forward, double sideways, double rotate, SampleMecanumDrive drive) {
        drive.setWeightedDrivePower(
                new Pose2d(
                        forward,
                        sideways, // note the drivetrain wheels are reversed so sideways is positive right
                        -rotate * 0.7
                )
        );
    }
}
