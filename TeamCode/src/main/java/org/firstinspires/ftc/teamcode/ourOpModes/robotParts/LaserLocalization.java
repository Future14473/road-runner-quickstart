package org.firstinspires.ftc.teamcode.ourOpModes.robotParts;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.LaserLocalization.DistanceSensorAlt;
import org.firstinspires.ftc.teamcode.LaserLocalization.point;
import org.firstinspires.ftc.teamcode.LaserLocalization.scaleGraphics;
import org.firstinspires.ftc.teamcode.ourOpModes.resources.IMU;
import org.firstinspires.ftc.teamcode.ourOpModes.resources.RotationUtil;

public class LaserLocalization {
    ModernRoboticsI2cRangeSensor range_left;
    ModernRoboticsI2cRangeSensor range_right;
    ModernRoboticsI2cRangeSensor range_front;
    ModernRoboticsI2cRangeSensor range_back;

    IMU imu;

    public LaserLocalization(IMU imu, HardwareMap hardwareMap){
        range_left = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range_left");
        range_right = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range_right");
        range_front = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range_front");
        range_back = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range_back");

        imu = new IMU(hardwareMap);
    }

    public Pose2d update(double heading){
        //0.915 is experimentally determined deviation
        double left = range_left.getDistance(DistanceUnit.INCH) / 12.0 / 0.915 + 7.5 / 12.0;
        double right = range_right.getDistance(DistanceUnit.INCH) / 12.0 / 0.915 + 7.5 / 12.0;
        double front = range_front.getDistance(DistanceUnit.INCH) / 12.0 / 0.915 + 5.15 / 12.0;
        double back = range_back.getDistance(DistanceUnit.INCH) / 12.0 / 0.915 + 9 / 12.0;

        DistanceSensorAlt.geom position = DistanceSensorAlt.calculate_location(left, right, front, back, heading, new scaleGraphics());

        if(position instanceof point) {
            //if [45, -135], reflect over y and x axes DON'T ASK ME WHY IT JUST WORKS
            heading = RotationUtil.mod(heading, 2 * Math.PI);
            if (heading < Math.toRadians(315) || heading > Math.toRadians(135))
                position.scale(4, 6, -1, -1);
            position.translate(-6, -6);
            position.scale(0, 0, 12, 12);

            return new Pose2d(((point)position).x, ((point)position).y, heading);
        }

        return null;
    }
}
