package org.firstinspires.ftc.teamcode.LaserLocalization;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.ourOpModes.resources.RotationUtil;

public class laserLocalization {
    ModernRoboticsI2cRangeSensor range_left;
    ModernRoboticsI2cRangeSensor range_right;
    ModernRoboticsI2cRangeSensor range_front;
    ModernRoboticsI2cRangeSensor range_back;

    public laserLocalization(HardwareMap hardwareMap){
        range_left = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range_left");
        range_right = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range_right");
        range_front = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range_front");
        range_back = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range_back");
    }

    public static void main(String[] args) {
        // test isAccurate()
        for(int i = 0; i < 720; i+=20){
            System.out.println(i + " " + isAccurate(Math.toRadians(i)));
        }
    }

    public static boolean isAccurate(double heading){
        heading = RotationUtil.mod(heading, Math.PI*2);
        heading %= Math.PI/2;
        double diff = Math.abs(heading - Math.PI/4);
        return diff > Math.toRadians(15);
    }

    public Pose2d update(double heading){
        //0.915 is experimentally determined deviation
        double left = range_left.getDistance(DistanceUnit.INCH) / 12.0 / 0.915 + 7.5 / 12.0;
        double right = range_right.getDistance(DistanceUnit.INCH) / 12.0 / 0.915 + 7.5 / 12.0;
        double front = range_front.getDistance(DistanceUnit.INCH) / 12.0 / 0.915 + 5.15 / 12.0;
        double back = range_back.getDistance(DistanceUnit.INCH) / 12.0 / 0.915 + 9 / 12.0;

        DistanceSensorAlt.geom position = DistanceSensorAlt.calculate_location(left, right, front, back, heading, new scaleGraphics());

        if(left > 1000 || right > 1000 || front > 1000 || back > 1000) //invalid readings
            return null;

        if(position instanceof point) {
            // hardcoded fine tuning
            position.translate(-0.5, -0.25);
            //if [45, -135], reflect over y and x axes DON'T ASK ME WHY IT JUST WORKS
            heading = RotationUtil.mod(heading, 2 * Math.PI);
            if (heading < Math.toRadians(315) || heading > Math.toRadians(135))
                position.scale(4, 6, -1, -1);

            position.translate(-2, -6);
            position.scale(0, 0, 12, 12);

            point p = (point)position;

            return new Pose2d(-p.y, p.x, heading);
        }

        return null;
    }
}
