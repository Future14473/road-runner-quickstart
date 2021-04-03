package org.firstinspires.ftc.teamcode.ourOpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Medium.DistanceSensorMath;
import org.firstinspires.ftc.teamcode.Medium.intersection;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.List;


@TeleOp(name = "Localization test", group = "Sensor")
public class MRSensortest extends LinearOpMode {

    ModernRoboticsI2cRangeSensor range_left;
    ModernRoboticsI2cRangeSensor range_right;
    ModernRoboticsI2cRangeSensor range_front;
    ModernRoboticsI2cRangeSensor range_back;

    SampleMecanumDrive drive;

    @Override public void runOpMode() {

        // get a reference to our compass
        range_left = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range_left");
        range_right = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range_right");
        range_front = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range_front");
        range_back = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range_back");


        // wait for the start button to be pressed
        waitForStart();

        drive = new SampleMecanumDrive(hardwareMap);

        while (opModeIsActive()) {
            //0.91 is experimentally determined deviation
            double left = range_left.getDistance(DistanceUnit.INCH) / 12.0 / 0.915 + 7.5 / 12.0;
            double right = range_right.getDistance(DistanceUnit.INCH) / 12.0 / 0.915 + 7.5 / 12.0;
            double front = range_front.getDistance(DistanceUnit.INCH) / 12.0 / 0.915 + 5.0 / 12.0;
            double back = range_back.getDistance(DistanceUnit.INCH) / 12.0 / 0.915 + 8.7 / 12.0;
            telemetry.addData("left", "%.2f ft", left);
            telemetry.addData("right", "%.2f ft", right);
            telemetry.addData("front", "%.2f ft", front);
            telemetry.addData("back", "%.2f ft", back);

            List<intersection> results = DistanceSensorMath.distanceToPosition(left, right, front, back);

            if(results.size() > 0)
                telemetry.addData("position", String.format("%.2f %.2f", results.get(0).a.x, results.get(0).a.y));
            else
                telemetry.addData("position", "undetermined");


            telemetry.update();

            DRIVE(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);


        }
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
                        -rotate * 2
                )
        );
    }

}

