

package org.firstinspires.ftc.teamcode.ourOpModes;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * {@link LaserLocalization} illustrates how to use the Modern Robotics
 * Range Sensor.
 *
 * The op mode assumes that the range sensor is configured with a name of "sensor_range".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 *
 * @see <a href="http://modernroboticsinc.com/range-sensor">MR Range Sensor</a>
 */
@TeleOp(name = "Sensor: MR range sensor", group = "Sensor")
public class LaserLocalization extends LinearOpMode {

    ModernRoboticsI2cRangeSensor rangeSensorBack;
    ModernRoboticsI2cRangeSensor rangeSensorLeft;
    ModernRoboticsI2cRangeSensor rangeSensorRight;
    ModernRoboticsI2cRangeSensor rangeSensorFront;




    @Override public void runOpMode() {

        // get a reference to our compass
        rangeSensorBack = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "sensor_range_back");
        rangeSensorLeft = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "sensor_range_left");
        rangeSensorRight = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "sensor_range_right");
        rangeSensorFront = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "sensor_range_front");



        // wait for the start button to be pressed
        waitForStart();

        while (opModeIsActive()) {
//            telemetry.addData("raw ultrasonic", rangeSensorBack.rawUltrasonic());
//            telemetry.addData("raw optical", rangeSensorBack.rawOptical());
//            telemetry.addData("cm optical", "%.2f cm", rangeSensorBack.cmOptical());
            telemetry.addData("cm Back", "%.2f cm", rangeSensorBack.getDistance(DistanceUnit.CM));
            telemetry.addData("cm Left", "%.2f cm", rangeSensorLeft.getDistance(DistanceUnit.CM));
            telemetry.addData("cm Right", "%.2f cm", rangeSensorRight.getDistance(DistanceUnit.CM));
            telemetry.addData("cm Front", "%.2f cm", rangeSensorFront.getDistance(DistanceUnit.CM));


            telemetry.update();
        }
    }
}
