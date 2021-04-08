package org.firstinspires.ftc.teamcode.ourOpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Bluetooth.BluetoothConvenient;
import org.firstinspires.ftc.teamcode.Follower.Follower;
import org.firstinspires.ftc.teamcode.LaserLocalization.DistanceSensorAlt;
import org.firstinspires.ftc.teamcode.LaserLocalization.point;
import org.firstinspires.ftc.teamcode.LaserLocalization.scaleGraphics;
import org.firstinspires.ftc.teamcode.Roadrunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.ourOpModes.resources.IMU;
import org.firstinspires.ftc.teamcode.ourOpModes.resources.RotationUtil;


@TeleOp(name = "Localization test", group = "Sensor")
public class MRSensortest extends LinearOpMode {

    ModernRoboticsI2cRangeSensor range_left;
    ModernRoboticsI2cRangeSensor range_right;
    ModernRoboticsI2cRangeSensor range_front;
    ModernRoboticsI2cRangeSensor range_back;

    VuforiaPhone vuforiaPhone;

    IMU imu;

    SampleMecanumDrive drive;

    @Override public void runOpMode() {
        telemetry.setAutoClear(false);

        BluetoothConvenient BT = new BluetoothConvenient(telemetry, hardwareMap, this);

        vuforiaPhone = new VuforiaPhone(hardwareMap, telemetry);

        // get a reference to our compass
        range_left = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range_left");
        range_right = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range_right");
        range_front = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range_front");
        range_back = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range_back");

        imu = new IMU(hardwareMap, telemetry);

        drive = new SampleMecanumDrive(hardwareMap);

        //CalibrateIMUwithLaser.calibrate(-1, imu, range_front, range_back, telemetry, drive);
        telemetry.setAutoClear(true);

        vuforiaPhone.beginTracking();

        while (!isStarted() || vuforiaPhone.getLocation()==null){
            telemetry.addData("vuforia", "looking for trackable");
            vuforiaPhone.printLocation(vuforiaPhone.lastLocation);
        }
        imu.thisHeadingIsActually(imu.getHeading(), vuforiaPhone.locationToHeading(vuforiaPhone.lastLocation));

        vuforiaPhone.stopTracking();

        // wait for the start button to be pressed
        waitForStart();

        new Follower(drive, null, this, telemetry, gamepad1, imu).goToHeading(0);

        while (opModeIsActive()) {
            //0.915 is experimentally determined deviation
            double left = range_left.getDistance(DistanceUnit.INCH) / 12.0 / 0.915 + 7.5 / 12.0;
            double right = range_right.getDistance(DistanceUnit.INCH) / 12.0 / 0.915 + 7.5 / 12.0;
            double front = range_front.getDistance(DistanceUnit.INCH) / 12.0 / 0.915 + 5.15 / 12.0;
            double back = range_back.getDistance(DistanceUnit.INCH) / 12.0 / 0.915 + 9 / 12.0;

            telemetry.addData("front", front);
            telemetry.addData("back", back);
            telemetry.addData("left", left);
            telemetry.addData("right", right);


            double heading = imu.getHeading();

            DistanceSensorAlt.geom position = DistanceSensorAlt.calculate_location(left, right, front, back, heading, new scaleGraphics());

            if(position != null) {
                //if [45, -135], reflect over y and x axes DON'T ASK ME WHY IT JUST WORKS
                heading = RotationUtil.mod(heading, 2 * Math.PI);
                if (heading > Math.toRadians(315) || heading < Math.toRadians(135))
                    position.scale(4, 6, -1, -1);
            }

            if(position instanceof point) {
                telemetry.addData("position", String.format("%.2f %.2f", ((point) position).x, ((point) position).y));
                BT.bluetoothClient.send(String.format("\\xyrplot %.2f %.2f %.2f\n", ((point) position).x, ((point) position).y, heading));
            }else if(position instanceof DistanceSensorAlt.line)
                telemetry.addData("position", "is a line");
            else
                telemetry.addData("position", "undetermined");

           // telemetry.addData("laser length vertical total", front + back);
            telemetry.addData("heading in deg", Math.toDegrees(heading));

            telemetry.update();

            DRIVE(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x * 0.6);
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
                        -rotate * 0.7
                )
        );
    }

}

