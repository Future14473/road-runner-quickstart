package org.firstinspires.ftc.teamcode.ourOpModes.resources;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;


public class IMU  {
    BNO055IMU imu;

    // added to raw heading in case we need to reset the zero point
    public double headingOffset = 0;

    public IMU (HardwareMap hardwareMap, Telemetry telemetry){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;

        parameters.loggingEnabled = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);
        // imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        telemetry.addData("IMU", "startup done");
        telemetry.update();

    }


    // AXES SET UP FOR A CERTAIN MOUNTING POSITION
    // LONGEST DIMENSION OF REV HUB ALIGNS WITH X
    // second longest aligns with Y
    // thickness (third longest) aligns with Z

    // Z is forward
    // X is side to side
    // Y is heading axis
    public pose getPosition() {
        Position p = imu.getPosition();
        if (p != null) {
            return new pose(p.x, p.z, getHeading());
        }
        return null;
    }

    public pose getAccel() {
        Acceleration a = imu.getLinearAcceleration();
        if (a != null) {
            return new pose(a.xAccel, a.yAccel, a.zAccel);
        }
        return null;
    }

    public double getHeading(){
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        return angles.firstAngle + headingOffset;
    }

    public void setPreviousHeadingTo(double oldHeading, double newHeading){
        headingOffset += newHeading - oldHeading;
    }

}
