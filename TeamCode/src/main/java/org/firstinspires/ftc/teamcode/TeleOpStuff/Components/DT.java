package org.firstinspires.ftc.teamcode.TeleOpStuff.Components;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

//positive - counterClockwise

@Config
public class DT {

    DcMotorEx rightFront;
    DcMotorEx rightBack;
    DcMotorEx leftFront;
    DcMotorEx leftBack;
    final double TICKS_PER_ROTATION = 384.5;
    final double WHEEL_RADIUS = 1.889; //Inches
    final double DT_WIDTH = 10.368; //Inches
    final double MAX_ANGULAR_VELOCITY = 45.553; //Radians per second
    final double MAX_VELOCITY = MAX_ANGULAR_VELOCITY * WHEEL_RADIUS;

    public DT(HardwareMap hardwareMap, LinearOpMode opMode){
        //Change motor configuration names if necessary
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    //converts distance to motor ticks (distance is in inches)
    public double distanceToTicks(double distance){
        return distance / (2 * Math.PI * WHEEL_RADIUS) * TICKS_PER_ROTATION;
    }

    //convert motor ticks to distance (distance is in inches)
    public double ticksToDistance(double ticks){
        return ticks * 2 * Math.PI * WHEEL_RADIUS / TICKS_PER_ROTATION;
    }

    //Sets right drive pod velocity (velocity is in inches/second, positive is forward)
    public void setRightPodVelocity(double velocity){
        rightFront.setVelocity(distanceToTicks(velocity));
        rightBack.setVelocity(-distanceToTicks(velocity));
    }

    //Sets left drive pod velocity (velocity is in inches/second, positive is forward)
    public void setLeftPodVelocity(double velocity){
        leftFront.setVelocity(-distanceToTicks(velocity));
        leftBack.setVelocity(distanceToTicks(velocity));
    }

    //Gets right drive pod velocity (velocity is in inches/second, positive is forward)
    public double getRightPodVelocity(){
        return ticksToDistance(rightFront.getVelocity());
    }

    //Gets left drive pod velocity (velocity is in inches/second, positive is forward)
    public double getLeftPodVelocity(){
        return ticksToDistance(leftBack.getVelocity());
    }

    //Gets right drive pod Angular Velocity in Radians/second (FYI: 3.14159265 radians = 180 degrees)
    public double getRightAngularVelocity(){
        return getRightPodVelocity() / (2 * Math.PI * WHEEL_RADIUS);
    }

    //Gets left drive pod Angular Velocity in Radians/second (FYI: 3.14159265 radians = 180 degrees)
    public double getLeftAngularVelocity(){
        return getLeftPodVelocity() / (2 * Math.PI * WHEEL_RADIUS);
    }

    //Sets robot velocity (inches/second, foreward is positive) and angular velocity (radians/second, counterclockwise positive)
    //Ensure the pod velocities do not go above the maximum velocity
    //If the pod velocities exceed motor maximum velocity, angular velocity will be prioritized
    public void setVelocity(double velocity, double angularVelocity){
        double velocityLeft = (velocity - (DT_WIDTH / 2) * angularVelocity);
        double velocityRight = (velocity + (DT_WIDTH / 2) * angularVelocity);
        if(velocityLeft < MAX_VELOCITY && velocityRight < MAX_VELOCITY) {
            setLeftPodVelocity(velocityLeft);
            setRightPodVelocity(velocityRight);
        } else {
            velocity = Math.min(MAX_VELOCITY + (DT_WIDTH / 2) * angularVelocity, MAX_VELOCITY - (DT_WIDTH / 2) * angularVelocity);
            velocityLeft = (velocity - (DT_WIDTH / 2) * angularVelocity);
            velocityRight = (velocity + (DT_WIDTH / 2) * angularVelocity);
            setLeftPodVelocity(velocityLeft);
            setRightPodVelocity(velocityRight);
        }
    }
}
