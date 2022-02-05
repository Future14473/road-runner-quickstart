package org.firstinspires.ftc.teamcode.TeleOpStuff.Components;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
public class OuttakeLift {

    DcMotorEx LiftMotor1;
    DcMotorEx LiftMotor2;
    PIDFCoefficients pidf;

    final double SPOOL_RADIUS = 0.708; //Inches
    final double TICKS_PER_ROTATION = 207.6;

    //Clockwise - Up

    public OuttakeLift(HardwareMap hardwareMap, LinearOpMode opMode){
        pidf = new PIDFCoefficients(10, 3,6, 0);

        LiftMotor1 = hardwareMap.get(DcMotorEx.class, "LiftMotor1");
        LiftMotor2 = hardwareMap.get(DcMotorEx.class, "LiftMotor2");
        LiftMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LiftMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LiftMotor1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        LiftMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
    }

    public double ticksToHeight(double ticks){
        return -ticks * 2 * Math.PI * SPOOL_RADIUS / TICKS_PER_ROTATION;
    }

    public double heightToTicks(double height){
        return height * TICKS_PER_ROTATION / (2 * Math.PI * SPOOL_RADIUS);
    }

    public void setHeight(double height){
        LiftMotor1.setTargetPosition((int) heightToTicks(height));
        LiftMotor2.setTargetPosition((int) heightToTicks(height));
    }
}
