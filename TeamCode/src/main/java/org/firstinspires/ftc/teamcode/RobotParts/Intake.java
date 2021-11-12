package org.firstinspires.ftc.teamcode.RobotParts;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Intake {
    DcMotor noodles;
    Servo retracter;
    Servo transfer;

    public static double retractInPos = 0;
    public static double retractOutPos = 1.0;
    public static double transferIntakePos = 0;
    public static double transferOutputPos = 1.0;


    public Intake(HardwareMap hardwareMap){
        noodles = hardwareMap.get(DcMotor.class, "noodles");
        noodles.setDirection(DcMotorSimple.Direction.REVERSE);

        retracter = hardwareMap.get(Servo.class, "retracter");
        retracter.setDirection(Servo.Direction.REVERSE);

        transfer = hardwareMap.get(Servo.class, "transfer");
    }

    void setNoodlePower(double power){
        noodles.setPower(power);
    }

    public void inNoodles(){
        noodles.setPower(1.0);
    }
    public void outNoodles(){
        noodles.setPower(-1.0);
    }

    public void flipIn(){
        retracter.setPosition(retractInPos);
    }

    public void flipOut(){
        retracter.setPosition(retractOutPos);
    }

    //goes to the intake transfer position
    public void transferIntake(){
        transfer.setPosition(transferIntakePos);
    }

    public void transferOutake(){
        transfer.setPosition(transferOutputPos);
    }
//    public void retract()
    public double getRetracterPosition(){
        return retracter.getPosition();
    }
}
