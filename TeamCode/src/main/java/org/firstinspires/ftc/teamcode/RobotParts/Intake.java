package org.firstinspires.ftc.teamcode.RobotParts;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Intake {
    DcMotor noodles;
    DcMotorEx slides;
    Servo retracter;
    Servo leftTransfer;
    Servo rightTransfer;

    public static double retractInPos = 0;
    public static double retractOutPos = 1.0;
    public static double transferIntakePos = 0;
    public static double transferOutputPos = 1.0;
    public static int slideInPos = 0;
    public static int slideOutPos = 170;
    public static int maxVelocity = 100;
    public boolean isSlideOut = false;
    public boolean isSlideIn = false;



    public Intake(HardwareMap hardwareMap){
        noodles = hardwareMap.get(DcMotor.class, "noodles");
        noodles.setDirection(DcMotorSimple.Direction.REVERSE);

        slides = hardwareMap.get(DcMotorEx.class, "intakeSlides");
        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slides.setDirection(DcMotorSimple.Direction.REVERSE);

        retracter = hardwareMap.get(Servo.class, "retracter");
        retracter.setDirection(Servo.Direction.REVERSE);

        leftTransfer = hardwareMap.get(Servo.class, "leftTransfer");

        rightTransfer = hardwareMap.get(Servo.class, "rightTransfer");
        rightTransfer.setDirection(Servo.Direction.REVERSE);
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

    public void slideIn(){
        isSlideOut = false;
        slides.setTargetPosition(slideInPos);
        slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slides.setVelocity(maxVelocity);
    }

    public void slideOut(){
        isSlideOut = true;
        slides.setTargetPosition(slideOutPos);
        slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slides.setVelocity(maxVelocity);
    }

    //goes to the intake transfer position
    public void transferIntake(){
        leftTransfer.setPosition(transferIntakePos);
        rightTransfer.setPosition(transferIntakePos);
    }

    public void transferOutake(){
        leftTransfer.setPosition(transferOutputPos);
        rightTransfer.setPosition(transferOutputPos);
    }
//    public void retract()

    public double getRetracterPosition(){
        return retracter.getPosition();
    }

    public int getSlidePosition(){return slides.getCurrentPosition();}
}
