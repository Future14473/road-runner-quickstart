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
    Servo flipper;
    Servo leftTransfer;
    Servo rightTransfer;

    public static double retractInPosTeleop = 0;
    public static double retractOutPosTeleop = 0.47;
    public static double transferIntakePos = 0;
    public static double transferOutputPos = 1.8;
    double noodlePower = 0;


    public Intake(HardwareMap hardwareMap){
        noodles = hardwareMap.get(DcMotor.class, "noodles");
        noodles.setDirection(DcMotorSimple.Direction.REVERSE);

        flipper = hardwareMap.get(Servo.class, "intakeFlipper");
        flipper.setDirection(Servo.Direction.REVERSE);

        leftTransfer = hardwareMap.get(Servo.class, "leftTransfer");

        rightTransfer = hardwareMap.get(Servo.class, "rightTransfer");
        rightTransfer.setDirection(Servo.Direction.REVERSE);
    }

    public void inNoodlesUp(){
        setInNoodlesSpeed();
        flipOutTeleop();
    }

    public void outNoodlesUp(){
        setOutNoodlesSpeed();
        flipOutTeleop();
    }

    public void setInNoodlesSpeed(){
        noodlePower = 1.0;
    }
    public void setOutNoodlesSpeed(){
        noodlePower = -1.0;
    }

    public void moveNoodles(){
        noodles.setPower(noodlePower);
    }
    public void setStopNoodlesSpeed() {noodlePower = 0;}


    public void flipInTeleop(){
        flipper.setPosition(retractInPosTeleop);
    }

    public void flipOutTeleop(){
        flipper.setPosition(retractOutPosTeleop);
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
}
