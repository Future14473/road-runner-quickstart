package org.firstinspires.ftc.teamcode.Hardware.Intake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Hardware.Outtake.Dumper;
import org.firstinspires.ftc.teamcode.Hardware.Outtake.Turret;
import org.firstinspires.ftc.teamcode.Hardware.util.Timer;

@Config
public class Intake {
    DcMotor intake;
    Servo dropDown;
    public static double dropPos = 0, upPos = 0.0;
    Dumper dumper;

    public Intake(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotor.class, "intake");
        dropDown = hardwareMap.get(Servo.class, "dropDown");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        dumper = new Dumper(hardwareMap);

    }

    public void setPower(double pow){intake.setPower(pow);}

    public void smartIn(Turret turret, Timer timer){
        if (turret.isDown()) {
            in();
        }
        if(turret.hasBlock() && turret.isDown()){
            timer.safeDelay(300);
            out();
            timer.safeDelay(200);
            if(turret.hasBlock()){
                if (turret.isShared) {
                    turret.upShared();
                } else {
                    turret.up();
                }
                timer.safeDelay(100);
                stop();
            }
        }
    }

    public void smartInAggressive(Turret turret, Timer timer){
        if (turret.isDown()) {
            in();
        }
        if(turret.hasBlock() && turret.isDown()){
            out();
            if (turret.isShared) {
                turret.upShared();
            } else {
                turret.up();
            }
            timer.safeDelay(100);
            stop();
        }
    }

    public void smartInAggressiveOutRed(Turret turret, Timer timer){
        if (turret.isDown()) {
            in();
        }
        if(turret.hasBlock() && turret.isDown()){
            timer.safeDelay(300);
            out();
            timer.safeDelay(300);
            if (turret.isShared && turret.naiveHasBlock()) {
                turret.upShared();
                timer.safeDelay(300);
            } if (!turret.isShared && turret.naiveHasBlock()) {
                turret.up();
            }x
//            if(turret.naiveHasBlock()) {
//                turret.outputRed();
//                timer.safeDelay(100);
//                stop();
//            } else {
//                turret.slidesIncrementUp();
//                timer.safeDelay(100);
//                dumper.intake();
//                timer.safeDelay(100);
//                turret.letGoEmergency();
//            }
            turret.outputRed();
            timer.safeDelay(100);
            stop();
        }
    }

    public void smartInAggressiveOutBlue(Turret turret, Timer timer){
        if (turret.isDown()) {
            in();
        }
        if(turret.hasBlock() && turret.isDown()){
            out();
            if (turret.isShared) {
                turret.upShared();
                timer.safeDelay(300);
            } else {
                turret.up();
            }
            turret.leftSharedHub();
            timer.safeDelay(100);
            stop();
        }
    }

    public void in(){
        intake.setPower(1.0);
    }

    public void out(){
        intake.setPower(-1.0);
    }

    public void stop() {intake.setPower(0);}

    public void drop() {dropDown.setPosition(dropPos);}

    public void up() {dropDown.setPosition(upPos);}
}
