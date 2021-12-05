package org.firstinspires.ftc.teamcode.RobotParts;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.Timer;

public class Cycler {
    Intake intake;
    Output output;
    Timer timer;

    public Cycler(Intake intake, Output output, LinearOpMode opMode){
        this.intake = intake;
        this.output = output;
        timer = new Timer(opMode);
        intake.flipInTeleop();
    }

    public void intakeOut(){
        intake.flipOutTeleop();
        timer.safeDelay(300);
        intake.inNoodles();
        intake.slideOut();
    }

    public void retractIntakeTransfer(){
        intake.stopNoodles();
        intake.slideIn();
        timer.safeDelay(500);
        intake.flipInTeleop();
        intake.transferOutake();
    }

    public void dumperOutPrep(){
        output.flipInDumper();
        output.extend();
        timer.safeDelay(500);
        output.flipHalfDumper();
        intake.transferIntake();
    }

    public void dumpRetract(){
        output.flipOutDumper();
        timer.safeDelay(500);
        output.flipInDumper();
        timer.safeDelay(500);
        output.retract();
    }

    public void dumpRetractExtendIntake(){
        output.flipOutDumper();
        intake.slideOut();
        timer.safeDelay(500);
        output.flipInDumper();
        timer.safeDelay(500);
        output.retract();
    }




}
