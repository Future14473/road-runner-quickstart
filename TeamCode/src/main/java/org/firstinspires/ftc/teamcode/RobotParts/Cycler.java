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
    }

    public void intakeOut(){
        intake.flipOut();
        intake.inNoodles();
        intake.slideOut();
    }

    public void retractIntakeTransfer(){
        intake.stopNoodles();
        intake.flipIn();
        intake.slideIn();
        intake.transferOutake();
    }

    public void dumperOutPrep(){
        output.flipInDumper();
        output.extend();
        output.flipHalfDumper();
        intake.transferIntake();
    }

    public void dumpRetract(){
        output.flipOutDumper();
        timer.safeDelay(500);
        output.flipInDumper();
        output.retract();
    }




}
