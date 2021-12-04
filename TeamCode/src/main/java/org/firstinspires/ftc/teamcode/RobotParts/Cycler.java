package org.firstinspires.ftc.teamcode.RobotParts;

public class Cycler {
    Intake intake;
    Output output;

    public Cycler(Intake intake, Output output){
        this.intake = intake;
        this.output = output;
    }

    public void intakeOut(){
        intake.inNoodles();
        intake.slideOut();
    }

    public void retractIntakeTransfer(){
        intake.stopNoodles();
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
        output.flipInDumper();
        output.retract();
    }




}
