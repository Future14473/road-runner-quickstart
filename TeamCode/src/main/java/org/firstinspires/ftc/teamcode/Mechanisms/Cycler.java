package org.firstinspires.ftc.teamcode.Mechanisms;

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

    public void transferIntakeToDumper(){
        intake.setStopNoodlesSpeed(); intake.moveNoodles(); //must be together to make change
        intake.flipInTeleop();
        intake.transferOutake();
    }

    public void dumperOutPrepHigh(){
        output.flipInDumper();
        output.extendHigh();
        timer.safeDelay(500);
        output.flipHalfDumper();
        intake.transferIntake();
    }

    // for auto
    public void dumperOutPrepLow(){
        output.flipInDumper();
        output.extendLow();
        timer.safeDelay(500);
        output.flipHalfDumper();
        intake.transferIntake();
    }
    public void dumperOutPrepMiddle(){
        output.flipInDumper();
        output.extendMid();
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

}
