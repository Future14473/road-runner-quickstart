package org.firstinspires.ftc.teamcode.RobotParts;

public class AutomaticDumper {
    Intake intake;
    Output output;

    public AutomaticDumper(Intake intake, Output output){
        this.intake = intake;
        this.output = output;
    }

    public void retractDump(){
        intake.slideIn();
        intake.transferOutake();
        output.extend();
        intake.transferIntake();
    }
}
