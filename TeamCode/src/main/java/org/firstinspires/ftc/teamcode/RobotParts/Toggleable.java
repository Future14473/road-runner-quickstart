package org.firstinspires.ftc.teamcode.RobotParts;

public class Toggleable {
    public boolean prev = false;
    Runnable func;

    public Toggleable(Runnable func){
        this.func = func;
    }

    public void toggle(boolean cond){
        if(cond != prev && cond){
            func.run();
        }
        prev = cond;
    }
}


