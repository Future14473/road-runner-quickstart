package org.firstinspires.ftc.teamcode.Hardware.Turret;

import org.firstinspires.ftc.teamcode.Hardware.Outtake.Linkages;
import org.firstinspires.ftc.teamcode.Hardware.Outtake.Slides;

public class Turret {
    Linkages linakges;
    LazySusan lazySusan;
    Slides slides;
    public void rotateTo(double degrees){
        lazySusan.rotateToDegrees(degrees);
    }
    public void turretleft(){
        rotateTo(90);
    }
    public void turretRight(){
        rotateTo(-90);
    }
    public void dumperIn(){
        linakges.dumperIn();
    }
    public void dumperOut(){
        linakges.dumperOut();
    }
    public void extendhigh(){
        slides.extendHigh();
    }
    public void extendmid(){
        slides.extendMid();
    }
    public void extendlow(){
        slides.extendLow();
    }
    public void retract(){
        rotateTo(0);
        slides.retract();
    }
}
