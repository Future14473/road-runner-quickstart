package org.firstinspires.ftc.teamcode.Hardware.Outtake;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Hardware.util.Timer;

public class Turret {
    Linkages linkages;
    Dumper dumper;
    Slides slides;
    LazySusan lazySusan;
    BoxSensor boxSensor;
    Timer timer;

    public Turret(HardwareMap hardwareMap, LinearOpMode linearOpMode){
        lazySusan = new LazySusan(hardwareMap);
        linkages = new Linkages(hardwareMap);
        slides = new Slides(hardwareMap);
        dumper = new Dumper(hardwareMap);
        boxSensor = new BoxSensor(hardwareMap);
        timer = new Timer(linearOpMode);
    }

    public void right(){
        lazySusan.rotateToDegrees(90);
        linkages.extend();
    }

    public void left(){
        lazySusan.rotateToDegrees(-90);
        linkages.extend();
    }

    public void back(){
        // make it go up first so it doesn't break the REV hubs
        up();
        lazySusan.rotateToDegrees(180);
        linkages.extend();
    }

    public void down(){
        dumper.dump();
        timer.safeDelay(500);
        linkages.retract();
        dumper.intake();
        timer.safeDelay(500);
        lazySusan.rotateToDegrees(0);
        timer.safeDelay(1000);
        slides.retract();
    }

    public void up(){
        dumper.close();
        timer.safeDelay(200);
        slides.extendHigh();
    }

    public void readyToIntake(){
        dumper.intake();
    }

    public void closeDumper(){
        dumper.close();
    }

    public boolean hasBlock(){return boxSensor.hasBlock();}

    public boolean isDown(){ return slides.LeftSlide.getCurrentPosition() < 200;}

    public void release(){dumper.dump();}



}
