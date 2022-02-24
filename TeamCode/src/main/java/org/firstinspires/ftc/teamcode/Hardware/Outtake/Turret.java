package org.firstinspires.ftc.teamcode.Hardware.Outtake;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Hardware.util.Timer;

public class Turret {
    Linkages linkages;
    Dumper dumper;
    Slides slides;
    LazySusan lazySusan;
    BoxSensor boxSensor;
    Timer timer;
    LinearOpMode opMode;

    public Turret(HardwareMap hardwareMap, LinearOpMode linearOpMode){
        lazySusan = new LazySusan(hardwareMap);
        linkages = new Linkages(hardwareMap);
        slides = new Slides(hardwareMap);
        dumper = new Dumper(hardwareMap);
        boxSensor = new BoxSensor(hardwareMap);
        timer = new Timer(linearOpMode);
        this.opMode = linearOpMode;
    }

    public void rightSharedHub(){
        slides.extendLow();
        timer.safeDelay(300);
        lazySusan.rotateToDegrees(90);
        linkages.extendShared();
    }

    public void leftSharedHub(){
        slides.extendLow();
        timer.safeDelay(300);
        lazySusan.rotateToDegrees(-90);
        linkages.extendShared();
    }

    public void right(){
        lazySusan.rotateToDegrees(135);
        linkages.extend();
    }

    public void letGoEmergency(){
        lazySusan.rotateToDegrees(0);
        while(opMode.opModeIsActive() && !lazySusan.isHome()){
            // wait
        }
        slides.retract();
        dumper.intake();
    }

    public void left(){
        lazySusan.rotateToDegrees(-135);
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
        timer.safeDelay(1100);

        boolean isAngle180 = (lazySusan.getTargetDegrees() - 180) < 0.05;
        lazySusan.rotateToDegrees(0);
//        timer.safeDelay(isAngle180 ? 950 : 800);
        while(opMode.opModeIsActive() && !lazySusan.isHome()){
            // wait
        }
        timer.safeDelay(500);
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

    public void toggleLinkages(){linkages.toggle();}

    public String getHeight() {return String.valueOf(slides.getHeight());}
}
