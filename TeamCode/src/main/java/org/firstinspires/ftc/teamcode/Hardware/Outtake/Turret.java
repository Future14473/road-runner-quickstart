package org.firstinspires.ftc.teamcode.Hardware.Outtake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Hardware.util.Timer;

@Config
public class Turret {
    Linkages linkages;
    Dumper dumper;
    Slides slides;
    LazySusan lazySusan;
    BoxSensor boxSensor;
    Timer timer;
    LinearOpMode opMode;
    Thread turretThread;
    public boolean goingUp;
    public boolean isShared = true;
    public static boolean RESET = false; // default shared bot

    public static double duckAngleBlue = 55;
    public static double duckAngleRed = -55;

    public Turret(HardwareMap hardwareMap, LinearOpMode linearOpMode) {
        lazySusan = new LazySusan(hardwareMap);
        linkages = new Linkages(hardwareMap);
        slides = new Slides(hardwareMap);
        dumper = new Dumper(hardwareMap);
        boxSensor = new BoxSensor(hardwareMap);
        timer = new Timer(linearOpMode);
        this.opMode = linearOpMode;
        goingUp = false;
    }


    public double calculateDirection(double Xcur, double Ycur, double Hcur, double Xtar, double Ytar){
        Xcur = Xcur + Math.cos(Hcur)*7;
        Ycur = Ycur + Math.sin(Hcur)*7;
        double deltaX = Xtar - Xcur;
        double deltaY = Ytar - Ycur;
        double Ang = Math.atan(deltaY / deltaX); // ang finds the angle between the raw X Y target and current (in radians)

        // if loop handles the wrapping cases
        if (deltaX > 0 && deltaY > 0){
            Ang = Ang;
        } else if (deltaX < 0 && deltaY > 0){
            Ang = Math.PI + Ang;
        } else if (deltaX < 0 && deltaY < 0){
            Ang = Math.PI + Ang;
        } else if (deltaX > 0 && deltaY < 0){
            Ang = 2*Math.PI + Ang;
        }
        // return the angle should be between 0 -2 PI according to the meep meep field view headings
        return Ang;
    }

    public double pointTo(double Xcur, double Ycur, double Hcur, double Xtar, double Ytar){
        double Angle = calculateDirection(Xcur, Ycur, Hcur, Xtar, Ytar); // calculates the angle from current to target
        lazySusan.rotateToDegreesRobotCentric(Math.toDegrees(-Angle+Hcur)); // should point the lazy susan towards xy coordinate
        return Angle;
    }

    public void turnTo(double angle){
        lazySusan.rotateToDegreesRobotCentric(angle);
    }

    public void preloadUpRed(){
        slides.extendHigh();
        timer.safeTurretDelay(500);
//        while (slides.isBusy() && opMode.opModeIsActive()){}
        lazySusan.rotateToDegreesRobotCentric(45);
        linkages.extend();
        timer.safeTurretDelay(1500);
    }

    public void preloadUpBlue(){
        slides.extendHigh();
        timer.safeTurretDelay(500);
//        while (slides.isBusy() && opMode.opModeIsActive()){}
        lazySusan.rotateToDegreesRobotCentric(-45);
        linkages.extend();
        timer.safeTurretDelay(1500);
    }
    public void preloadMidBlue(){
        slides.extendMid();
        timer.safeTurretDelay(500);
        lazySusan.rotateToDegreesRobotCentric(-45);
//        while (slides.isBusy() && opMode.opModeIsActive()){}
        linkages.extend();
        timer.safeTurretDelay(900);
    }
    public void preloadMidRed(){
        slides.extendMid();
        timer.safeTurretDelay(500);
        lazySusan.rotateToDegreesRobotCentric(45);
//        while (slides.isBusy() && opMode.opModeIsActive()){}
        linkages.extend();
        timer.safeTurretDelay(900);
    }
    public void preloadLowBlue(){
        slides.extendLowPrepBlue();
        timer.safeTurretDelay(500);
//        while (slides.isBusy() && opMode.opModeIsActive()){}
        lazySusan.rotateToDegreesRobotCentric(-45);
        linkages.extendLowAuto();
        timer.safeTurretDelay(1000);
        slides.extendLow();
        timer.safeTurretDelay(800);
    }
    public void preloadLowRed(){
        slides.preRetract();
        timer.safeTurretDelay(500);
//        while (slides.isBusy() && opMode.opModeIsActive()){}
        lazySusan.rotateToDegreesRobotCentric(45);
        linkages.extendLowAuto();
        timer.safeTurretDelay(1000);
        slides.extendLow();
        timer.safeTurretDelay(800);
    }

    public void preloadDownLowBlue(){
        // down
        dumper.dump();
        timer.safeTurretDelay(500);
        slides.extendLowPrepBlue();
        timer.safeTurretDelay(500);
        linkages.retract();
        dumper.intake();
        timer.safeTurretDelay(1100);

        lazySusan.rotateToDegreesRobotCentric(0);
        while(opMode.opModeIsActive() && !lazySusan.isHome()){
            // wait
        }
        slides.retract();
    }
    public void preloadDownLowRed(){
        // down
        dumper.dump();
        timer.safeTurretDelay(500);
        slides.preRetract();
        timer.safeTurretDelay(500);
        linkages.retract();
        dumper.intake();
        timer.safeTurretDelay(1100);

        lazySusan.rotateToDegreesRobotCentric(0);
        while(opMode.opModeIsActive() && !lazySusan.isHome()){
            // wait
        }
        slides.retract();
    }

    public void preloadDown(){
        // down
        dumper.dump();
        timer.safeTurretDelay(500);
        linkages.retract();
        dumper.intake();
        timer.safeTurretDelay(1100);

        lazySusan.rotateToDegreesRobotCentric(0);
        while(opMode.opModeIsActive() && !lazySusan.isHome()){
            // wait
        }
        slides.retract();
    }

    public void duckScorePrepRed() {
        dumper.close();
        slides.extendHigh();
        timer.safeTurretDelay(500);
        this.turnTo(duckAngleRed);
        timer.safeTurretDelay(500);
        linkages.extend();
        timer.safeTurretDelay(800);
    }

    public void duckScorePrepBlue(){
        dumper.close();
        slides.extendHigh();
        timer.safeTurretDelay(500);
        this.turnTo(duckAngleBlue);
        timer.safeTurretDelay(500);
        linkages.extend();
        timer.safeTurretDelay(800);
    }

    public void rightSharedHub(){
        slides.extendLowPrepBlue();
        timer.safeTurretDelay(300);
        lazySusan.rotateToDegreesRobotCentric(90);
        timer.safeTurretDelay(300);
        slides.scoreShared();
        linkages.extendSharedMid();
    }

    public void leftSharedHub(){
        slides.extendLowPrepBlue();
        timer.safeTurretDelay(300);
        lazySusan.rotateToDegreesRobotCentric(-90);
        timer.safeTurretDelay(300);
        slides.scoreShared();
        linkages.extendSharedMid();
    }

    public void right(){
        slides.extendHigh();
        lazySusan.rotateToAbsolutePos(180);
        linkages.extend();
    }

    public void letGoEmergency(){
        lazySusan.rotateToDegreesRobotCentric(0);
        while(opMode.opModeIsActive() && !lazySusan.isHome()){
            // wait
        }
        slides.retract();
        dumper.intake();
    }

    public void left(){
        slides.extendHigh();
        lazySusan.rotateToAbsolutePos(-180);
        linkages.extend();
    }

    public void back(){
        // make it go up first so it doesn't break the REV hubs
        lazySusan.rotateToDegreesRobotCentric(180);
        linkages.extend();
    }

    public void down(){
        dumper.dump();
        timer.safeTurretDelay(500);
        linkages.retract();
        dumper.intake();
        timer.safeTurretDelay(500);
        lazySusan.rotateToDegreesRobotCentric(0);
        timer.safeTurretDelay(isShared ? 475 : 1500);
        slides.retract();
    }

    public void downKeepBlock(){
        linkages.retract();
        dumper.intake();
        timer.safeTurretDelay(500);
        lazySusan.rotateToDegreesRobotCentric(0);
        timer.safeTurretDelay(isShared ? 350 : 1000);
        slides.retract();
    }

    public void resetTurretZero(){
        lazySusan.resetTurret(timer);
    }

    public void up(){
        goingUp = true;
        dumper.close();
        timer.safeTurretDelay(400);
        if (isShared){
            slides.extendLowPrepBlue();
        } else {
            slides.extendHigh();
        }
        while (slides.isBusy()){} // slides.extend is async so make sure to only go up after it goes up
        goingUp = false;
    }
    public void upShared(){
        goingUp = true;
        dumper.close();
        timer.safeTurretDelay(400);
        slides.extendMid();
        while (slides.isBusy()){} // slides.extend is async so make sure to only go up after it goes up
        goingUp = false;
    }

    public void readyToIntake(){
        dumper.intake();
    }

    public void closeDumper(){
        dumper.close();
    }

    public boolean hasBlock(){return boxSensor.hasBlock();}

    public boolean isDown(){ return (slides.LeftSlide.getCurrentPosition() < 200) && !goingUp;}

    public void release(){dumper.dump();}

    public void toggleLinkages(){linkages.toggle();}

    public void linkOutShared(){linkages.extendShareFar();}
    public void linkMidShared(){linkages.extendSharedMid();}
    public void linkCloseShared(){linkages.extendShareClose();}

    public String getHeight() {return String.valueOf(slides.getHeight());}
    
    public void resetWholeTurret(){
        RESET = true;
        slides.extendHigh();
        RESET = false;
        downKeepBlock();
    }
}
