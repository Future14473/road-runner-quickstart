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
    public boolean goingUp, isShared = false;

    public static double duckAngle = 45;
    public volatile boolean isPreloadUp = false, isPreloadMid = false, isPreloadLow = false,
            isPreloadDown = false, isPreloadDownLow = false,
            isDuckScorePrepRed = false, isDuckScorePrepBlue = false,
            isDown = false, isUp = false;

    public Turret(HardwareMap hardwareMap, LinearOpMode linearOpMode){
        lazySusan = new LazySusan(hardwareMap);
        linkages = new Linkages(hardwareMap);
        slides = new Slides(hardwareMap);
        dumper = new Dumper(hardwareMap);
        boxSensor = new BoxSensor(hardwareMap);
        timer = new Timer(linearOpMode);
        this.opMode = linearOpMode;
        goingUp = false;
//        new Thread( () -> {
//            while (opMode.opModeIsActive()){
//                if (isPreloadUp){
//                    preloadUp();
//                    isPreloadUp = false;
//                } else if (isPreloadMid){
//                    preloadMid();
//                    isPreloadMid = false;
//                } else if (isPreloadLow){
//                    preloadLow();
//                    isPreloadLow = false;
//                } else if (isPreloadDown){
//                    preloadDown();
//                    isPreloadDown = false;
//                } else if (isPreloadDownLow){
//                    preloadDownLow();
//                    isPreloadDown = false;
//                } else if (isDuckScorePrepRed){
//                    duckScorePrepRed();
//                    isDuckScorePrepRed = false;
//                } else if (isDuckScorePrepBlue){
//                    duckScorePrepBlue();
//                    isDuckScorePrepBlue = false;
//                } else if (isDown){
//                    down();
//                    isDown = false;
//                } else if (isUp){
//                    up();
//                    isUp = false;
//                }
//            }
//        }).start();
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

    public void preloadUp(){
        slides.extendHigh();
        timer.safeDelay(500);
//        while (slides.isBusy() && opMode.opModeIsActive()){}
        linkages.extend();
        timer.safeDelay(500);
    }
    public void preloadMid(){
        slides.extendMid();
        timer.safeDelay(500);
        lazySusan.rotateToDegreesRobotCentric(0);
//        while (slides.isBusy() && opMode.opModeIsActive()){}
        linkages.extend();
        timer.safeDelay(500);
    }
    public void preloadLow(){
        slides.extendLow();
        timer.safeDelay(500);
//        while (slides.isBusy() && opMode.opModeIsActive()){}
        linkages.extendLowAuto();
        timer.safeDelay(500);
    }

    public void preloadDownLow(){
        // down
        dumper.dump();
        timer.safeDelay(500);
        slides.extendMid();
        linkages.retract();
        dumper.intake();
        timer.safeDelay(1100);

        lazySusan.rotateToDegreesRobotCentric(0);
        while(opMode.opModeIsActive() && !lazySusan.isHome()){
            // wait
        }
        slides.retract();
    }


    public void preloadDown(){
        // down
        dumper.dump();
        timer.safeDelay(500);
        linkages.retract();
        dumper.intake();
        timer.safeDelay(1100);

        lazySusan.rotateToDegreesRobotCentric(0);
        while(opMode.opModeIsActive() && !lazySusan.isHome()){
            // wait
        }
        slides.retract();
    }

    public void duckScorePrepRed(){
        dumper.close();
        slides.extendHigh();
        timer.safeDelay(500);
        //lazySusan.rotateToDegreesRobotCentric(-45);
        timer.safeDelay(500);
        linkages.extend();
        timer.safeDelay(800);

        // down
//        down();
//        dumper.dump();
//        timer.safeDelay(500);
//        linkages.retract();
//        dumper.intake();
//        timer.safeDelay(1100);
//
//        lazySusan.rotateToDegrees(0);
//        while(opMode.opModeIsActive() && !lazySusan.isHome()){
//            // wait
//        }
//        slides.retract();
    }

    public void duckScorePrepBlue(){
        dumper.close();
        slides.extendHigh();
        timer.safeDelay(500);
        this.turnTo(duckAngle);
        timer.safeDelay(500);
        linkages.extend();
        timer.safeDelay(800);

        // down
//        down();
//        dumper.dump();
//        timer.safeDelay(500);
//        linkages.retract();
//        dumper.intake();
//        timer.safeDelay(1100);
//
//        lazySusan.rotateToDegrees(0);
//        while(opMode.opModeIsActive() && !lazySusan.isHome()){
//            // wait
//        }
//        slides.retract();
    }

    public void rightSharedHub(){
        slides.extendLow();
        timer.safeDelay(300);
        lazySusan.rotateToDegreesRobotCentric(90);
        timer.safeDelay(300);
        slides.scoreShared();
        linkages.extendShared();
    }

    public void leftSharedHub(){
        slides.extendLow();
        timer.safeDelay(300);
        lazySusan.rotateToDegreesRobotCentric(-90);
        timer.safeDelay(300);
        slides.scoreShared();
        linkages.extendShared();
    }

    public void right(){
        lazySusan.rotateToDegreesRobotCentric(135);
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
        lazySusan.rotateToDegreesRobotCentric(-135);
        linkages.extend();
    }

    public void back(){
        // make it go up first so it doesn't break the REV hubs
        up();
        lazySusan.rotateToDegreesRobotCentric(180);
        linkages.extend();
    }

    public void down(){
        dumper.dump();
        timer.safeDelay(500);
        linkages.retract();
        dumper.intake();
        timer.safeDelay(500);

        boolean isAngle180 = (lazySusan.getTargetDegrees() - 180) < 0.05;
        lazySusan.rotateToDegreesRobotCentric(0);
//        timer.safeDelay(isAngle180 ? 950 : 800);
        while(opMode.opModeIsActive() && !lazySusan.isHome()){
            // wait
        }
        timer.safeDelay(isShared ? 500 : 850);
        slides.retract();
    }

    public void up(){
        goingUp = true;
        dumper.close();
        timer.safeDelay(400);
        slides.extendHigh();
        goingUp = false;
    }
    public void upShared(){
        goingUp = true;
        dumper.close();
        timer.safeDelay(400);
        slides.extendMid();
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

    public String getHeight() {return String.valueOf(slides.getHeight());}

    public void preloadUpAsync(){
        isPreloadUp = true;
    }
    public void preloadMidAsync(){
        isPreloadMid = true;
    }
    public void preloadLowAsync(){
        isPreloadLow = true;
    }
    public void preloadDownAsync(){
        isPreloadDown = true;
    }
    public void preloadDownLowAsync(){
        isPreloadDownLow = true;
    }
    public void duckScorePrepRedAsync(){
        isDuckScorePrepRed = true;
    }
    public void duckScorePrepBlueAsync(){
        isDuckScorePrepBlue = true;
    }
    public void downAsync(){
        isDown = true;
    }
    public void upAsync(){
        isUp = true;
    }
}
