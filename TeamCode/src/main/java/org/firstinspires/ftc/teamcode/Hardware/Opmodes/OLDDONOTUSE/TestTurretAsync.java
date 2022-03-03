package org.firstinspires.ftc.teamcode.Hardware.Opmodes.OLDDONOTUSE;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.Outtake.Turret;

@TeleOp
@Disabled
public class TestTurretAsync extends LinearOpMode {
    public volatile boolean isPreloadUp = false, isPreloadMid = false, isPreloadLow = false,
            isPreloadDown = false, isPreloadDownLow = false,
            isDuckScorePrepRed = false, isDuckScorePrepBlue = false,
            isDown = false, isUp = false;

    @Override
    public void runOpMode() throws InterruptedException {
        Turret turret = new Turret(hardwareMap, this);

        waitForStart();

        new Thread( () -> {
            while (opModeIsActive()){
                if (isPreloadUp){
                    turret.preloadUpBlue();
                    isPreloadUp = false;
                } else if (isPreloadMid){
                    turret.preloadMidBlue();
                    isPreloadMid = false;
                } else if (isPreloadLow){
                    turret.preloadLowBlue();
                    isPreloadLow = false;
                } else if (isPreloadDown){
                    turret.preloadDown();
                    isPreloadDown = false;
                } else if (isPreloadDownLow){
                    turret.preloadDownLow();
                    isPreloadDown = false;
                } else if (isDuckScorePrepRed){
                    turret.duckScorePrepRed();
                    isDuckScorePrepRed = false;
                } else if (isDuckScorePrepBlue){
                    turret.duckScorePrepBlue();
                    isDuckScorePrepBlue = false;
                } else if (isDown){
                    turret.down();
                    isDown = false;
                } else if (isUp){
                    turret.up();
                    isUp = false;
                }
            }
        }).start();

        isPreloadUp = true;
        while (opModeIsActive()){

        }
//        while (opModeIsActive()){
//            if (gamepad1.x){
//                turret.preloadUp();
//                telemetry.addData("Status", "preloadUp Synchronous");
//            }
//            if (gamepad1.y){
//                turret.down();
//                telemetry.addData("Status", "down Synchronous");
//            }
//            if(gamepad1.a){
//                turret.preloadUpAsync();
//                telemetry.addData("Status", "preloadUpAsync");
//                telemetry.addData("isPreloadUp", turret.isPreloadUp);
//            }
//            if(gamepad1.b){
//                turret.downAsync();
//                telemetry.addData("Status", "downAsync");
//                telemetry.addData("isDown", turret.isDown);
//            }
//            telemetry.addData("Status", "running");
//            telemetry.update();
//        }
    }
}
