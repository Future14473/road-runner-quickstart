package org.firstinspires.ftc.teamcode.Hardware.util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.Outtake.Turret;

public class Timer {
    LinearOpMode opMode;
    public Timer (LinearOpMode opMode) {
        this.opMode = opMode;
    }

    public void safeDelay(long delay){
        long start = System.currentTimeMillis();
        while((System.currentTimeMillis() - start < delay) && opMode.opModeIsActive()){
            //wait
        }
    }

    public void safeTurretDelay(long delay){
        long start = System.currentTimeMillis();
        while((System.currentTimeMillis() - start < delay) && opMode.opModeIsActive() && !Turret.RESET){
            //wait
        }
    }
}
