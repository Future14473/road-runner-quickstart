package org.firstinspires.ftc.teamcode.RobotParts;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ourOpModes.DirtyGlobalVariables;
import org.firstinspires.ftc.teamcode.ourOpModes.QingAuto;
import org.firstinspires.ftc.teamcode.ourOpModes.resources.Timing;
@Config
public class ShooterFlicker {
    Servo flicker;
    Timing timer;
    LinearOpMode opMode;
    Telemetry telemetry;

    public static int flickbackdelay = 150, firstdelay = 615, seconddelay = 750;

    public ShooterFlicker(HardwareMap hardwareMap, LinearOpMode opMode, Telemetry telemetry) {
        flicker = hardwareMap.get(Servo.class, "flicker");
        timer = new Timing(opMode);
        this.opMode = opMode;
        this.telemetry = telemetry;
    }

    public double flickIn = 0.52, flickOut = 0.36;

    public void flickIn() { // left bumper
        flicker.setPosition(flickIn);
    }

    public void flickOut() { // right bumper
        flicker.setPosition(flickOut);
    }

    public double getPosition() {
        return flicker.getPosition();
    }

    int waitTime = 500;

    public void flickThrice(Shooter shooter) {
        for (int i = 0; i < 3; i++) {
            wait_until_speed(shooter);

            if(i==0)
                delay(900);
            if(i==1)
                delay(700);
            if(i==2)
                delay(350);

            flickOut();
            delay(1000);

            flickIn();
            delay(200);
        }
    }

    void wait_until_speed(Shooter shooter){
        while (opMode.opModeIsActive() && Math.abs(shooter.getShooterVelocity() - shooter.getTargetVelocity()) > 30){

        }
    }

    public void fastTriFlick(Shooter shooter){
        for (int i = 0; i < 3; i++) {
            if(i == 0){
                DirtyGlobalVariables.telemetry.addData("first shot", shooter.getShooterVelocity());
            }
            if(i == 1){
                shooter.increaseSpeed();
                timer.safeDelay(firstdelay);
                DirtyGlobalVariables.telemetry.addData("second shot", shooter.getShooterVelocity());
            }
            if(i == 2){
                timer.safeDelay(seconddelay);
                DirtyGlobalVariables.telemetry.addData("third shot", shooter.getShooterVelocity());
            }
            flickOut();
            timer.safeDelay(flickbackdelay);
            flickIn();
        }
        shooter.decreaseSpeed();
    }

    public void singleFlick(){
        flickOut();
        timer.safeDelay(waitTime);
        flickIn();
        timer.safeDelay(200);
    }

    public void delay(long delay){
        long start = System.currentTimeMillis();
        while((System.currentTimeMillis() - start < delay) && opMode.opModeIsActive()){
            //wait
        }
    }
}

