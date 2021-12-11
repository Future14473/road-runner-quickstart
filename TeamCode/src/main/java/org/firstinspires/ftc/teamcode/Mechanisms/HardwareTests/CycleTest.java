package org.firstinspires.ftc.teamcode.Mechanisms.HardwareTests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Mechanisms.Cycler;
import org.firstinspires.ftc.teamcode.Mechanisms.Intake;
import org.firstinspires.ftc.teamcode.Mechanisms.Output;

@TeleOp(group = "1 Teleop")
public class CycleTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Output output = new Output(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Cycler cycler = new Cycler(intake, output, this);
        waitForStart();

        while (opModeIsActive()){
            if(gamepad1.x){
                intake.inNoodlesUp();
            }

            if (gamepad1.y){
                cycler.transferIntakeToDumper();
            }

            if(gamepad1.b){
                cycler.dumperOutPrep();
            }

            if(gamepad1.a){
                cycler.dumpRetract();
            }

            telemetry.update();
        }
    }
}
