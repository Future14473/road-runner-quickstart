package org.firstinspires.ftc.teamcode.Hardware.Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.Outtake.Dumper;
import org.firstinspires.ftc.teamcode.Hardware.Outtake.Turret;
import org.firstinspires.ftc.teamcode.Hardware.util.Timer;
@TeleOp
public class DumperTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Dumper dumper = new Dumper(hardwareMap);
        Turret turret = new Turret(hardwareMap, this);
        Timer timer = new Timer(this);
        waitForStart();
        while (opModeIsActive()){
            if(gamepad1.x){
                dumper.halfDump();
            }
            if(gamepad1.y){
                dumper.dump();
            }
            if(gamepad1.a){
                dumper.intake();
            }
            if (gamepad1.b){
                dumper.halfDump();
                timer.safeDelay(200);
                dumper.moreHalfDump();
                timer.safeDelay(200);
                dumper.dump();
                timer.safeDelay(250);
            }
            if(gamepad1.right_bumper){
                dumper.close();
            }
            if (gamepad1.left_bumper){
                dumper.moreHalfDump();
            }
        }
    }
}
