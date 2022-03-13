package org.firstinspires.ftc.teamcode.Hardware.Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.teamcode.Hardware.Duck.Capstone;
import org.firstinspires.ftc.teamcode.Hardware.Outtake.Linkages;

@TeleOp
@Disabled
public class CapstoneTEST extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Capstone capstone = new Capstone(hardwareMap);
        waitForStart();
        while (opModeIsActive()){
            if(gamepad1.dpad_down){
                capstone.down();
            }
            if(gamepad1.dpad_right){
                capstone.collect();
            }
            if(gamepad1.dpad_up){
                capstone.up();
            }
        }
    }
}
