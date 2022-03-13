package org.firstinspires.ftc.teamcode.Hardware.Opmodes.NewArchive;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.Outtake.Linkages;

@TeleOp
@Disabled
public class LinkageTEST extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Linkages linkages = new Linkages(hardwareMap);
        waitForStart();
        while (opModeIsActive()){
            if(gamepad1.x){
                linkages.extend();
            }
            if(gamepad1.y){
                linkages.retract();
            }
            if(gamepad1.dpad_right){
//                linkages
            }
        }
    }
}
