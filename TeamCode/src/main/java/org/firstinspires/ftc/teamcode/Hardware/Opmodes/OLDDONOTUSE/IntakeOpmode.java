package org.firstinspires.ftc.teamcode.Hardware.Opmodes.OLDDONOTUSE;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.Intake.Intake;


@TeleOp(group = "1 Teleop")
    public class IntakeOpmode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Intake intake = new Intake(hardwareMap);
        waitForStart();
        while (opModeIsActive()){
            if (gamepad1.right_trigger > 0){
                intake.in();
            } else if (gamepad1.left_trigger > 0){
                intake.out();
            } else{
                intake.stop();
            }
    }
}
}