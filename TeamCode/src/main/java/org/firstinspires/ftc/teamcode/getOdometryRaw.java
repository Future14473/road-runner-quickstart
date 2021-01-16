package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(group = "drive")
public class getOdometryRaw extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor horizontal, left, right;


        waitForStart();
        horizontal = hardwareMap.get(DcMotor.class,"frontLeft");
        left = hardwareMap.get(DcMotor.class,"taco");
        right = hardwareMap.get(DcMotor.class,"backLeft"); //broken wire
        while (opModeIsActive()){
            telemetry.addData("Horizontal Ticks", horizontal.getCurrentPosition());
            telemetry.addData("Left Ticks", left.getCurrentPosition());
            telemetry.addData("Right Ticks", right.getCurrentPosition());
            telemetry.update();
        }
    }
}
