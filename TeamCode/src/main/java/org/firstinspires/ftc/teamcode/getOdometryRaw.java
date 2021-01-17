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
        horizontal = hardwareMap.get(DcMotor.class,"taco");
        left = hardwareMap.get(DcMotor.class,"backLeft");
        right = hardwareMap.get(DcMotor.class,"intake"); //broken wire

        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (opModeIsActive()){
            telemetry.addData("Horizontal Ticks", horizontal.getCurrentPosition());
            telemetry.addData("Left Ticks", left.getCurrentPosition());
            telemetry.addData("Right Ticks", right.getCurrentPosition());
            telemetry.update();
        }
    }
}
