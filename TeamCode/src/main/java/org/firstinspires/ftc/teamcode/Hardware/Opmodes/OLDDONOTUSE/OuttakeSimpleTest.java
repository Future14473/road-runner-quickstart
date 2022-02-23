package org.firstinspires.ftc.teamcode.Hardware.Opmodes.OLDDONOTUSE;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
@Disabled
public class OuttakeSimpleTest extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor front = hardwareMap.get(DcMotor.class, "frontSlide");
        DcMotor back = hardwareMap.get(DcMotor.class, "backSlide");

        waitForStart();
        while (opModeIsActive()){
            if (gamepad1.x){
                front.setPower(1.0);
                back.setPower(-1.0);
            }else if(gamepad1.y){
                front.setPower(1.0);
                back.setPower(1.0);
            }else if (gamepad1.a){
                front.setPower(-1.0);
                back.setPower(1.0);
            }else if(gamepad1.b){
                front.setPower(-1.0);
                back.setPower(-1.0);
            } else{
                front.setPower(0);
                back.setPower(0);
            }

        }
    }
}
