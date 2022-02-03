package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class DriveTestSimple extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        DcMotor leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        DcMotor rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        DcMotor rightFront = hardwareMap.get(DcMotor.class, "rightFront");

        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()){

            //forward
            if (gamepad1.x){
                leftFront.setPower(1.0);
                leftRear.setPower(1.0);
                rightRear.setPower(1.0);
                rightFront.setPower(1.0);
            }else if (gamepad1.y){
                leftFront.setPower(1.0);
                leftRear.setPower(1.0);
                rightRear.setPower(-1.0);
                rightFront.setPower(-1.0);
            }else {
                leftFront.setPower(0);
                leftRear.setPower(0);
                rightRear.setPower(0);
                rightFront.setPower(0);
            }

        }
    }
}
