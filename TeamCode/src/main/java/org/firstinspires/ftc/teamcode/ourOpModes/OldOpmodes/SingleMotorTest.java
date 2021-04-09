package org.firstinspires.ftc.teamcode.ourOpModes.OldOpmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(group = "drive")
@Disabled
public class SingleMotorTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotorEx leftFront = hardwareMap.get(DcMotorEx.class, "frontLeft");
        DcMotorEx leftRear = hardwareMap.get(DcMotorEx.class, "backLeft");
         DcMotorEx rightRear = hardwareMap.get(DcMotorEx.class, "backRight");
        DcMotorEx rightFront = hardwareMap.get(DcMotorEx.class, "frontRight");

        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftRear.setDirection(DcMotorEx.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()){
            if (gamepad1.a){
                leftFront.setPower(1);
            }else {
                leftFront.setPower(0);
            }

            if (gamepad1.b){
                leftRear.setPower(1);
            }else {
                leftRear.setPower(0);
            }

            if (gamepad1.x){
                rightFront.setPower(1);
            }else {
                rightFront.setPower(0);
            }

            if (gamepad1.y){
                rightRear.setPower(1);
            }else {
                rightRear.setPower(0);
            }
        }
    }
}
