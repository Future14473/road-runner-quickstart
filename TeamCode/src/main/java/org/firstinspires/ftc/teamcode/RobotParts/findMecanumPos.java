package org.firstinspires.ftc.teamcode.RobotParts;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class findMecanumPos extends LinearOpMode{
    public static int lfpos;
    public static int rfpos;
    public static int rrpos;
    public static int rlpos;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx lf = hardwareMap.get(DcMotorEx.class, "leftFront");
        DcMotorEx rf = hardwareMap.get(DcMotorEx.class, "rightFront");
        DcMotorEx rr = hardwareMap.get(DcMotorEx.class, "rightRear");
        DcMotorEx rl = hardwareMap.get(DcMotorEx.class, "leftRear");


        while (opModeIsActive()){
            lfpos = lf.getCurrentPosition();
            rfpos = rf.getCurrentPosition();
            rrpos = rr.getCurrentPosition();
            rlpos = rl.getCurrentPosition();

            telemetry.addData("Left Front Posistion",lfpos );
            telemetry.addData("Right Front Posistion",rfpos );
            telemetry.addData("Right Rear Posistion",rrpos );
            telemetry.addData("Right Left Posistion",rlpos );
            telemetry.update();
        }



    }
}
