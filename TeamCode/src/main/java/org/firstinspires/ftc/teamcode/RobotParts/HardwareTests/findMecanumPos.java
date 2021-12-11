package org.firstinspires.ftc.teamcode.RobotParts.HardwareTests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(group = "!")
public class findMecanumPos extends LinearOpMode{
    public static int lfpos;
    public static int rfpos;
    public static int rrpos;
    public static int rlpos;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx lf = hardwareMap.get(DcMotorEx.class, "leftFront");
        DcMotorEx rf = hardwareMap.get(DcMotorEx.class, "rightFront");
        DcMotorEx rr = hardwareMap.get(DcMotorEx.class, "leftFront");
        DcMotorEx rl = hardwareMap.get(DcMotorEx.class, "leftRear");
        int targetposistion = 100;
        int velocity = 1000;
        waitForStart();
        while (opModeIsActive()){


            rr.setDirection(DcMotor.Direction.REVERSE);
            rl.setDirection(DcMotor.Direction.REVERSE);

            lf.setTargetPosition(targetposistion);
            rf.setTargetPosition(targetposistion);
            rr.setTargetPosition(targetposistion);
            rl.setTargetPosition(targetposistion);
      /*      lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);*/
            rr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
          /*  rl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lf.setVelocity(velocity);
            rf.setVelocity(velocity);*/
            rr.setVelocity(velocity);
         /*   rl.setVelocity(velocity);

            lfpos = lf.getCurrentPosition();
            rfpos = rf.getCurrentPosition();*/
            rrpos = rr.getCurrentPosition();
        //    rlpos = rl.getCurrentPosition();

            telemetry.addData("Left Front Posistion",lfpos );
            telemetry.addData("Right Front Posistion",rfpos );
            telemetry.addData("Right Rear Posistion",rrpos );
            telemetry.addData("Right Left Posistion",rlpos );
            telemetry.update();


        }



    }
}

