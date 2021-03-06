package org.firstinspires.ftc.teamcode.ourOpModes.unit_tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(group = "drive")
public class AllCoreMechanismsTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor taco = hardwareMap.get(DcMotor.class, "taco");
        DcMotor intake = hardwareMap.get(DcMotor.class, "intake");
        DcMotorEx shooter = hardwareMap.get(DcMotorEx.class, "shooter");



        CRServo shooter_roller1, shooter_roller2;

        shooter_roller1 = hardwareMap.get(CRServo.class, "shooter_roller1");
        shooter_roller2 = hardwareMap.get(CRServo.class, "shooter_roller2");
        shooter_roller2.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Hold Down Gamepad 1 D-pad left and right ", "to move shooter");
        telemetry.addData("Hold Down Gamepad 1 A or B", "to move rollers");

        telemetry.addData("Hold Down Gamepad 1 left stick y ", "to move intake");
        telemetry.addData("Hold Down Gamepad 1 right stick y ", "to move taco");
        telemetry.update();
        waitForStart();

        while (!isStopRequested()) {
            intake.setPower(gamepad1.left_stick_y);
            taco.setPower(gamepad1.right_stick_y);

            if(shooter.getVelocity() < 1710)
            {
                shooter.setVelocity((gamepad1.dpad_left ? -100 : 0) + (gamepad1.dpad_right ? 100 : 0));
            }
            else
            {
                shooter.setVelocity(0);
            }
            shooter_roller1.setPower((gamepad1.x ? 1 : 0) - (gamepad1.y ? 1 : 0));
            shooter_roller2.setPower((gamepad1.x ? 1 : 0) - (gamepad1.y ? 1 : 0));
        }
    }
}