package org.firstinspires.ftc.teamcode.ourOpModes;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;



import org.firstinspires.ftc.teamcode.ourOpModes.resources.IMU;
import org.firstinspires.ftc.teamcode.ourOpModes.resources.RotationUtil;
import org.firstinspires.ftc.teamcode.ourOpModes.robotParts.Mecanum;
import org.firstinspires.ftc.teamcode.ourOpModes.robotParts.Wobble_Arm;




@TeleOp(name="Teleop", group="Teleop")
//@Disabled
//use DriveWheelIMULocalization for the same functionality instead
public class Teleop extends LinearOpMode
{
    // Declare OpMode members.
    Mecanum MecanumDrive;
    DcMotor intake;
    DcMotor taco;
    DcMotor shooter_adjuster;
    DcMotor shooter;
    CRServo shooter_roller1;
    CRServo shooter_roller2;
    Wobble_Arm wobble_arm;
    IMU imu;

    public void runOpMode() throws InterruptedException {

        imu = new IMU(hardwareMap, telemetry);

        MecanumDrive = new Mecanum(hardwareMap);
        intake = hardwareMap.get(DcMotor.class, "intake");
        taco = hardwareMap.get(DcMotor.class, "taco");


        wobble_arm = new Wobble_Arm(hardwareMap);

        shooter_roller1 = hardwareMap.get(CRServo.class, "shooter_roller1");
        shooter_roller2 = hardwareMap.get(CRServo.class, "shooter_roller2");
        shooter_roller2.setDirection(DcMotorSimple.Direction.REVERSE);

        shooter = hardwareMap.get(DcMotor.class, "shooter");
        shooter_adjuster = hardwareMap.get(DcMotor.class, "shooter_adjuster");
        shooter_adjuster.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter_adjuster.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status", "Initialized");

        waitForStart();


        while (opModeIsActive()){

        double y = -gamepad1.right_stick_y;
        double x = gamepad1.right_stick_x;

        // absolute turning
        double targetDir = -Math.atan2(gamepad1.left_stick_y,gamepad1.left_stick_x) - Math.PI/2;
        double magnitude = Math.hypot(gamepad1.left_stick_y,gamepad1.left_stick_x);
        double turnPwr = RotationUtil.turnLeftOrRight(imu.getHeading(), targetDir, Math.PI * 2);

        // stop when no one is touching anything
        MecanumDrive.drive(x/3, y/3,
                (magnitude > 0.5 && Math.abs(turnPwr) > 0.08)? turnPwr:0);

        double intakeOut = gamepad2.right_trigger;
        double intakeIn = -gamepad2.left_trigger;

        // make the intake do the correct trigger, + is outward, - is inward
        intake.setPower(-(intakeIn + intakeOut));
        taco.setPower((intakeIn + intakeOut));

        shooter_roller1.setPower((gamepad2.x?1:0) - (gamepad2.y?1:0));
        shooter_roller2.setPower((gamepad2.x?1:0) - (gamepad2.y?1:0));

//        shooter_roller.setPower(1);
        // shooter adjuster
        shooter_adjuster.setPower( (gamepad2.dpad_up?-0.5:0.5) + (gamepad2.dpad_down?0.5:-0.5) );
        shooter.setPower((gamepad2.dpad_left?-0.795:0) + (gamepad2.dpad_right?0.795:0) );

        // shooter gate
//        gate.setPower(0);
//        if (gamepad1.x){
//            gate.setPower(1);
//        } else if (gamepad1.y){
//            gate.setPower(-1);
//        }

        if (gamepad1.a){
            wobble_arm.down();
            telemetry.addData("arm", "down");
        }
        if (gamepad1.b){
            wobble_arm.releaseWobble();
            telemetry.addData("release ", "wobble");
        }
        if (gamepad1.right_bumper){
            telemetry.addData("pressed bumbper", true);
            wobble_arm.grab();
        }


        telemetry.addData("intake power", intakeIn);
//        telemetry.addData("Wobble_Adjuster_position", wobble_angler.getPosition());
        telemetry.update();
    }



}
}



