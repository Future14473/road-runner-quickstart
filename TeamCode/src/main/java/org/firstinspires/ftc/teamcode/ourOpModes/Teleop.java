package org.firstinspires.ftc.teamcode.ourOpModes;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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
    DcMotorEx shooter;
    CRServo shooter_roller1;
    CRServo shooter_roller2;
    Wobble_Arm wobble_arm;
    IMU imu;
    double headingZero = 0;

    public void runOpMode() throws InterruptedException {

        imu = new IMU(hardwareMap, telemetry);

        MecanumDrive = new Mecanum(hardwareMap);
        intake = hardwareMap.get(DcMotor.class, "intake");
        taco = hardwareMap.get(DcMotor.class, "taco");


        wobble_arm = new Wobble_Arm(hardwareMap, Teleop.this);

        shooter_roller1 = hardwareMap.get(CRServo.class, "shooter_roller1");
        shooter_roller2 = hardwareMap.get(CRServo.class, "shooter_roller2");
        shooter_roller2.setDirection(DcMotorSimple.Direction.REVERSE);

        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter_adjuster = hardwareMap.get(DcMotor.class, "shooter_adjuster");
        shooter_adjuster.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter_adjuster.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status", "Initialized");

        //Reset wobble arm to up position
        wobble_arm.automaticReleaseWobble();
        waitForStart();


        while (opModeIsActive()){

            double y = -gamepad1.right_stick_y;
            double x = gamepad1.right_stick_x;

            // absolute turning
            double targetDir = -Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 2;
            double magnitude = Math.hypot(gamepad1.left_stick_y, gamepad1.left_stick_x);
            double turnPwr = RotationUtil.turnLeftOrRight(imu.getHeading(), targetDir + headingZero, Math.PI * 2);

            if (! (gamepad1.right_trigger > 0 || gamepad1.left_trigger > 0) ){
                x*= 1.0/3;
                y*= 1.0/3;
            }

            // stop when no one is touching anything
            MecanumDrive.drive(x, y,
                    (magnitude > 0.5 && Math.abs(turnPwr) > 0.08) ? 2 * turnPwr : 0);

            double intakeOut = gamepad2.right_trigger;
            double intakeIn = -gamepad2.left_trigger;
            boolean tacoIn = gamepad2.left_bumper;
            boolean tacoOut = gamepad2.right_bumper;

            // make the intake do the correct trigger, + is outward, - is inward
            intake.setPower(-(intakeIn + intakeOut));

            if(tacoOut)
            {
                taco.setPower(1);
            }
            else if(tacoIn)
            {
                taco.setPower(-1);
            }
            else{
                taco.setPower(0);
            }

            shooter_roller1.setPower((gamepad2.x ? 1 : 0) - (gamepad2.y ? 1 : 0));
            shooter_roller2.setPower((gamepad2.x ? 1 : 0) - (gamepad2.y ? 1 : 0));

//        shooter_roller.setPower(1);
            // shooter adjuster
            shooter_adjuster.setPower((gamepad2.dpad_up ? -0.5 : 0.5) + (gamepad2.dpad_down ? 0.5 : -0.5));
            // shooter.setPower((gamepad2.dpad_left ? -0.795 : 0) + (gamepad2.dpad_right ? 0.795 : 0));
            //shooter.setVelocity((gamepad2.dpad_left ? -0.7 : 0) + (gamepad2.dpad_right ? 0.7 : 0));
            if(shooter.getVelocity() < 1810)
            {
                shooter.setVelocity((gamepad2.dpad_left ? -100 : 0) + (gamepad2.dpad_right ? 100 : 0));
            }
            else
            {
                shooter.setVelocity(0);
            }
            telemetry.addData("Shooter Velocity", shooter.getVelocity());

            // shooter gate
//        gate.setPower(0);
//        if (gamepad1.x){
//            gate.setPower(1);
//        } else if (gamepad1.y){
//            gate.setPower(-1);
//        }

            if (gamepad1.a) {
                wobble_arm.down();
            }
            if (gamepad1.b) {
                wobble_arm.up();
            }
            if (gamepad1.right_bumper) {
                wobble_arm.grab();
            }
            if (gamepad1.left_bumper) {
                wobble_arm.unGrab();
            }

            if(gamepad1.dpad_up)
            {
                headingZero = imu.getHeading();
            }


            telemetry.addData("IsGrabbing? ", wobble_arm.isGrabbing);
            telemetry.addData("Angler Postion:", wobble_arm.getAnglerPosition());
            telemetry.addData("Gripper Postion:", wobble_arm.getGripperPosition());

            //        telemetry.addData("Wobble_Adjuster_position", wobble_angler.getPosition());
            telemetry.update();
        }


    }
}



