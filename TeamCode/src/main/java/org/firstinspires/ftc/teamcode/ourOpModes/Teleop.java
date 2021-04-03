package org.firstinspires.ftc.teamcode.ourOpModes;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;


import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.ourMovementLib.Follower;
import org.firstinspires.ftc.teamcode.ourOpModes.resources.IMU;
import org.firstinspires.ftc.teamcode.ourOpModes.resources.RotationUtil;
import org.firstinspires.ftc.teamcode.ourOpModes.robotParts.RingCollector;
import org.firstinspires.ftc.teamcode.ourOpModes.robotParts.Wobble_Arm;




@TeleOp(name="Teleop", group="Teleop")
//@Disabled
//use DriveWheelIMULocalization for the same functionality instead
public class Teleop extends LinearOpMode
{
    // Declare OpMode members.
    //Mecanum MecanumDrive;
    DcMotor intake;
    DcMotor taco;
    DcMotorEx shooter;
    CRServo shooter_roller1;
    CRServo shooter_roller2;
    Wobble_Arm wobble_arm;
    IMU imu;
    double headingZero = 0;

    public void runOpMode() throws InterruptedException {

        imu = new IMU(hardwareMap, telemetry);

        //MecanumDrive = new Mecanum(hardwareMap);
        RingCollector ringCollector = new RingCollector(hardwareMap);


        wobble_arm = new Wobble_Arm(hardwareMap, Teleop.this);


        shooter = hardwareMap.get(DcMotorEx.class, "shooter");

        telemetry.addData("Status", "Initialized");

        //Reset wobble arm to up position
        wobble_arm.automaticReleaseWobble();
        VuforiaPhone vuforiaPhone = new VuforiaPhone(hardwareMap, telemetry);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Follower follower = new Follower(drive, vuforiaPhone, this, telemetry, gamepad1, imu);

        waitForStart();

        vuforiaPhone.beginTracking();

        while (opModeIsActive()){

            double y = -gamepad1.right_stick_y;
            double x = gamepad1.right_stick_x;

            // absolute turning
            double targetDir = -Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 2;
            double magnitude = Math.hypot(gamepad1.left_stick_y, gamepad1.left_stick_x);
            double turnPwr = RotationUtil.turnLeftOrRight(imu.getHeading(), targetDir + headingZero, Math.PI * 2);

            if (! (gamepad1.right_trigger > 0 || gamepad1.left_trigger > 0) ){
                x*= 1.0/3;
//                y*= 1.0/3;
            }

            if(gamepad2.right_stick_button){
                // high goal
                follower.goTo(-4, 22, -0.31);
            }

            follower.DRIVE(y, x, (magnitude > 0.5 && Math.abs(turnPwr) > 0.08) ? -turnPwr/2 : 0);

            // keep the velocity at 1710 at all times
            if(shooter.getVelocity() < 1710)
            {
                shooter.setVelocity(200); //todo check if this actually works
            }
            telemetry.addData("Shooter Velocity", shooter.getVelocity());

            if (gamepad2.left_trigger != 0){
                ringCollector.collect();
            }
            if (gamepad2.right_trigger != 0){
                ringCollector.out();
            }

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

            telemetry.update();
        }


    }
}


