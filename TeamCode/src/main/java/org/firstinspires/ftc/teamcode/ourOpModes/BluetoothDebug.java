package org.firstinspires.ftc.teamcode.ourOpModes;


import android.app.Activity;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Bluetooth.BluetoothClient;
import org.firstinspires.ftc.teamcode.Bluetooth.Interpreter;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.ourMovementLib.Follower;
import org.firstinspires.ftc.teamcode.ourOpModes.resources.IMU;
import org.firstinspires.ftc.teamcode.ourOpModes.resources.RotationUtil;
import org.firstinspires.ftc.teamcode.ourOpModes.robotParts.Wobble_Arm;

import java.util.Arrays;


@TeleOp(name="BT Debug", group="Teleop")
@Disabled
//use DriveWheelIMULocalization for the same functionality instead
public class BluetoothDebug extends LinearOpMode
{
    // Bluetooth Debugging
    BluetoothClient bluetoothClient;
    // Forth Interpreter
    Interpreter interpreter;

    // Declare OpMode members.
    //Mecanum MecanumDrive;
    DcMotor intake;
    DcMotor taco;
    DcMotor shooter_adjuster;
    DcMotorEx shooter;
    CRServo shooter_roller1;
    CRServo shooter_roller2;
    Wobble_Arm wobble_arm;
    IMU imu;
    double headingZero = 0;

    public int testNum = 69420;

    public void toTele(){
        telemetry.addData("Test", "message");
        telemetry.update();
        bluetoothClient.send("test message\n");
    }
    DcMotor test;
    public void startTest(){
        test.setPower(1.0);
    }
    public void stopTest(){
        test.setPower(0);
    }

    public void runOpMode() throws InterruptedException {

        // Intialize Bluetooth Stuff
        interpreter = new Interpreter(
                message -> bluetoothClient.send(message),
                this);

        bluetoothClient = new BluetoothClient(
                (Activity) hardwareMap.appContext, "C8:21:58:6A:A3:A0",
                "00001101-0000-1000-8000-00805F9B34FB",
                 telemetry,
                 response -> interpreter.nexttoks.addAll(
                        Arrays.asList(response.split(" "))));



        imu = new IMU(hardwareMap, telemetry);

        //MecanumDrive = new Mecanum(hardwareMap);
        intake = hardwareMap.get(DcMotor.class, "intake");
        taco = hardwareMap.get(DcMotor.class, "taco");
        wobble_arm = new Wobble_Arm(hardwareMap, BluetoothDebug.this);

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
        test = hardwareMap.get(DcMotor.class, "frontLeft");
        //VuforiaPhone vuforiaPhone = new VuforiaPhone(hardwareMap, telemetry);
        //SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        //drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Follower follower = null; //new Follower(drive, vuforiaPhone, this, telemetry, gamepad1, imu);

        //================= START =====================
        waitForStart();


        bluetoothClient.startHostSession();

        new Thread(() -> interpreter.begin()).start();

        //vuforiaPhone.beginTracking();

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

            if(gamepad2.right_stick_button){
                // high goal
                follower.goTo(-4, 22, -0.31);
            }

            // stop when no one is touching anything
            //MecanumDrive.drive(x, y,
                    //(magnitude > 0.5 && Math.abs(turnPwr) > 0.08) ? 2 * turnPwr : 0);
            //follower.DRIVE(y, x, (magnitude > 0.5 && Math.abs(turnPwr) > 0.08) ? -turnPwr/2 : 0);

            double intakeOut = gamepad2.right_trigger;
            double intakeIn = -gamepad2.left_trigger;
            boolean tacoIn = gamepad2.left_bumper;
            boolean tacoOut = gamepad2.right_bumper;

            // make the intake do the correct trigger, + is outward, - is inward
            intake.setPower(-(intakeIn + intakeOut));

            if(tacoOut)
            {
                taco.setPower(0.5);
            }
            else if(tacoIn)
            {
                taco.setPower(-0.5);
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
            if(shooter.getVelocity() < 1710)
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



