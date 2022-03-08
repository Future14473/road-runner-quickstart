package org.firstinspires.ftc.teamcode.Hardware.Opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.Duck.Duck;
import org.firstinspires.ftc.teamcode.Hardware.Intake.Intake;
import org.firstinspires.ftc.teamcode.Hardware.Outtake.BoxSensor;
import org.firstinspires.ftc.teamcode.Hardware.Outtake.LazySusan;
import org.firstinspires.ftc.teamcode.Hardware.Outtake.Linkages;
import org.firstinspires.ftc.teamcode.Hardware.Outtake.Turret;
import org.firstinspires.ftc.teamcode.Hardware.util.Timer;
import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class RedTeleop extends LinearOpMode {
    public static double forwardSpeed = 0.6;
    public static double turnSpeed = 0.6;

    @Override
    public void runOpMode() throws InterruptedException {
        Turret turret = new Turret(hardwareMap, this);
        Intake intake = new Intake(hardwareMap);
        LazySusan lazySusan = new LazySusan(hardwareMap);
        SampleTankDrive tankDrive = new SampleTankDrive(hardwareMap);
        Duck duck = new Duck(hardwareMap);
        Timer timer = new Timer(this);
        BoxSensor colorSensor = new BoxSensor(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        boolean rightBumperPrev = false, aPrevState = false, xPrevState = false,
                leftButton = false, rightButton = false;

        turret.readyToIntake();

        intake.drop();

        waitForStart();

        new Thread( () -> {
            while (opModeIsActive()) {
//                tankDrive.setWeightedDrivePower(new Pose2d(gamepad1.left_stick_y * forwardSpeed, 0, gamepad1.right_stick_x * turnSpeed));
                tankDrive.setPowerDir(gamepad1.left_stick_y * forwardSpeed, gamepad1.right_stick_x * turnSpeed);
            }
        }).start();

        while (opModeIsActive()) {
            // Cycling
            if(gamepad1.right_trigger > 0){
                intake.stop();
            } else {
                intake.smartInAggressiveOutRed(turret, timer);
            }

            if (gamepad1.left_trigger > 0){
                turret.down();
            }

            if (gamepad1.dpad_up){
                turret.up();
                turret.outputRed();
            }

            // Toggle Linkages
            if (rightBumperPrev != gamepad1.right_bumper) {
                if (gamepad1.right_bumper) {
                    turret.toggleLinkages();
                }
            } rightBumperPrev = gamepad1.right_bumper;

            // Duck _______________________________
            duck.setStop();
            if (gamepad1.left_bumper){
                duck.setRed();
            } duck.move();

            // Scoring Height Changes
            if (aPrevState != gamepad1.a){
                if (gamepad1.a){
                    turret.isShared = !turret.isShared;
                }
            } aPrevState = gamepad1.a;

            // Manual Override Turret
            if(gamepad1.dpad_left){
               turret.lazySusanIncrementLeft();
            }
            if(gamepad1.dpad_right){
                turret.lazySusanIncrementRight();
            }
            if(gamepad1.dpad_up){
                turret.slidesIncrementUp();
            }
            if(gamepad1.dpad_down){
                turret.slidesIncrementDown();
            }
            if(gamepad1.circle){
                // puts the current turret and lazy susan position to 0
                turret.resetEncoders();
            }

            // Drivetrain speed increases ______________________
            if (leftButton != gamepad1.left_stick_button){
                if (gamepad1.left_stick_button){
                    forwardSpeed += 0.15;
                }
            } leftButton = gamepad1.left_stick_button;

            if (rightButton != gamepad1.right_stick_button){
                if (gamepad1.right_stick_button){
                    turnSpeed += 0.15;
                }
            } rightButton = gamepad1.right_stick_button;

            // Telemetry _______________-
            telemetry.addData("isShared", turret.isShared);
            telemetry.addData("Forward Speed", forwardSpeed);
            telemetry.addData("Turn Speed", turnSpeed);
            telemetry.addData("Dumper Block", colorSensor.getColor());
            telemetry.addData("Toggle Pos", Linkages.toggleIndex);
            telemetry.addData("ColorSensor Detect", colorSensor.hasBlock() ? "filled" : "empty");
            telemetry.addData("Dumper isFilled", turret.hasBlock() ? "filled" : "empty");
            telemetry.addData("Turret Angle", lazySusan.getDegrees());
            telemetry.addData("Slide Height", turret.getHeight());
            telemetry.update();

            //           if (gamepad1.right_trigger > 0) {
//               intake.smartIn(turret, timer);
//           }
           /* if (gamepad1.right_trigger > 0) {
                intake.smartInAggressiveOutRed(turret, timer);
            }
            *//*else if (gamepad1.left_trigger > 0){
                intake.out();
            }*//* else {
                intake.stop();
            }

*/
            // TODO: 3/7/22 make this a button
//           else if (gamepad1.left_trigger > 0){
//                intake.out();
//            } else {
//               intake.stop();
//           }

            // TODO: 3/7/22 manual overrides later
//            if (gamepad1.dpad_up) {
//                turret.up();
//            }
//
//            if (gamepad1.dpad_right) {
//                turret.right();
//            }
//            if (gamepad1.dpad_left) { // make retract and down all in one method later
//                turret.left();
//            }


//            if (game1yPrevState != gamepad1.y) {
//                if (gamepad1.y) {
//                    turret.toggleLinkages();
//                }
//            }
//            game1yPrevState = gamepad1.y;

            // TODO: 3/7/22 add lazy susan turret overrides

//            if ((gamepad1.right_stick_x > 0) && !turret.isDown()){
//                lazySusan.turnRightIncrement();
//            }
//            if ((gamepad1.right_stick_x < 0) && !turret.isDown()){
//                lazySusan.turnLeftIncrement();
//            }

//            if (gamepad1.left_bumper){
//                turret.letGoEmergency();
//                intake.out();
//                timer.safeDelay(800);
//            }
            //            if (gamepad1.right_bumper){
//                turret.linkOutShared();
//            }
//            if (gamepad1.y){
//                turret.toggleLinkages();
//            }

            // TODO: 3/7/22 manual control reset the whole turret
//            if (gamepad1.left_stick_button){
//                turret.resetWholeTurret();
//            }

//            rightBumperPrev = rightBumperCurrState;
        }
    }
}
