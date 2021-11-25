package org.firstinspires.ftc.teamcode.RobotParts;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(group = "1 Teleop")
public class BasicTeleop extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Intake intake = new Intake(hardwareMap);
        Output output = new Output(hardwareMap);
        Duck duck = new Duck(hardwareMap);
        RetractableOdo retractableOdo = new RetractableOdo(hardwareMap);
        waitForStart();

        while (opModeIsActive()){
            intake.setNoodlePower(gamepad1.right_trigger - gamepad1.left_trigger);

            if (gamepad1.x){
                intake.flipIn();
                telemetry.addData("Retracter In Position ", Intake.retractInPos);
                telemetry.addData("Flip Status", "in");
            }

            if (gamepad1.a) {
                intake.flipOut();
                telemetry.addData("Retracter In Position ", Intake.retractOutPos);
                telemetry.addData("Flip Status", "out");
            }

            if (gamepad1.b) {
                intake.transferIntake();
                telemetry.addData("Transfer In Position ", Intake.transferIntakePos);
                telemetry.addData("Transfer Status", "Intake Position");
            }

            if (gamepad1.y) {
                intake.transferOutake();
                telemetry.addData("Transfer In Position ", Intake.transferOutputPos);
                telemetry.addData("Transfer Status", "Output Position");
            }

            if (gamepad1.right_bumper){
                    intake.slideIn();
                    intake.stopNoodles();
                    telemetry.addData("Slide Status", "Sliding In");
                }
             if (gamepad1.left_bumper) {
                     intake.slideOut();
                     intake.inNoodles();
                     telemetry.addData("Slide Status", "Sliding Out");
                 }

            if (gamepad1.dpad_up){
                retractableOdo.upOdo();
                telemetry.addData("Retractable Odo Status ", "Up");
            }
            if (gamepad1.dpad_down){
                retractableOdo.downOdo();
                telemetry.addData("Retractable Odo Status ", "Down");
            }

            duck.setStopSpeed();
            if (gamepad1.dpad_right){
                duck.setBlueSpeed();
                telemetry.addData("Duck Status", "Blue");
            }
            if (gamepad1.dpad_left){
                duck.setRedSpeed();
                telemetry.addData("Duck Status", "Red");
            }
            duck.moveDuck();


            if (gamepad1.right_stick_button){
                output.extend();
                telemetry.addData("Output Status", "Extending");
            }
            if(gamepad1.left_stick_button){
                output.retract();
                telemetry.addData("Output Status", "Retracting");
            }

            if(gamepad2.x){
                output.flipOutDumper();
            }
            if(gamepad2.y){
                output.flipInDumper();
            }

//             need new FFM Cable first
            if (gamepad2.dpad_up) {
                 output.flipInDumper();
                 telemetry.addData("Dumper status", "Flipped in");
             }
            if (gamepad2.dpad_down) {
                output.flipOutDumper();
                telemetry.addData("Dumper status", "Flipped out");
            }





            telemetry.addData("Intake Slide Pos", intake.getSlidePosition());
            telemetry.addData("Retracter Position ", intake.getRetracterPosition());
            telemetry.update();
        }
    }
}
