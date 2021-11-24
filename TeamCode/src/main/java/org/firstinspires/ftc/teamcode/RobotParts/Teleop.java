package org.firstinspires.ftc.teamcode.RobotParts;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(group = "1 Teleop")
public class Teleop extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Intake intake = new Intake(hardwareMap);
        Output output = new Output(hardwareMap);
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
                    telemetry.addData("Slide Status", "Sliding In");
                }
             if (gamepad1.left_bumper) {
                     intake.slideOut();
                     telemetry.addData("Slide Status", "Sliding Out");
                 }
//             need new FFM Cable first
//            if (gamepad1.dpad_up) {
//                 output.flipInDumper();
//                 telemetry.addData("Dumper status", "Flipped in");
//             }
//            if (gamepad1.dpad_down) {
//                output.flipOutDumper();
//                telemetry.addData("Dumper status", "Flipped out");
//            }





            telemetry.addData("Intake Slide Pos", intake.getSlidePosition());
            telemetry.addData("Retracter Position ", intake.getRetracterPosition());
            telemetry.update();
        }
    }
}
