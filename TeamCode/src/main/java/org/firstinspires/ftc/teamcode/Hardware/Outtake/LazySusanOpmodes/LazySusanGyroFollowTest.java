package org.firstinspires.ftc.teamcode.Hardware.Outtake.LazySusanOpmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.IMU.IMU;
import org.firstinspires.ftc.teamcode.Hardware.Outtake.LazySusan;

@TeleOp
public class LazySusanGyroFollowTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        LazySusan lazySusan = new LazySusan(hardwareMap);
        IMU imu = new IMU(hardwareMap);
        waitForStart();
        lazySusan.rotateToDegrees(imu.getHeading());
        while (opModeIsActive()){
            telemetry.addData("Turret Position", lazySusan.getTicks());
            telemetry.addData("Turret Degrees", lazySusan.getDegrees());
            telemetry.addData("Turret Target Position", lazySusan.getTargetTicks());
            telemetry.update();
        }
    }
}
