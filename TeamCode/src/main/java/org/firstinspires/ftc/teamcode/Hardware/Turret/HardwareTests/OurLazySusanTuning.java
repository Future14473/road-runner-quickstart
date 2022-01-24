package org.firstinspires.ftc.teamcode.Hardware.Turret.HardwareTests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Hardware.Turret.LazySusan;
import org.firstinspires.ftc.teamcode.Hardware.util.Timer;

@TeleOp
public class OurLazySusanTuning extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        LazySusan lazySusan = new LazySusan(hardwareMap);
//        DcMotorEx lazySusan = hardwareMap.get(DcMotorEx.class, "lazySusan");
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Timer timer = new Timer(this);
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        NanoClock clock = NanoClock.system();
        waitForStart();

        while (opModeIsActive()){
            // turn positive
            lazySusan.rotateToDegrees(90);
            while(lazySusan.isStillMoving()){
                telemetry.addData("CurrentPosition", lazySusan.getDegrees());
                telemetry.addData("TargetPosition", 90);
                telemetry.addData("PID", lazySusan.getPIDCoef());
                telemetry.update();
            }

            timer.safeDelay(2000);

            // turn 0
            lazySusan.rotateToDegrees(0);
            while(lazySusan.isStillMoving()){
                telemetry.addData("CurrentPosition", lazySusan.getDegrees());
                telemetry.addData("TargetPosition", 0);
                telemetry.update();
            }
            timer.safeDelay(2000);
        }
    }
}
