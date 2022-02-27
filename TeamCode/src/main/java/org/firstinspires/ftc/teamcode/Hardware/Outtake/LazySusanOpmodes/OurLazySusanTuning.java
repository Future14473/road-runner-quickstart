package org.firstinspires.ftc.teamcode.Hardware.Outtake.LazySusanOpmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.Outtake.LazySusan;
import org.firstinspires.ftc.teamcode.Hardware.util.Timer;

@TeleOp
@Disabled
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
            lazySusan.rotateToDegreesRobotCentric(90);
            while(lazySusan.isStillMoving()){
                telemetry.addData("CurrentPosition", lazySusan.getDegrees());
                telemetry.addData("TargetPosition", 90);
                telemetry.addData("PID", lazySusan.getPIDCoef());
                telemetry.update();
            }

            timer.safeDelay(2000);

            // turn 0
            lazySusan.rotateToDegreesRobotCentric(0);
            while(lazySusan.isStillMoving()){
                telemetry.addData("CurrentPosition", lazySusan.getDegrees());
                telemetry.addData("TargetPosition", 0);
                telemetry.update();
            }
            timer.safeDelay(2000);
        }
    }
}
