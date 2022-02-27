package org.firstinspires.ftc.teamcode.Hardware.Opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.Outtake.LazySusan;
import org.firstinspires.ftc.teamcode.Hardware.Outtake.Turret;

@TeleOp
@Config
public class TestTurret extends LinearOpMode {
    public static double degrees = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        Turret turret = new Turret(hardwareMap, this);
        LazySusan lazySusan = new LazySusan(hardwareMap, telemetry);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        turret.up();
        waitForStart();
        double prevDegrees = 0;

        while (opModeIsActive()){
            if(prevDegrees != degrees){
                lazySusan.rotateToDegreesRobotCentric(degrees);
            }
            prevDegrees = degrees;
            telemetry.addData("Current Degrees", lazySusan.getDegrees());
            telemetry.addData("Current Degrees", lazySusan.getDegrees());
            telemetry.update();
        }
        turret.down();
    }
}
