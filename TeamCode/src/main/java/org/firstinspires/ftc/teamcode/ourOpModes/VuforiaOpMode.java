package org.firstinspires.ftc.teamcode.ourOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
@Disabled
@TeleOp(name="Vuforia Nav", group ="Concept")
public class VuforiaOpMode extends LinearOpMode {
    private static final float mmPerInch        = 25.4f;
    @Override
    public void runOpMode() throws InterruptedException {

        VuforiaPhone vuforia = new VuforiaPhone(hardwareMap, telemetry);

        waitForStart();

        vuforia.beginTracking();

        while (!isStopRequested()){
            OpenGLMatrix location = vuforia.getLocation();
            if(location != null) {
                VectorF translation = location.getTranslation();
                Orientation rotation = Orientation.getOrientation(location, EXTRINSIC, XYZ, DEGREES);
                telemetry.addData("x", translation.get(0) / mmPerInch);
                telemetry.addData("y", translation.get(1) / mmPerInch);
                telemetry.addData("r", rotation.thirdAngle);
                telemetry.update();
            }

        }

        vuforia.stopTracking();
    }
}
