package org.firstinspires.ftc.teamcode.FF_OpenCV;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

@Config
public class JustCameraStream extends OpenCvPipeline {

    Telemetry telemetry;
    public JustCameraStream(Telemetry telemetry){
        this.telemetry = telemetry;
    }

    Mat matStream = new Mat();

    // only if the value is between this range will it be onsidered "blue"
    public static Scalar colorHigh = new Scalar(23,50,70); //TODO tune
    public static Scalar colorLow = new Scalar(2,255,255); // TODO tune

    public static Scalar capColor = new Scalar(23,50,70);
    public static Scalar notCapColor = new Scalar(32,255,255);

    public static Rect leftROI = new Rect(new Point(0,0),
                                        new Point(0,100));

    public static Rect middleROI = new Rect(new Point(100,0),
                                            new Point(20,310));
    public static Rect rightROI = new Rect(new Point(5,62),
                                            new Point(200,40));

    public static double percentColorThreshold = 0.4;

    public enum Location{
        LEFT,
        RIGHT,
        MIDDLE,
        NOT_FOUND
    }
    private Location location;


    @Override
    public Mat processFrame(Mat input) {
        return input;
    }
}
