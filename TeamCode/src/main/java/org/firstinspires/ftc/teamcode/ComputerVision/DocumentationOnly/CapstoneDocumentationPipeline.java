package org.firstinspires.ftc.teamcode.ComputerVision.DocumentationOnly;



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
public class CapstoneDocumentationPipeline extends OpenCvPipeline {
    Telemetry telemetry;
    Mat mat = new Mat();
    public enum Location {
        LEFT,
        RIGHT,
        MIDDLE,
    }
    private Location location;

    public static int leftX1 = 30, leftX2 = 120, rightX1 = 190, rightX2 = 270;
    public static int  height1 = 15,  height2 = 145;

    public static double PERCENT_COLOR_THRESHOLD = 0.3;

    public static int lowH = 90, lowS = 90, lowV = 120;
    public static int highH = 255, highS = 255, highV = 255;

    public CapstoneDocumentationPipeline(Telemetry t) { telemetry = t; }

    @Override
    public Mat processFrame(Mat input) {

        Rect LEFT_ROI = new Rect(
                new Point(leftX1, height1),
                new Point(leftX2, height2));
        Rect RIGHT_ROI = new Rect(
                new Point(rightX1, height1),
                new Point(rightX2, height2));

        Scalar notDetectedColor = new Scalar(255, 0, 0);
        Scalar detectedColor = new Scalar(0, 255, 0);

        Imgproc.rectangle(input, LEFT_ROI, notDetectedColor);
        Imgproc.rectangle(input, RIGHT_ROI, detectedColor);

        return input;
    }

    public Location getLocation() {
        return location;
    }
}
