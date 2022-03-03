package org.firstinspires.ftc.teamcode.ComputerVision;



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
public class RedCapstonePipeline extends OpenCvPipeline {
    Telemetry telemetry;
    Mat mat = new Mat();
    // location is relative to the camera
    public enum Location {
        OUT_OF_FRAME,
        RIGHT,
        LEFT,
    }
    private Location location;

    public static int leftX1 = 30, leftX2 = 160, rightX1 = 200, rightX2 = 315;
    public static int  height1 = 140,  height2 = 230;

    public static double PERCENT_COLOR_THRESHOLD = 0.3;

    public static int lowH = 90, lowS = 90, lowV = 30;
    public static int highH = 255, highS = 255, highV = 255;

    public RedCapstonePipeline(Telemetry t) { telemetry = t; }

    @Override
    public Mat processFrame(Mat input) {
        Rect LEFT_ROI = new Rect(
                new Point(leftX1, height1),
                new Point(leftX2, height2));
        Rect RIGHT_ROI = new Rect(
                new Point(rightX1, height1),
                new Point(rightX2, height2));

        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        // For Yellow only
//        Scalar lowHSV = new Scalar(23, 50, 70);
//        Scalar highHSV = new Scalar(32, 255, 255);

        // For Blue
        Scalar lowHSV = new Scalar(lowH, lowS, lowV);
        Scalar highHSV = new Scalar(highH, highS, highV);

        Core.inRange(mat, lowHSV, highHSV, mat);

        Mat left = mat.submat(LEFT_ROI);
        Mat right = mat.submat(RIGHT_ROI);

        double leftValue = Core.sumElems(left).val[0] / LEFT_ROI.area() / 255;
        double rightValue = Core.sumElems(right).val[0] / RIGHT_ROI.area() / 255;

        left.release();
        right.release();

        telemetry.addData("Left raw value", (int) Core.sumElems(left).val[0]);
        telemetry.addData("Right raw value", (int) Core.sumElems(right).val[0]);
        telemetry.addData("Left percentage", Math.round(leftValue * 100) + "%");
        telemetry.addData("Right percentage", Math.round(rightValue * 100) + "%");

        boolean capLeft = leftValue > PERCENT_COLOR_THRESHOLD;
        boolean capRight = rightValue > PERCENT_COLOR_THRESHOLD;

        if (capLeft) {
            location = Location.LEFT;
            telemetry.addData("Capstone Location", "Left of Camera");
        } else if (capRight){
            location = Location.RIGHT;
            telemetry.addData("Capstone Location", "Right of Camera");
        } else {
            location = Location.OUT_OF_FRAME;
            telemetry.addData("Capstone Location", "Out Of Frame");
        }
        telemetry.update();

        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);

        Scalar notDetectedColor = new Scalar(255, 0, 0);
        Scalar detectedColor = new Scalar(0, 255, 0);

        Imgproc.rectangle(mat, LEFT_ROI, location == Location.LEFT ? detectedColor:notDetectedColor);
        Imgproc.rectangle(mat, RIGHT_ROI, location == Location.RIGHT? detectedColor:notDetectedColor);

        return mat;
    }

    public Location getLocation() {
        return location;
    }
}
