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
public class CapDetector extends OpenCvPipeline {

    Telemetry telemetry;
    public CapDetector (Telemetry telemetry){
        this.telemetry = telemetry;
    }

    Mat matStream = new Mat();

    // only if the value is between this range will it be onsidered "blue"
    public static Scalar colorHigh = new Scalar(0,94,92); //TODO tune
    public static Scalar colorLow = new Scalar(180, 78, 84); // TODO tune

    public static Scalar capColor = new Scalar(255,0,0);
    public static Scalar notCapColor = new Scalar(0,255,0);

    public static final Rect leftROI = new Rect(new Point(0,0),
                                        new Point(0,100));

    public static final Rect middleROI = new Rect(new Point(100,0),
                                            new Point(20,310));
    public static final Rect rightROI = new Rect(new Point(5,62),
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
        // process Frame gets the camera stream and allows you to add annotations too]
        Imgproc.cvtColor(input, matStream, Imgproc.COLOR_RGB2HSV );

        //matrix becomes grayscale (within range = white, out of range = black)
        Core.inRange(matStream, colorLow, colorHigh, matStream);

        //extrat ROI from image
        Mat left = matStream.submat(leftROI);
        Mat right = matStream.submat(rightROI);
        Mat middle = matStream.submat(middleROI);

        //see percentage of matrix becomes white
        //first because only 1 channel in grayscalle image
        double leftValue = Core.sumElems(left).val[0] / leftROI.area() / 255;
        double middleValue = Core.sumElems(middle).val[0] / middleROI.area() / 255;
        double rightValue = Core.sumElems(right).val[0] / rightROI.area() / 255;

        left.release();
        right.release();
        middle.release();

        telemetry.addData("% of White per ROI left", Math.round(leftValue*100)+"%");
        telemetry.addData("% of White per ROI middle", Math.round(middleValue *100)+"%");
        telemetry.addData("% of White per ROI right", Math.round(rightValue *100)+"%");

        boolean isLeft = leftValue > percentColorThreshold;
        boolean isMiddle = middleValue > percentColorThreshold;
        boolean isRight = rightValue > percentColorThreshold;

        if (isLeft){
            location = Location.LEFT;
        } else if (isMiddle){
            location = Location.MIDDLE;
        }else {
            location = Location.RIGHT;
        }
        telemetry.addData("isLeft", isLeft);
        telemetry.addData("isMiddle", isMiddle);
        telemetry.addData("isRight",isRight);
        telemetry.update();

        //draw rectangles to visualize
        Imgproc.cvtColor(matStream, matStream, Imgproc.COLOR_GRAY2RGB);

        Imgproc.rectangle(matStream, leftROI, (location == Location.LEFT) ? capColor: notCapColor);
        Imgproc.rectangle(matStream, middleROI, (location == Location.MIDDLE) ? capColor: notCapColor);
        Imgproc.rectangle(matStream, rightROI, (location == Location.RIGHT) ? capColor: notCapColor);
        return matStream;
    }
}
