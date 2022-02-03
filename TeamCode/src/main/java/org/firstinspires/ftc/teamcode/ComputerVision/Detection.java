package org.firstinspires.ftc.teamcode.ComputerVision;
//Ethan Was Here
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

import static org.opencv.imgproc.Imgproc.COLOR_BGR2GRAY;
import static org.opencv.imgproc.Imgproc.COLOR_BGR2HSV;
import static org.opencv.imgproc.Imgproc.COLOR_RGB2BGR;
import static org.opencv.imgproc.Imgproc.CV_SHAPE_ELLIPSE;
import static org.opencv.imgproc.Imgproc.contourArea;
import static org.opencv.imgproc.Imgproc.cvtColor;

public class Detection extends OpenCvPipeline {

    // Variables for rings
    public static final double DISKR = 0.2;
    public static final double DMAXR = DISKR * 2.2;
    public static final double DMINR = DISKR * 0.4;
    public static final double AREADIFF = 0.8; //TODO, precent detection of red goal poles
    public static final double ringW = 12.7; //in cm

    // Variables for wobble goal
    public static final double STICKR = 9;
    public static final double SMAXR = STICKR * 1.2;
    public static final double SMINR = STICKR * 0.5;
    public static final double wobbleW = 20.32;

    // Variables for camera view -> Distance
    public static final double x0 = 20;
    public static final double z0 = 30;
    public static final int yP = 288;
    public static final int xP = 352;
    public static final double theta0 = Math.atan(z0/x0);
    public static final double viewAngle = Math.PI - 2* theta0;
    public static final double realX0 = 35;
    public static final double slope = 0.46;

    //Tracking the ringCounts over time
    int[] ringCounts = {0, 0, 0};

    //Reusable Mats
    Mat recolored = new Mat();
    Mat formatted = new Mat();
    Mat threshold = new Mat();
    Mat subthreshold = new Mat();
    Mat submat = new Mat();
    Mat canvas = new Mat();
    int value = 0;
    int saturation = 0;
    public int stack;

    //Location of tracked object
    double angle = 0;
    double distance = 0;

    Telemetry telemetry;
    Mat output;

    public Detection(Telemetry t){
        this.telemetry = t;
    }

    @Override
    public Mat processFrame(Mat input){
        picSetup(input);
        stack = finalRings();
//        output = markWobble(input, find_wobble(formatted, "blue"));
        output = markRings(input, find_rings(formatted));

        CountRings(find_rings(formatted));

        //telemetry.addData("angle", Math.toDegrees(angle));
        //telemetry.addData("distance", distance);
        telemetry.addData("ring count", stack);
        telemetry.update();

        formatted.release();
        return output;
    }

    // Setting up the picture to have a height of 960 pix and recoloring to BGR
    // Updates the avgValues
    public void picSetup(Mat input) {
        cvtColor(input, formatted, COLOR_RGB2BGR);
        avgValues(formatted);
    }


    // Creates a new Mat object converted from BGR to HSV
    public Mat copyHSV(Mat input){
        cvtColor(input, recolored, COLOR_BGR2HSV);
        return recolored;
    }

    // Creates Arrays with every 15 pixels and sorts them to find the median
    // for HSV Value and Saturation
    public void avgValues(Mat input){
        Mat copy = copyHSV(input);
        int[] values = new int[(int)(Math.ceil(input.width()/5.0) * Math.ceil(input.height()/5.0))];
        int[] saturations = new int[(int)(Math.ceil(input.width()/5.0) * Math.ceil(input.height()/5.0))];
        int index = 0;
        for(int y = 0; y < input.height(); y+=5){
            for(int x = 0; x < input.width(); x+=5){
                values[index] = (int)copy.get(y, x)[2];
                saturations[index] = (int)copy.get(y, x)[1];
                index++;
            }
        }
        copy.release();
        Arrays.sort(values);
        Arrays.sort(saturations);
        int medVal = values[(int)(values.length/2)];
        int medSat = saturations[(int)(saturations.length/2)];

        // Equations derived by testing in different lighting
        // and using linear regression to go from median values to target
        value = Math.max((int)(-0.65*medVal + 165.42), 0);
        saturation = Math.max((int)(1.19*medSat + 38.78), 0);

    }

    public Mat find_yellows(Mat input){
        Mat copy = copyHSV(input);
        Core.inRange(copy, new Scalar(8, saturation, value), new Scalar(20, 255, 255), threshold);
        copy.release();
        return threshold;
    }
    public Mat find_blues(Mat input){
        Mat copy = copyHSV(input);
        Core.inRange(copy, new Scalar(110, saturation + 60, value - 60), new Scalar(135, 255, 255), threshold);
        copy.release();
        return threshold;
    }
    public Mat find_reds(Mat input){
        Mat copy = copyHSV(input);
        Core.inRange(copy, new Scalar(0, saturation, value - 70), new Scalar(11, 255, 255), threshold);
        copy.release();
        return threshold;
    }

    /*
    Sets the entire row of pixels to black if
    a lot of the pixels in the row are black
    */

    public Mat extendEdge(Mat input){
        int rowCount = 0;
        double[] blackpix = {0.0};
        for(int y = 0; y < input.height(); y++){
            for(int x = 0; x < input.width(); x++){
                if(input.get(y, x)[0] == 0.0){
                    rowCount++;
                }
            }
            if(rowCount > 0.5*input.width()) {
                for (int x = 0; x < input.width(); x++) {
                    input.put(y, x, blackpix);
                }
            }
            rowCount = 0;
        }
        return input;
    }

    /*
    Takes in roughly filtered contours and reprocesses
    them to sort out the rings and seperate ring stacks
    into individual rings
    */


    public ArrayList<Rect> find_subcontours(Mat input){
        subthreshold = find_yellows(input);

        Mat gray = new Mat();
        Mat grayblur = new Mat();
        Mat edgesX = new Mat();
        Mat sub = new Mat();
        Mat finalMat = new Mat();
        ArrayList<Rect> output = new ArrayList<>();
        cvtColor(input, gray, COLOR_BGR2GRAY);
        Imgproc.blur(gray, grayblur, new Size(2,2));
        Imgproc.Sobel(grayblur, edgesX, -1, 0, 1);
        Core.subtract(subthreshold, edgesX, sub);

        Core.inRange(sub, new Scalar(215, 215, 215), new Scalar(255, 255, 255), sub);

        finalMat = extendEdge(sub);

        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(finalMat, contours, new Mat(), Imgproc.CHAIN_APPROX_NONE, Imgproc.CHAIN_APPROX_SIMPLE);

        contours.removeIf(m -> {
            Rect rect = Imgproc.boundingRect(m);
            double r = (double)rect.height/rect.width;
            return ((r > DMAXR) || (r < DMINR) || (rect.width < input.width() * 0.6));
        });

        for(MatOfPoint contour: contours){
            Rect rect = Imgproc.boundingRect(contour);
            Imgproc.rectangle(input, rect.tl(), rect.br(), new Scalar(0, 255, 255), 2);
            output.add(rect);
        }
        gray.release();
        grayblur.release();
        edgesX.release();
        sub.release();
        finalMat.release();
        return output;
    }

    public ArrayList<Stack> find_rings(Mat input){

        threshold = find_yellows(input);

        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(threshold, contours, new Mat(), Imgproc.CHAIN_APPROX_NONE, Imgproc.CHAIN_APPROX_SIMPLE);

        //rough filtering
        contours.removeIf(m -> {
            Rect rect = Imgproc.boundingRect(m);
            return (rect.area() < 200) || (rect.height > rect.width) || (rect.height + rect.y < yP*3/5);
        });

        //Rings will be sorted into Stacks in rectsData
        ArrayList<Stack> rectsData= new ArrayList<>();

        //sorting "rings" into stacks

        for(MatOfPoint contour:contours){
            Rect rect = Imgproc.boundingRect(contour);
            Rect oldrect = Imgproc.boundingRect(contour);
            int tempx = rect.x;
            int tempy = rect.y;
            rect.x = (int)Math.max(rect.x - (rect.width*0.1), 0);
            rect.y = (int)Math.max(rect.y - (rect.height*0.1), 0);
            rect.width = (int)Math.min((rect.width*1.1) + tempx, input.width()) - rect.x;
            rect.height = (int)Math.min((rect.height*1.1) + tempy, input.height()) - rect.y;
            submat = new Mat(input.clone(), rect);
            double r = (double)rect.height/rect.width;
            if((r < DMAXR) && (r > DMINR)){
                rectsData.add(new Stack(oldrect));
                continue;
            }
            ArrayList<Rect> moreRects = find_subcontours(submat);
            for(Rect subrect: moreRects){
                double epsilonW = rect.width * 0.3;
                double epsilonH = rect.height * 0.3;
                subrect.x += rect.x;
                subrect.y += rect.y;
                int index = Stack.closeIn(rectsData, subrect, epsilonW, epsilonH);
                if(index == -2){
                    System.out.println("Overlap");
                }
                else if(index == -1){
                    rectsData.add(new Stack(subrect));
                }
                else{
                    rectsData.get(index).addExisting(subrect);
                }
            }

        }

        if(!rectsData.isEmpty()){
            Rect temp = Stack.closestStack(rectsData);
            distance = findDistance(temp, ringW);
            angle = findAngle(temp);
        }
        else{
            angle = 0;
            distance = 0;
        }

        threshold.release();
        return rectsData;
    }

    public Rect wobble_stick(Mat input, MatOfPoint contour){
        Rect rect = Imgproc.boundingRect(contour);
        int newX = rect.x;
        int newY = (int)Math.max(rect.y - (rect.height*0.1), 0);
        int newW = rect.width;
        int newH = (int)Math.min((rect.height * 0.95) + newY, input.height()) - newY;
        submat = new Mat(input.clone(), new Rect(newX, newY, newW, newH));

        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(submat, contours, new Mat(), Imgproc.CHAIN_APPROX_NONE, Imgproc.CHAIN_APPROX_SIMPLE);

        //initial filtering
        contours.removeIf(m -> {
            double height2 = Math.pow(Imgproc.boundingRect(m).height, 2);
            double area = contourArea(m);
            return ((area < 100) || (area < height2/SMAXR) || (area > height2/SMINR));
        });

        MatOfPoint max = new MatOfPoint();
        double area = -1;
        for(MatOfPoint maxFind: contours){
            if(contourArea(maxFind) > area){
                area = contourArea(maxFind);
                max = maxFind;
            }
        }
        Rect maxRect = Imgproc.boundingRect(max);
        Rect finalRect = new Rect(newX + maxRect.x, newY + maxRect.y, maxRect.width, maxRect.height);

        if(area != -1){
            return finalRect;
        }
        else{
            return null;
        }
    }

    public Rect find_wobble(Mat input, String side){

        List<MatOfPoint> contours = new ArrayList<>();

        // Choosing to use red or blue thresholding
        if(side.equals("blue")){
            threshold = find_blues(input);
        }
        else if(side.equals("red")){
            threshold = find_reds(input);
        }
        else{
            System.out.println("ERROR: Not a valid side.");
        }

        Mat kernel = Imgproc.getStructuringElement(CV_SHAPE_ELLIPSE, new Size(3, 3));
        Mat kernelE = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));

        // Dilating and eroding to produce smoother results with less noise
        Imgproc.dilate(threshold, threshold, kernel);
        Imgproc.erode(threshold, threshold, kernelE);
        Imgproc.findContours(threshold, contours, new Mat(), Imgproc.CHAIN_APPROX_NONE, Imgproc.CHAIN_APPROX_SIMPLE);

        Rect closest = new Rect();
        int maxHeight = -1;
        for(MatOfPoint contour: contours){
            Rect rect = Imgproc.boundingRect(contour);
            if((rect.area() > 200) && (rect.width < rect.height)){
                Rect temp = wobble_stick(threshold, contour);
                if(temp != null) {
                    if (temp.height + temp.y > maxHeight) {
                        closest = temp;
                        maxHeight = temp.height + temp.y;
                    }
                }
            }
        }

        return closest;
    }

    public Mat markRings(Mat input, ArrayList<Stack> rectsData){
        canvas = input.clone();
        for(Stack stack: rectsData){
            if(stack.count > 0){
                double Angle = findAngle(stack.fullStack);
                Imgproc.rectangle(canvas, stack.fullStack.tl(), stack.fullStack.br(), new Scalar(255, 255, 0), 1);
                Imgproc.putText(canvas, "" + stack.count,
                        new Point(stack.fullStack.x + 10, stack.fullStack.y - 10),
                        Imgproc.FONT_HERSHEY_SIMPLEX, 0.4, new Scalar(255, 255, 0), 1);
                // Imgproc.putText(copy, "Angle: " + (int)Angle, new Point(data[1] + data[2]/2, data[4] + 100),
                // Imgproc.FONT_HERSHEY_SIMPLEX, 1, new Scalar(0, 255, 0), 2);

            }
        }

        return canvas;
    }

    public int CountRings(ArrayList<Stack> rectsData){
        if(rectsData.isEmpty()){
            ringCounts[0]++;
            return 0;
        }
        int maxCount = Collections.max(rectsData).count;
        if(maxCount == 1){
            ringCounts[1]++;
            return 1;
        }
        else{
            ringCounts[2]++;
            return 4;
        }
    }

    //Finds the maximum index of ringCounts
    public int finalRings(){
        int index = 0;
        for(int i = 0; i<3; i++){
            if(ringCounts[i] > ringCounts[index]){
                index = i;
            }
        }
        return index;
    }


    public Mat markWobble(Mat input, Rect rect){
        //labeling found wobble
        canvas = input.clone();
        double Angle = findAngle(rect);
        Imgproc.rectangle(canvas, rect.tl(), rect.br(), new Scalar(0, 255, 255), 2);
        Imgproc.putText(canvas, "Wobble Goal", new Point(rect.x, rect.y -100), Imgproc.FONT_HERSHEY_SIMPLEX, 1, new Scalar(255, 255, 0), 2);
        Imgproc.putText(canvas, "Angle: " + (int)Angle, new Point(rect.x, rect.y -50 ), Imgproc.FONT_HERSHEY_SIMPLEX, 1, new Scalar(0, 0, 255), 2);
        return canvas;
    }

    public double findAngle(Rect target){
        double dx = (target.x + target.width/2.0) - xP/2.0;
        double camAngle =  Math.toRadians(dx/(double)xP*52);
        double botAngle = Math.atan((Math.tan(camAngle)*distance)/(distance + 8.5));
        if(Math.abs(botAngle - camAngle) > camAngle * 0.2 && camAngle > 0.25){
            telemetry.addData("USEDCAMANGLE!!!", "");
            telemetry.update();
            return camAngle;
        }
        else{
            return botAngle;
        }
    }

    public double findDistance(Rect target, double objWidth){
        return (xP*objWidth/target.width - 8)/0.92/2.54;
    }

}
