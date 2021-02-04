package org.firstinspires.ftc.teamcode.cv;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.openftc.easyopencv.OpenCvPipeline;

import org.opencv.core.*;
import org.opencv.core.Point;
import org.opencv.imgproc.Imgproc;

import static org.opencv.imgproc.Imgproc.*;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

public class Detection extends OpenCvPipeline {

    // Variables for rings
    public static final double DISKR = 0.2;
    public static final double DMAXR = DISKR * 2;
    public static final double DMINR = DISKR * 0.4;
    public static final double AREADIFF = 0.5;

    // Variables for wobble goal
    public static final double STICKR = 9;
    public static final double SMAXR = STICKR * 1.2;
    public static final double SMINR = STICKR * 0.5;

    // Variables for camera view -> Distance
    public static final double x0 = 20;
    public static final double z0 = 30;
    public static final int yP = 288;
    public static final int xP = 352;
    public static final double theta0 = Math.atan(z0/x0);
    public static final double viewAngle = Math.PI - 2* theta0;
    public static final double realX0 = 35;
    public static final double slope = 0.46;

    ArrayList<double[]> wobbles = new ArrayList<>();
    int wobbleIterations = 10;

    Mat recolored = new Mat();
    Mat formatted = new Mat();
    Mat threshold = new Mat();
    Mat subthreshold = new Mat();
    Mat submat = new Mat();
    Mat canvas = new Mat();
    int value = 0;
    int saturation = 0;
    int stack;

    double angle = 0;

    Telemetry telemetry;
    Mat output;

    Point objPosition = new Point();

    public Detection(Telemetry t){
        this.telemetry = t;
    }

    @Override
    public Mat processFrame(Mat input){
        picSetup(input);
        output = markRings(input, find_rings(formatted));
        stack = CountRings(find_rings(formatted));
        telemetry.addData("Ring Count", stack);
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
        Core.inRange(copy, new Scalar(7, saturation, value), new Scalar(30, 255, 255), threshold);
        copy.release();
        return threshold;
    }

    public Mat find_blues(Mat input){
        Mat copy = copyHSV(input);
        Core.inRange(copy, new Scalar(110, saturation, value), new Scalar(125, 255, 255), threshold);
        copy.release();
        return threshold;
    }
    public Mat find_reds(Mat input){
        Mat copy = copyHSV(input);
        Core.inRange(copy, new Scalar(0, saturation, value), new Scalar(11, 255, 255), threshold);
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
            return (rect.area() < 700) || (rect.height > rect.width) || (rect.height + rect.y < yP/2);
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
            angle = jankAngle(Stack.closestStack(rectsData));
        }

        threshold.release();
        return rectsData;
    }

    public Boolean wobble_stick(Mat input, MatOfPoint contour){
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
            return ((area < 700) || (area < height2/SMAXR) || (area > height2/SMINR));
        });

        return (contours.size() > 0);
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

        Mat kernel = Imgproc.getStructuringElement(CV_SHAPE_ELLIPSE, new Size(5, 5));
        Mat kernelE = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size((int)(input.width()/62.5), 1));

        // Dilating and eroding to produce smoother results with less noise
        Imgproc.dilate(threshold, threshold, kernel);
        Imgproc.erode(threshold, threshold, kernelE);
        Imgproc.findContours(threshold, contours, new Mat(), Imgproc.CHAIN_APPROX_NONE, Imgproc.CHAIN_APPROX_SIMPLE);

        // rough filtering
        contours.removeIf(m -> {
            Rect rect = Imgproc.boundingRect(m);
            double r = (double) rect.height/rect.width;
            return ((rect.area() < 500) || (rect.width > rect.height)) || !wobble_stick(threshold, m);
        });

        MatOfPoint max = new MatOfPoint();
        double area = -1;
        for(int i = 0; i<contours.size();i++){
            MatOfPoint contour = contours.get(i);
            if(contourArea(contour) > area){
                area = contourArea(contour);
                max = contour;
            }
        }
        Rect maxRect = Imgproc.boundingRect(max);
        Rect finalRect = new Rect(maxRect.x, maxRect.y, maxRect.width, (int)Math.round(maxRect.height*(28.0/24)));
        addWobble(finalRect);
        kernel.release();
        kernelE.release();
        return finalRect;
    }

    public Mat markRings(Mat input, ArrayList<Stack> rectsData){
        canvas = input.clone();
        for(Stack stack: rectsData){
            if(stack.count > 0){
                double Angle = jankAngle(stack.fullStack);
                Imgproc.rectangle(canvas, stack.fullStack.tl(), stack.fullStack.br(), new Scalar(255, 255, 0), 2);
                Imgproc.putText(canvas, "" + stack.count,
                        new Point(stack.fullStack.x + 10, stack.fullStack.y - 20),
                        Imgproc.FONT_HERSHEY_SIMPLEX, 0.4, new Scalar(0, 255, 0), 2);
                //Imgproc.putText(copy, "Angle: " + (int)Angle, new Point(data[1] + data[2]/2, data[4] + 100),
                // Imgproc.FONT_HERSHEY_SIMPLEX, 1, new Scalar(0, 255, 0), 2);

            }
        }

        return canvas;
    }

    public int CountRings(ArrayList<Stack> rectsData){
        if(rectsData.isEmpty()){
            return 0;
        }
        int maxCount = Collections.max(rectsData).count;
        if(maxCount == 1){
            return 1;
        }
        else{
            return 4;
        }
    }


    public Mat markWobble(Mat input, Rect rect){
        //labeling found wobble
        canvas = input.clone();
        double Angle = find_Angle(rect);
        Imgproc.rectangle(canvas, rect.tl(), rect.br(), new Scalar(0, 255, 255), 2);
        Imgproc.putText(canvas, "Wobble Goal", new Point(rect.x, rect.y -100), Imgproc.FONT_HERSHEY_SIMPLEX, 1, new Scalar(255, 255, 0), 2);
        Imgproc.putText(canvas, "Angle: " + (int)Angle, new Point(rect.x, rect.y -50 ), Imgproc.FONT_HERSHEY_SIMPLEX, 1, new Scalar(0, 0, 255), 2);
        return canvas;
    }

    public double jankAngle(Rect target){
        double dx = (target.x + target.width/2.0) - xP/2.0;
        return Math.toRadians(dx/(double)xP*70);
    }


    //ANYTHING BEYOND THIS POINT IS NOT USED















    public Point bestWobble(){
        if(wobbles.isEmpty()){
            return new Point(0, 0);
        }
        int maxWobblesI = 0;
        double maxWobbles = 0;
        for(int i = 0; i<wobbles.size(); i++){
            double[] wobble = wobbles.get(i);
            if(wobble[0] > maxWobbles){
                maxWobbles = wobble[0];
                maxWobblesI = i;
            }
        }

        double[] maxWobbleFinal = wobbles.get(maxWobblesI);
        wobbles.clear();
        telemetry.addData("Best Wobble: ", new Point(maxWobbleFinal[1], maxWobbleFinal[2]));
        telemetry.update();

        return new Point(maxWobbleFinal[1], maxWobbleFinal[2]);
    }

    public void addWobble(Rect rect){
        Point rel = find_Point(rect);
        boolean match = false;
        for(int i = 0; i<wobbles.size(); i++){
            double[] wobble = wobbles.get(i);
            double dist = Math.hypot(rel.x - wobble[1], rel.y - wobble[2]);
            if(dist < 150){
                wobble[1] = (wobble[0] * wobble[1] + rel.x)/(wobble[0] + 1);
                wobble[2] = (wobble[0] * wobble[2] + rel.y)/(wobble[0] + 1);
                wobble[0]++;
                match = true;
                break;
            }
        }
        if(!match){
            double[] added = {1, rel.x, rel.y};
            wobbles.add(added);
        }
    }

    public Point bestRing(ArrayList<double[]> rectsData){
        return new Point();
    }

    public double find_Angle(Rect obj){
        double centerX = obj.x + (double)obj.width/2;
        double centerY = obj.y + obj.height;
        double y = pix2Y(centerY);
        double x = pix2RealX(centerX, centerY);
        double angle = Math.atan(y/x) - Math.PI/2;
        if(y/x < 0){
            angle += Math.PI;
        }
        return Math.toDegrees(angle);
    }

    public double pix2Y(double pixY){
        double angle = (yP - pixY)/yP * viewAngle;
        return z0 * Math.tan(theta0 + angle) + x0;
    }

    public double pix2RealX(double pixX, double pixY){
        double y = pix2Y(pixY);
        double fullX = realX0 + y*slope;
        return (pixX - xP/2.0)/(xP) * fullX;
    }

    public Point find_Point(Rect obj){
        double centerX = obj.x + (double)obj.width/2;
        double centerY = obj.y + obj.height;
        double y = pix2Y(centerY);
        double x = pix2RealX(centerX, centerY);
        return new Point(x, y);
    }

}