package org.firstinspires.ftc.teamcode.drive.opmode.gradde.mathUtils;

import java.lang.reflect.Array;
import java.util.ArrayList;

public class StandardDev {
    public static double getStandardDev(ArrayList<Double> loss){
        double sum = 0;

        for (Double val : loss){
            sum += Math.pow( val - 0 , 2.0);
        }
        return Math.sqrt(sum/(loss.size()-1));
    }

    static double getAverageArray(double[] arr){
        double sum = 0;
        for (double val : arr){
            sum += val;
        }

        return sum/arr.length;
    }
}
