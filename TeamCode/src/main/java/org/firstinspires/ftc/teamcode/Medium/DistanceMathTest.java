package org.firstinspires.ftc.teamcode.Medium;


import java.util.List;

public class DistanceMathTest {


    public static void main(String[] args) {
        List<intersection> i = DistanceSensorMath.distanceToPosition(4.48, 3.69, 6.65, 5.36);

        for(intersection in : i){
            System.out.println(in);
        }
    }
}
