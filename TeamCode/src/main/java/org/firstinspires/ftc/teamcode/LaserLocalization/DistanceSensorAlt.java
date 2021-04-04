package org.firstinspires.ftc.teamcode.LaserLocalization;

import java.util.ArrayList;
import java.util.List;

public class DistanceSensorAlt {
    static double HEIGHT = 12;
    static double WIDTH = 8;

    // ang is trig rads
    public static void calculate_location(double up, double down, double left, double right, double angle){
        HEIGHT = 12;
        WIDTH = 8;
        List<Object> vertical_partials = partial_solve(up, down, angle);

        HEIGHT = 8;
        WIDTH = 12;
        List<Object> horizontal_partials = partial_solve(left, right, angle);

        HEIGHT = 12;
        WIDTH = 8;
        for(Object partial : horizontal_partials){
            if(partial instanceof line){
                ((line) partial).rotate(HEIGHT/2, HEIGHT/2, Math.PI/2);
            }
        }

        //draw them
    }


    public static List<Object> partial_solve(double up, double down, double angle){
        List<Object> partial_solutions = new ArrayList<>();

        // touching top and bottom
        if(up + down > HEIGHT && up + down < Math.hypot(HEIGHT, WIDTH)){
            double run = Math.sqrt((up+down)*(up+down) - HEIGHT*HEIGHT);

            double robot_height = 12 * up / (up+down);

            if(angle > Math.PI/2)
                partial_solutions.add(new line(
                    new point(run * up / (up+down), robot_height),
                    new point(WIDTH - run * down / (up+down), robot_height)));
            else
                partial_solutions.add(new line(
                        new point(run * down / (up+down), robot_height),
                        new point(WIDTH - run * up / (up+down), robot_height)));
        }

        // touching two adjacent walls
        if(up + down < Math.hypot(HEIGHT, WIDTH)){
            double run = (up + down) * Math.cos(angle);
            double rise = (up + down) * Math.sin(angle);

            double robot_height = rise * up / (up+down);

            if(angle > Math.PI/2) {
                //left bottom
                partial_solutions.add(
                        new point(run * up / (up + down), rise * down / (up + down)));

                // right up
                partial_solutions.add(
                        new point( WIDTH - run * down / (up + down), HEIGHT - rise * up / (up + down)));
            }else {
                // right bottom
                partial_solutions.add(
                        new point(WIDTH - run * up / (up + down), rise * down / (up + down)));

                // left up
                partial_solutions.add(
                        new point(run * down / (up + down), HEIGHT - rise * up / (up + down)));
            }
        }

        return partial_solutions;
    }


    static class line{
        point start;
        point end;

        public line(point a, point b){
            start = a;
            end = b;
        }

        public void rotate(double h, double v, double r) {
            start.rotate(h, v, r);
            end.rotate(h, v, r);
        }
    }
}
