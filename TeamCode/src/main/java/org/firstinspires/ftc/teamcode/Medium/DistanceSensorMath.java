package org.firstinspires.ftc.teamcode.Medium;

//import android.graphics.Color;

//import java.awt.*;
import java.util.ArrayList;
import java.util.List;

public class DistanceSensorMath {


    public static List<intersection> distanceToPosition(double left, double right, double up, double down){
        List<Solution> solutions_vert = findSolution(up, down, 8, 12);
        List<Solution> solutions_hori = findSolution(left, right, 12, 8);

        for(Solution solution : solutions_hori) {
            solution.rotater(-Math.PI/2);
            solution.rotate(4, 4, Math.PI / 2);
        }

        //g.setColor(new Color(100, 200, 100));
        /*for(Solution solution : solutions_hori)
            solution.draw(g);

        //g.setColor(new Color(255, 100, 100));
        for(Solution solution : solutions_vert)
            solution.draw(g);*/

        // find intersect
        //g.setColor(Color.red);
        List<intersection> intersections = new ArrayList<intersection>();
        for(Solution a : solutions_hori){
            for(Solution b : solutions_vert){
                List<intersection> intersect = a.intersectsWith(b);
                if(intersect != null) {
                    intersections.addAll(intersect);
                }
            }
        }
        //PRINT SOLUTIONS =================================
        //System.out.println("Intersections");
        /*for(intersection intersect : intersections){
            //System.out.println(intersect);

            if(intersect instanceof intersection.point_intersect){
                g.fillOval(intersect.a.x, intersect.a.y, 0.08, 0.08);
            }

            if(intersect instanceof intersection.line_intersect){
                intersection.line_intersect li = (intersection.line_intersect) intersect;
                g.drawLine(intersect.a.x, intersect.a.y, li.b.x, li.b.y, 3);
            }
        }*/

        return intersections;
    }

    static int rand(){
        return (int)(Math.random() * 255);
    }

    static List<Solution> findSolution (double sup, double inf, double width, double height) {
        List<Solution> solutions = new ArrayList<Solution>();

        // solve for Bottom Right to Top Left symmetry
        Solution vsol = vertical_solve(sup, inf, width, height);
        if(vsol != null) solutions.add(vsol);

        Solution hsol = horizontal_solve(sup, inf, width, height);
        if(hsol != null) solutions.add(hsol);

//        var csol = corner_solve(sup, inf, width, height);
//        if(csol != null)solutions.add(csol);
//
//        var csol_180 = corner_solve(inf, sup, width, height);
//        if(csol_180 != null) csol_180.rotate(width/2.0, height/2.0, -Math.PI);
//        if(csol_180 != null) solutions.add(csol_180);

        return solutions;
    }

    //solution for Bottom Right to Top Left symmetry
    private static Solution.opposed_faces_solution vertical_solve (double sup, double inf, double WIDTH, double HEIGHT){
        double length = sup + inf;
        double diagonal = Math.hypot(WIDTH, HEIGHT);

        if(length > diagonal || length < HEIGHT)
            return null;

        double delta_x = Math.sqrt(length*length - HEIGHT*HEIGHT);
        double down_delta_x = inf/length * delta_x;
        double up_delta_x = sup/length * delta_x;

        // negative slope solution
        double left_limit = up_delta_x;
        double right_limit = WIDTH - down_delta_x;
        double height = inf/length * HEIGHT;

        return new Solution.opposed_faces_solution(
                new point(left_limit, height),
                new point(right_limit, height),
                Math.atan2(HEIGHT, delta_x));
    }

    //solution for Bottom Right to Top Left symmetry
    private static Solution.opposed_faces_solution horizontal_solve (double sup, double inf, double WIDTH, double HEIGHT){
        double length = sup + inf;
        double diagonal = Math.hypot(WIDTH, HEIGHT);

        if(length > diagonal || length < WIDTH)
            return null;

        double delta_y = Math.sqrt(length*length - WIDTH*WIDTH);
        double down_delta_y = inf/length * delta_y;
        double up_delta_y = sup/length * delta_y;

        // left higher than right solution
        double lower_limit = down_delta_y;
        double upper_limit = HEIGHT - up_delta_y;
        double width = (1-inf/length) * WIDTH;

        return new Solution.opposed_faces_solution(
                new point(width, lower_limit),
                new point(width, upper_limit),
                Math.atan2(delta_y, WIDTH));
    }

    private static Solution.corner_solution corner_solve(double sup, double inf, double WIDTH, double HEIGHT){
        double length = sup + inf;
        double diagonal = Math.hypot(WIDTH, HEIGHT);

        if(length > diagonal)
            return null;

        point startpoint = new point(sup,0.1);
        point endpoint = new point(0.1,inf);

        double startAng = Math.PI, endAng = Math.PI/2;
        // if start point is not flat
        if(length > WIDTH){
            double y_elev = Math.sqrt(length*length - WIDTH*WIDTH);
            double ratio = inf/length;
            double x_of = WIDTH * (1-ratio);
            double y_of = y_elev * ratio;

            startpoint = new point(x_of, y_of);
            startAng = Math.atan2(y_elev, WIDTH);
        }

        // if end point is not vertical
        if(length > HEIGHT){
            double x_width = Math.sqrt(length*length - HEIGHT*HEIGHT);
            double ratio = inf/length;
            double x_of = x_width * (1-ratio);
            double y_of = HEIGHT * ratio;

            endpoint = new point(x_of, y_of);
            startAng = Math.atan2(HEIGHT, x_width);
        }

        return new Solution.corner_solution(0, 0, inf, sup, startpoint, endpoint, startAng, endAng);

    }


}
