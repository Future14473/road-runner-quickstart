package org.firstinspires.ftc.teamcode.LaserLocalization;

import java.util.ArrayList;
import java.util.List;

public class DistanceSensorAlt {
    static double HEIGHT = 12;
    static double WIDTH = 8;

    // ang is trig rads
    public static geom calculate_location( double left, double right, double up, double down, double angle) {
        scaleGraphics g = new scaleGraphics();

        // robot angle to trig angle
        angle -= Math.PI/2;

        // keep within [-360,360]
        if(angle > Math.PI*2 || angle < 0)
            angle %= (Math.PI*2);

        // keep within [0,360]
        if(angle < 0)
            angle += Math.PI*2;

        // keep within second and forth quadrants (flip over x-axis if not)
        boolean v_flip;
        if(v_flip = (angle % Math.PI) < Math.PI/2)
            angle = 2*Math.PI - angle;

        // compute
        geom solution = calculate_location_quadrantII(left, right, up, down, angle, g);
        if(solution == null)
            return null;

        if (v_flip)
            solution.scale(WIDTH / 2, HEIGHT / 2, 1, -1);

        solution.draw(g);

        return solution;
    }

    static geom calculate_location_quadrantII(double left, double right, double up, double down, double angle, scaleGraphics g){
        List<geom> vertical_partials = partial_solve(up, down, angle, WIDTH, HEIGHT);

        List<geom> horizontal_partials = partial_solve(left, right, angle, HEIGHT, WIDTH);

        for(geom partial : horizontal_partials)
            partial.rotate(WIDTH/2, WIDTH/2, Math.PI/2);

        geom sol = null;
        // will return no solution when robot is aligned to within ~0.1 degs of x axis
        if(horizontal_partials.size() > 0 && vertical_partials.size() > 0)
            sol = line_line_intersect((line)horizontal_partials.get(0), (line)vertical_partials.get(0));
        else
            return null;

        //draw them
        g.setColor(scaleGraphics.Color.red);
        for(geom partial : vertical_partials)
            partial.draw(g);

        g.setColor(scaleGraphics.Color.green);
        for(geom partial : horizontal_partials)
            partial.draw(g);

        return sol;
    }



    static List<geom> partial_solve(double up, double down, double angle, double width, double height){
        List<geom> partial_solutions = new ArrayList<>();

        double rise = (up + down) * Math.sin(fold_range(angle));
        double run = (up + down) * Math.cos(fold_range(angle));

        double up_ratio = up / (up+down);
        double down_ratio = down / (up+down);

        if(angle > Math.PI/2){
            partial_solutions.add(new line(
                            //left bottom
                            new point(run * up_ratio, rise * down_ratio),
                            // right up
                            new point(width - run * down_ratio, height - rise * up_ratio)
                    ));
        }else{
            partial_solutions.add(new line(
                            // right bottom
                            new point(width - run * up_ratio, rise * down_ratio),
                            // left up
                            new point(run * down_ratio, height - rise * up_ratio)
                    ));
        }
        return partial_solutions;
    }

    //https://stackoverflow.com/questions/563198/how-do-you-detect-where-two-line-segments-intersect
    static geom line_line_intersect(line a, line b){
        //t = (q − p) × s / (r × s)
        //u = (q − p) × r / (r × s)

        point p = a.start;
        point q = b.start;

        point r = point.subtract(a.end, a.start);
        point s = point.subtract(b.end, b.start);

        double cross_r_s = point.cross(r, s);
        double q_p_cross_r = point.cross(point.subtract(q, p), r);

        if (Math.abs(cross_r_s) < 1e-4) {
            if (Math.abs(q_p_cross_r) < 1e-4) {
                double avg_x = (a.start.x + b.start.x)/2.0;

                double a_max_y = Math.max(a.start.y, a.end.y);
                double b_max_y = Math.max(b.start.y, b.end.y);

                double a_min_y = Math.min(a.start.y, a.end.y);
                double b_min_y = Math.min(b.start.y, b.end.y);

                // collinear
                return new line(
                        new point(avg_x, Math.max(a_min_y, b_min_y)),
                        new point(avg_x, Math.min(a_max_y, b_max_y)));
            }

            // parallel, not intersecting
            return null;
        }


        double t = point.cross(point.subtract(q, p), s) / cross_r_s;
        double u = q_p_cross_r / cross_r_s;

        double spoof = 0.1;

        // p + t r
        if (0-spoof <= t && t <= 1+spoof)
            if (0-spoof <= u && u <= 1+spoof)
                return point.add(p, point.multiply(t, r));

        System.out.println("2nd null line a: " + a + " line b: " + b);
        return null;
    }

    // keep within first quadrant
    static double fold_range(double r){
        if(r > Math.PI)
            r -= Math.PI;

        if(r > Math.PI/2)
            r = Math.PI - r;

        return r;
    }


    public interface geom {
        void rotate(double h, double v, double r);

        void translate(double h, double v);

        void scale(double h, double v, double xs, double ys);

        void draw (scaleGraphics g);
    }

    public static class line implements geom{
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

        public void translate(double h, double v){
            start.translate(h, v);
            end.translate(h, v);
        }

        public void scale(double h, double v, double xs, double ys) {
            start.scale(h, v, xs, ys);
            end.scale(h, v, xs, ys);
        }

        @Override
        public void draw(scaleGraphics g) {
            g.drawLine(start.x, start.y, end.x, end.y);
        }

        @Override
        public String toString(){
            return "line: " + start + " " + end;
        }
    }
}
