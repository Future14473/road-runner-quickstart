package org.firstinspires.ftc.teamcode.Medium;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class intersection {
    public point a;

    public static class point_intersect extends intersection {
        public point_intersect(point a){
            this.a = a;
        }

        @Override
        public String toString() {
            return "Point intersection " + a;
        }
    }

    public static class el_point_intersect extends point_intersect{
        Solution.corner_solution s;

        public el_point_intersect(point a, Solution.corner_solution s) {
            super(a);
            this.s = s;
        }


    }
    public static class line_intersect extends intersection {
        point b;
        public line_intersect(point a, point b){
            this.a = a;
            this.b = b;
        }

        @Override
        public String toString() {
            return "Line intersection " + a + " " + b;
        }

    }

    //https://www.khanacademy.org/computer-programming/c/5567955982876672

    public static List<intersection> ellipse_ellipse_intersect(Solution.corner_solution a, Solution.corner_solution b){
        // l1 is width, l2 is height

        List<point> intersections = new ArrayList<point>();

        List<point> a_poly = ellipse_to_polygon(a);
        List<point> b_poly = ellipse_to_polygon(b);

        // I hate this as much as you,
        // but fuck quartics. Not today

        double startAnga = Math.atan2(
                (a.startpoint.y - a.center.y),
                a.startpoint.x - a.center.x);
        double endAnga = Math.atan2(
                (a.endpoint.y - a.center.y),
                a.endpoint.x - a.center.x);
        double startAngb = Math.atan2(
                (b.startpoint.y - b.center.y),
                b.startpoint.x - b.center.x);
        double endAngb = Math.atan2(
                (b.endpoint.y - b.center.y),
                b.endpoint.x - b.center.x);

        for(int i = 0; i < a_poly.size()-1; i++){
            point a0 = a_poly.get(i);
            point a1 = a_poly.get((i + 1) % (a_poly.size()-1));

            double point_ang_a = Math.atan2(
                    a0.y - a.center.y,
                    a0.x - a.center.x);

            //System.out.println(point_ang_a + " " + startAng_a + " " + endAng_a);
            //System.out.println("\t" + point_ang_b + " " + startAng_b + " " + endAng_b);

            if(!between(point_ang_a, startAnga, endAnga))
                continue;

            for(int j = 0; j < b_poly.size()-1; j++) {

                point b0 = b_poly.get(j);
                point b1 = b_poly.get((j + 1) % (b_poly.size()-1));

                double point_ang_b = Math.atan2(
                        b0.y - b.center.y,
                        b0.x - b.center.x);

                if(!between(point_ang_b, startAngb, endAngb))
                    continue;

               // var inter = fast_line_line_intersection(a0, a1, b0, b1);
                intersection inter = line_line_intersect(
                        new Solution.opposed_faces_solution(a0, a1, 0),
                        new Solution.opposed_faces_solution(b0, b1, 0));
                if(inter == null)
                    continue;

                /*var startAng_a = Math.atan2(
                        a.startpoint.y - a.center.y,
                        a.startpoint.x - a.center.x);
                var endAng_a = Math.atan2(
                        a.endpoint.y - a.center.y,
                        a.endpoint.x - a.center.x);

                var startAng_b = Math.atan2(
                        b.startpoint.y - b.center.y,
                        b.startpoint.x - b.center.x);
                var endAng_b = Math.atan2(
                        b.endpoint.y - b.center.y,
                        b.endpoint.x - b.center.x);

                var point_ang_a = Math.atan2(
                        inter.y - a.center.y,
                        inter.x - a.center.x);

                var point_ang_b = Math.atan2(
                        inter.y - b.center.y,
                        inter.x - b.center.x);

                //System.out.println(point_ang_a + " " + startAng_a + " " + endAng_a);
                //System.out.println("\t" + point_ang_b + " " + startAng_b + " " + endAng_b);

                if(between(point_ang_a, startAng_a, endAng_a) &&
                        between(point_ang_b, startAng_b, endAng_b))*/
                    intersections.add(inter.a);

                /*intersections.addAll(intersection.ellipse_line_intersection(
                        b, new Solution.opposed_faces_solution(a0, a1, 0)
                ));*/
            }
        }

        double x_avg = 0;
        double y_avg = 0;
        double count = 0;
        for(point point : intersections){
            x_avg += point.x;
            y_avg += point.y;
            count++;
        }

        return Arrays.asList(new point_intersect(new point(x_avg/count, y_avg/count)));
    }

    static List<point> buf_0 = new ArrayList<point>();
    static List<point> buf_1 = new ArrayList<point>();

    // line a0 to a1; line b0 to b1
    public static point fast_line_line_intersection(point a0, point a1, point b0, point b1){

        point a_bound_min = new point(
                Math.min(a0.x, a1.x),
                Math.min(a0.y, a1.y)
        );

        point a_bound_max = new point(
                Math.max(a0.x, a1.x),
                Math.max(a0.y, a1.y)
        );

        point b_bound_min = new point(
                Math.min(b0.x, b1.x),
                Math.min(b0.y, b1.y)
        );

        point b_bound_max = new point(
                Math.max(b0.x, b1.x),
                Math.max(b0.y, b1.y)
        );

        // the bounding boxes intersect!
        if(
                a_bound_max.x > b_bound_min.x && a_bound_min.x < b_bound_max.x &&
                a_bound_max.y > b_bound_min.y && a_bound_min.y < b_bound_max.y
                //point.length(point.subtract(point.midpoint(a0, a1), point.midpoint(b0, b1))) < 2
        ){

            double lowest_max_x = Math.min(a_bound_max.x, b_bound_max.x);
            double lowest_max_y = Math.min(a_bound_max.y, b_bound_max.y);

            double highest_min_x = Math.max(a_bound_min.x, b_bound_min.x);
            double highest_min_y = Math.max(a_bound_min.y, b_bound_min.y);

            //buf_0.add(point.midpoint(a0, a1));
            //buf_1.add(point.midpoint(b0, b1));

            //return new point((a0.x + a1.x)/2.0, (a0.y + a1.y)/2.0);
            return point.midpoint(point.midpoint(a0, a1), point.midpoint(b0, b1));
            //return new point((highest_min_x + lowest_max_x) / 2.0, (highest_min_y + lowest_max_y) / 2.0);
        }

        return null;
    }

    public static List<point> ellipse_to_polygon(Solution.corner_solution a){
        double x = a.center.x;
        double y = a.center.y;

        double w = Math.abs(a.intercept.x - a.center.x);
        double h = Math.abs(a.intercept.y - a.center.y);
//TODO
//        Ell ellipse = new java.awt.geom.Ellipse2D.Double(x - w, y - h, 2 * w, 2 * h);
//        var pathIterator = new FlatteningPathIterator(ellipse.getPathIterator(new AffineTransform()), 0.01);
//        var points = new ArrayList<point>();
//
//        while (!pathIterator.isDone()){
//            var coords = new double[6];
//            pathIterator.currentSegment(coords);
//
//            points.add(new point(coords[0], coords[1]));
//
//            pathIterator.next();
//        }

        return new ArrayList<point>();
    }

    //https://stackoverflow.com/questions/563198/how-do-you-detect-where-two-line-segments-intersect
    public static intersection line_line_intersect(Solution.opposed_faces_solution a, Solution.opposed_faces_solution b){
        //t = (q − p) × s / (r × s)
        //u = (q − p) × r / (r × s)

        point p = a.beg;
        point q = b.beg;

        point r = point.subtract(a.end, a.beg);
        point s = point.subtract(b.end, b.beg);

        double cross_r_s = point.cross(r, s);
        double q_p_cross_r = point.cross(point.subtract(q, p), r);

        if (Math.abs(cross_r_s) < 1e-4) {
            if (Math.abs(q_p_cross_r) < 1e-4) {
                double avg_x = (a.beg.x + b.beg.x)/2.0;

                double a_max_y = Math.max(a.beg.y, a.end.y);
                double b_max_y = Math.max(b.beg.y, b.end.y);

                double a_min_y = Math.min(a.beg.y, a.end.y);
                double b_min_y = Math.min(b.beg.y, b.end.y);

                return new line_intersect(
                        new point(avg_x, Math.max(a_min_y, b_min_y)),
                        new point(avg_x, Math.min(a_max_y, b_max_y)));
            }
            return null;
        }


        double t = point.cross(point.subtract(q, p), s) / cross_r_s;
        double u = q_p_cross_r / cross_r_s;

        // p + t r
        if (0 <= t && t <= 1)
            if (0 <= u && u <= 1)
                return new point_intersect(point.add(p, point.multiply(t, r)));

        return null;
    }

    public static intersection bastard(){
        return null;
    }

    public static List<intersection> ellipse_line_intersection(Solution.corner_solution ellipse, Solution.opposed_faces_solution line){
        double j = ellipse.center.x;
        double k = ellipse.center.y;

        double w = Math.abs(ellipse.intercept.x - j);
        double h = Math.abs(ellipse.intercept.y - k);

        double m = point.slope(point.subtract(line.end, line.beg));
        double b = line.beg.y - line.beg.x * m;


        //System.out.print("eli " + ellipse.center + " " + j + " " + k + " " + m + " " + b);

        if(Double.isInfinite(m)) {
            b = line.beg.x; // special behavior of e_l_i where, if m is infinity, b becomes x-intercept
            b -= j;
        }else{
            b -= m*j; // x translation on y-intercept
            b -= k;   // y translation on y-intercept
        }

        List<point> intersect = ellipse_line_intersection(w, h, m, b);

        for(point point : intersect){
            point.translate(j, k);
        }

        double startAng = Math.atan2(
                ellipse.startpoint.y - ellipse.center.y,
                ellipse.startpoint.x - ellipse.center.x);
        double endAng = Math.atan2(
                ellipse.endpoint.y - ellipse.center.y,
                ellipse.endpoint.x - ellipse.center.x);

        List<intersection> true_intersect = new ArrayList<intersection>();

        for(point point : intersect){
            double point_ang = Math.atan2(
                    point.y - ellipse.center.y,
                    point.x - ellipse.center.x);

            if(between(point.x, line.beg.x, line.end.x) &&      // within line segment x
                    between(point.y, line.beg.y, line.end.y) && // within line segment y
                    between(point_ang, startAng, endAng))       // within angles
            {
                if( (line.angle - ellipse.slope_at(point.x, point.y)) % Math.PI/2 < 0.05)
                    true_intersect.add(new point_intersect(point));
                else
                    System.out.println("rejected: " + line.angle + " " + ellipse.slope_at(point.x, point.y));
            }
        }

        if(true_intersect.size() > 0) {
            //System.out.println(" Solution " + true_intersect.get(0));
        }else{
            //System.out.println();
        }

        return true_intersect;
    }

    static boolean between(double val, double a, double b){
        double close = Math.abs(b - a) < 1e-4/2.0 ? 1e-4/2.0 : 0;

        if(a > b){
            return val <= a + close && val >= b - close;
        }else{
            return val <= b + close && val >= a - close;
        }
    }

    static boolean between_strict(double val, double a, double b){
        double close = -0.2;

        if(a > b){
            return val <= a + close && val >= b - close;
        }else{
            return val <= b + close && val >= a - close;
        }
    }

    static boolean between_variable(double val, double a, double b, double set){
        double close = -set;

        if(a > b){
            return val <= a + close && val >= b - close;
        }else{
            return val <= b + close && val >= a - close;
        }
    }

    // special behavior where, if m is infinity, b becomes x-intercept
    // https://www.symbolab.com/solver/step-by-step/solve%20for%20x%2C%20%5Cfrac%7B%5Cleft(x%5Cright)%5E%7B2%7D%7D%7Bw%5E%7B2%7D%7D%2B%5Cfrac%7B%5Cleft(mx%2Bb%5Cright)%5E%7B2%7D%7D%7Bh%5E%7B2%7D%7D%3D1
    private static List<point> ellipse_line_intersection(double w, double h, double m, double b){
        //System.out.println("\nprimitive eli " + w + " " + h + " " + m + " " + b);

        // case m is infinity
        if(Double.isInfinite(m)){
            double y_discr = h*h * (1 - (b*b)/(w*w));

            if (y_discr < 0)
                return new ArrayList<point>();

            double y0 = Math.sqrt(y_discr);
            double y1 = -y0;
            double x = b;

            //System.out.println("\t prim sol " + x + " " + y0 + " " + x + " " + y1);

            return Arrays.asList(new point(x, y0), new point(x, y1));
        }

        double x_discr = m*m*w*w + h*h - b*b;
        if (x_discr < 0)
            return new ArrayList<point>();

        double x0 = w * (h * Math.sqrt(x_discr) - m*b*w) / (m*m*w*w + h*h);
        double x1 = -x0;

        double y0 = m * x0 + b;
        double y1 = m * x1 + b;

        //System.out.println("\t prim sol " + x0 + " " + y0 + " " + x1 + " " + y1);

        return Arrays.asList(new point(x0, y0), new point(x1, y1));
    }
}
