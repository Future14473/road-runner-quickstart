package org.firstinspires.ftc.teamcode.Medium;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

abstract class Solution {
    direction direction;

    enum direction{
        opposed_second_quadrant, down_left
    }

    //public abstract void draw(scaleGraphics g);
    public abstract void rotate(double h, double v, double r);
    public abstract void rotater(double dr);

    public abstract void scale(double h, double v, double xs, double ys);
    public abstract List<intersection> intersectsWith(Solution o);
    abstract protected Object clone();


    static class opposed_faces_solution extends Solution{
        point beg;
        point end;
        public double angle;

        public opposed_faces_solution(point beg, point end, double angle){
            this.direction = direction.opposed_second_quadrant;
            this.beg = beg;
            this.end = end;
            this.angle = angle;
        }

        /*@Override
        public void draw(scaleGraphics g) {
            //g.fillOval(beg.x, beg.y, 0.1, 0.1);
            //g.fillOval(end.x, end.y, 0.1, 0.1);
            g.drawLine(beg.x, beg.y, end.x, end.y);
            //g.drawStringScaled(String.format("%.2f", angle), beg.x + 0.2, beg.y);
        }*/

        @Override
        public void rotate(double h, double v, double r) {
            beg.rotate(h, v, r);
            end.rotate(h, v, r);
            angle += r;
        }

        @Override
        public void rotater(double dr) {
            angle += dr;
        }

        @Override
        public void scale(double h, double v, double xs, double ys) {
            beg.scale(h, v, xs, ys);
            end.scale(h, v, xs, ys);
        }

        @Override
        public List<intersection> intersectsWith(Solution o) {
            // line to line intersection
            if (o instanceof opposed_faces_solution){

                intersection line_line_inter = intersection.line_line_intersect(this, (opposed_faces_solution)o);

                if(line_line_inter != null) {
                    // angles must match
                    //if(!intersection.between_variable(angle, ((opposed_faces_solution) o).angle, ((opposed_faces_solution) o).angle, 1)) {
                        //return new ArrayList<>();
                    //}
                    return Arrays.asList(line_line_inter);
                } else
                    return new ArrayList<>();
            }
            // line to ellipse intersection
            if(o instanceof corner_solution){
                return intersection.ellipse_line_intersection((corner_solution) o, this);
            }

            return null;
        }

        @Override
        protected opposed_faces_solution clone() {
            return new opposed_faces_solution(beg.clone(), end.clone(), angle);
        }

    }

    static class corner_solution extends Solution{
        point center;
        point intercept;
        point startpoint, endpoint;
        double seg_ang_beg, seg_ang_end;
        double sup;
        public corner_solution(double center_x, double center_y,
                               double y_intercept, double x_intercept,
                               point startpoint, point endpoint,
                               double seg_ang_beg, double seg_ang_end){

            direction = direction.down_left;

            this.center = new point(center_x, center_y);

            this.intercept = new point(x_intercept, y_intercept);
            this.sup = y_intercept;

            this.startpoint = startpoint;
            this.endpoint = endpoint;
            this.seg_ang_end = seg_ang_end;
            this.seg_ang_beg = seg_ang_beg;
        }

        @Override
        public void rotate(double h, double v, double r) {
            center.rotate(h, v, r);
            intercept.rotate(h, v, r);
            startpoint.rotate(h, v, r);
            endpoint.rotate(h, v, r);
        }

        @Override
        public void rotater(double dr) {

        }

        @Override
        public void scale(double h, double v, double xs, double ys) {
            center.scale(h, v, xs, ys);
            intercept.scale(h, v, xs, ys);
            startpoint.scale(h, v, xs, ys);
            endpoint.scale(h, v, xs, ys);
        }

        public double slope_at(double x, double y){
            double centerx = center.x;
            double centery = center.y;

            double transx = x - centerx;
            double transy = y = centery;

            double height_axis = intercept.y - centery;
            double width_axis = intercept.x - centerx;

            return Math.tan(-Math.sqrt(1+(sup*sup)/(transx*transx)) * Math.signum(transx*transy));
        }

        @Override
        public List<intersection> intersectsWith(Solution o) {
            if (o instanceof opposed_faces_solution){
                double startAng = Math.atan2(
                        (startpoint.y - center.y),
                        startpoint.x - center.x);
                double endAng = Math.atan2(
                        (endpoint.y - center.y),
                        endpoint.x - center.x);

                /*if(!intersection.between_strict(((opposed_faces_solution) o).angle, seg_ang_beg, seg_ang_end)){
                    System.out.println("intersection refused: " + ((opposed_faces_solution) o).angle +
                            " " +  seg_ang_beg + " " + seg_ang_end);
                    return null;
                }*/
                return intersection.ellipse_line_intersection(this, (opposed_faces_solution) o);
            }

            if(o instanceof corner_solution){
                // fuck quartic equations. Not today
                return intersection.ellipse_ellipse_intersect(this, (corner_solution) o);
            }

            return null;
        }

        /*@Override
        public void draw(scaleGraphics g){
            var points = intersection.ellipse_to_polygon(this);
//            for(var point : points)
//                g.fillOval(point.x, point.y, 0.1, 0.1);

            var x_radius = intercept.x - center.x;
            var y_radius = intercept.y - center.y;

            var x_stretch_ratio = Math.abs(x_radius/y_radius);

            var startAng = Math.atan2(
                    (startpoint.y - center.y) * x_stretch_ratio,
                     startpoint.x - center.x);
            var endAng = Math.atan2(
                    (endpoint.y - center.y) * x_stretch_ratio,
                     endpoint.x - center.x);
            var angDiff = RotationUtil.turnLeftOrRight(startAng, endAng, Math.PI*2);

            startAng = Math.toDegrees(startAng);
            endAng = Math.toDegrees(endAng);
            angDiff = Math.toDegrees(angDiff);

            g.drawArc(center.x, center.y,
                    intercept.x - center.x, intercept.y - center.y,
                    startAng, angDiff);

        }*/

        @Override
        protected corner_solution clone() {
            return new corner_solution(
                    center.x, center.y,
                    intercept.x, intercept.y,
                    startpoint.clone(), endpoint.clone(),
                    seg_ang_beg, seg_ang_end);
        }
    }


}
