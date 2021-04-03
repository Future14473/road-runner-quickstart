package org.firstinspires.ftc.teamcode.Medium;

import static java.lang.Math.cos;
import static java.lang.Math.sin;

public class point {
    public double x, y;

    public point(double x, double y){
        this.x = x;
        this.y = y;
    }

    public void rotate(double r){
        double temp_x = x;
        x = x * cos(r) - y * sin(r);
        y = temp_x * sin(r) + y * cos(r);
    }

    public void translate(double h, double v){
        x += h;
        y += v;
    }

    public void rotate(double h, double v, double r){
        translate(-h, -v);
        rotate(r);
        translate(h, v);
    }

    public void scale(double h, double v, double xs, double ys){
        translate(-h, -v);
        x *= xs;
        y *= ys;
        translate(h, v);
    }

    public static point subtract (point a, point b){
        return new point(a.x - b.x, a.y - b.y);
    }
    public static point add (point a, point b){
        return new point(a.x + b.x, a.y + b.y);
    }
    public static point multiply(double b, point a){
        return multiply(a, b);
    }
    public static point multiply(point a, double b){
        return new point(a.x * b, a.y * b);
    }
    public static double cross (point a, point b){
        return a.x * b.y - a.y * b.x;
    }

    public static double length(point a){
        return Math.hypot(a.x, a.y);
    }

    public static double length(double x0, double y0, double x1, double y1){
        return Math.hypot(y1-y0, x1-x0);
    }

    public static double slope(point a){
        if(a.x < 1e4)
            return a.y / 0; // return Infinity or -Infinity
        return a.y / a.x;
    }

    public static point midpoint(point a, point b){
        return new point((a.x + b.x) / 2.0, (a.y + b.y) / 2.0);
    }

    @Override
    public String toString() {
        return "point{ " +
                x + " " +
                y +
                " }";
    }

    @Override
    protected point clone(){
        return new point(x, y);
    }

    public static point intersect_xaxis(double m, double x, double y, double xaxis){
        double y_diff = xaxis - y;
        if(m == 0)
            m = 0.001;

        double x_diff = y_diff / m;

        return new point(x + x_diff, xaxis);
    }

    public static point intersect_yaxis(double m, double x, double y, double yaxis){
        double x_diff = yaxis - x;
        double y_diff = x_diff * m;

        return new point(yaxis, y + y_diff);
    }

    public static point intersect_tl(double m, double x, double y){
        point xaxis = intersect_xaxis(m, x, y, 12);
        point yaxis = intersect_yaxis(m, x, y, 0);

        double a = xaxis.x - x;
        double b = yaxis.x - x;

        if(a > b)
            return xaxis;
        else
            return yaxis;
    }

    public static point intersect_tr(double m, double x, double y){
        point xaxis = intersect_xaxis(m, x, y, 12);
        point yaxis = intersect_yaxis(m, x, y, 8);

        double a = xaxis.x - x;
        double b = yaxis.x - x;

        if(a < b)
            return xaxis;
        else
            return yaxis;
    }

    public static point intersect_bl(double m, double x, double y){
        point xaxis = intersect_xaxis(m, x, y, 0);
        point yaxis = intersect_yaxis(m, x, y, 0);

        double a = xaxis.x - x;
        double b = yaxis.x - x;

        if(a > b)
            return xaxis;
        else
            return yaxis;
    }

    public static point intersect_br(double m, double x, double y){
        point xaxis = intersect_xaxis(m, x, y, 0);
        point yaxis = intersect_yaxis(m, x, y, 8);

        double a = xaxis.x - x;
        double b = yaxis.x - x;

        if(a < b)
            return xaxis;
        else
            return yaxis;
    }
}
