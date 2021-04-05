package org.firstinspires.ftc.teamcode.Follower.paths;

import org.firstinspires.ftc.teamcode.Follower.PathPoint;

public class DaPath {
    public static PathPoint origin = new PathPoint(0,0,0);

    //test points
    public static PathPoint forward = new PathPoint(0,10,0);
    public static PathPoint strafe = new PathPoint(10,0,0);
    public static PathPoint turn = new PathPoint(0,0, Math.toRadians(90));
}
