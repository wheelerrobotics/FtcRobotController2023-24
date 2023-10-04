package org.firstinspires.ftc.teamcode.utility;

import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.atan;
import static java.lang.Math.cos;
import static java.lang.Math.sin;

public class Pose {

    // question: should I have the delta vars in here?
    public double x = 0;
    public double y = 0;
    public double r = 0;
    public Pose(double x, double y, double r){
        this.x = x;
        this.y = y;
        this.r = r;
    }
    public void setPose(double x, double y, double r) {
        this.x = x;
        this.y = y;
        this.r = r;
    }
    public Pose getPoseRobotCentric(double curr) {

        double theta = (x==0) ? ((PI/2) * (y/abs(y))) : atan(y/x) + (x < 0 ? PI : 0);
        double mag = Math.sqrt(x*x + y*y);

        return new Pose(mag*cos(theta+curr), mag*sin(theta+curr), r);
    }
}
