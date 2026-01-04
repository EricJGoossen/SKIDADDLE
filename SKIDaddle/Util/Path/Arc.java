package Util.Path;

import Util.Vector;

public class Arc {
    public Segment current;

    public Vector point;
    public int index;

    public Vector vel;
    public double transition;

    public Arc(Segment current, Vector point, int index, Vector vel, double transition) {
        this.current = current;
        this.point = point;
        this.index = index;
        this.vel = vel;
        this.transition = transition;
    }

    public Arc(Vector start, Vector mid, Vector end, int index) {
        current = new Segment(start, mid);
        
        point = mid;
        this.index = index;
        
        vel = new Vector();
        transition = 0.0;
    }

    public Arc() {
        this.current = new Segment();
        this.vel = new Vector();
        this.transition = 0.0;
    }

    public double distAlongSegment(Vector v) {
        return point.minus(v).projMag(current.dir);
    }

    public double dist() {
        return Math.max(current.mag - transition, 0.0);
    }

    public void setVel(Vector vel) {
        this.vel = vel;
    }
    
    public void setTransition(double transition) {
        this.transition = transition;
    }

    @Override
    public String toString() {
        return "Arc(index: " + index + ", midpoint: " + point + ", vel: " + vel + ", transition: " + transition + ")";
    }
}
