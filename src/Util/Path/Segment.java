package Util.Path;

import Util.Vector;

public class Segment {
    public Vector dir;
    public double mag;

    public Segment(Vector dir, double mag) {
        this.dir = dir;
        this.mag = mag;
    }

    public Segment(Vector start, Vector end) {
        this(start.dir(end), start.dist(end));
    }

    public Segment() {
        this(new Vector(), 0);
    }

    public Vector dist() {
        return dir.times(mag);
    }

    @Override 
    public String toString() {
        return "Segment(dir: " + dir + ", mag: " + mag + ")";
    }
}
