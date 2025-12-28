package Util;

public class PathSegment {
    public Vector last;
    public Vector current;
    public Vector next;

    public Vector lastSegDir;
    public Vector nextSegDir;

    public double transitionDistance;
    public Vector maxVel;
    public int idx;

    public PathSegment() {
        last = new Vector();
        current = new Vector();
        next = new Vector();
        transitionDistance = 0;
        maxVel = new Vector();
        idx = 1;
    }

    public double distAlongSegment(Vector v) {
        return current.minus(v).projMag(last.dir(current));
    }

    public void advance(Path p) {
        idx++;

        last = current;
        current = next;

        if (idx + 1 < p.points().length) {
            next = p.points()[idx + 1];
        }

        lastSegDir = nextSegDir;
        nextSegDir = current.dir(next);
    }

    public void set(Path path, int idx) {
        assert(idx > 0);
        assert(path.points().length > 1);

        last = path.points()[idx - 1];
        current = path.points()[idx];
        next = idx + 1 < path.points().length ? 
            path.points()[idx + 1] :
            current;

        lastSegDir = last.dir(current);
        nextSegDir = current.dir(next);
    }
}
