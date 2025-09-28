package Util;

import java.util.ArrayList;
import java.util.Arrays;

import Simulator.Draw.Graph;

/**
 * Represents a path composed of points generated from Hermite splines or Bezier curves.
 * Provides functionality to create paths, flatten Bezier curves, and visualize paths.
 */
public class Path {
    private static final double ACCEL = Constants.MAX_ACCELERATION;
    private static final double EPS = Constants.ZERO_TOLERANCE;

    /** The points making up this path. */
    private Vector[] points;

    /**
     * Represents a Hermite spline defined by two points and their corresponding tangents.
     */
    public static class Hermite {
        public Vector p1;
        public Vector p2;
        public Vector v1;
        public Vector v2;

        /**
         * Constructs a Hermite spline segment.
         *
         * @param p1 Start point of the spline
         * @param p2 End point of the spline
         * @param v1 Tangent at the start point
         * @param v2 Tangent at the end point
         */
        public Hermite(Vector p1, Vector p2, Vector v1, Vector v2) {
            this.p1 = p1;
            this.p2 = p2;
            this.v1 = v1;
            this.v2 = v2;
        }
    }

    /**
     * Gets the points in the path
     * @return Returns the points
     */
    public Vector[] points() {
        return points;
    }

    /**
     * Creates a path from a list of points.
     *
     * @param points Array of path points
     */
    public void create(Vector[] points) {
        this.points = points;
    }

    /**
     * Creates a path from a Hermite spline by converting it into a Bezier curve.
     *
     * @param h    Hermite spline
     * @param eps  Error tolerance for curve flattening
     */
    public void create(Hermite h, double eps) {
        Vector[] cPts = hermiteToBezier(h);
        create(cPts, eps);
    }

    /**
     * Creates a path from Bezier control points and flattens it into a series of line segments.
     *
     * @param cPts Four Bezier control points
     * @param eps  Error tolerance for flattening
     * @throws IllegalArgumentException if cPts does not contain exactly 4 elements
     */
    public void create(Vector[] cPts, double eps) {
        if (cPts.length != 4) {
            throw new IllegalArgumentException(
                "Bezier must have exactly 4 control points: " + cPts.length
            );
        }

        ArrayList<Vector> flatPts = flatten(cPts, new ArrayList<>(), eps);
        flatPts.add(0, cPts[0]);
        this.points = flatPts.toArray(new Vector[0]);
    }

    /**
     * Converts a Hermite spline into equivalent Bezier curve control points.
     *
     * @param h Hermite spline
     * @return Array of 4 Bezier control points
     * @throws IllegalArgumentException if start and end points are too close together
     */
    private Vector[] hermiteToBezier(Hermite h) {
        double d = h.p2.minus(h.p1).hypot();   // distance between p1 and p2
        double s1 = h.v1.hypot();              // magnitude of v1
        double s2 = h.v2.hypot();              // magnitude of v2

        if (d < EPS) {
            throw new IllegalArgumentException(
                "Start and end points are too close together. Provide points with greater separation."
            );
        }

        //esimate the travel time
        double vAve = 0.5 * (s1 + s2);
        double t = vAve < EPS ? 0 : d / vAve;

        //Estimate the required accleration to reach the target
        double aReq = (s2 * s2 - s1 * s1) / (2.0 * d); 

        //If estimated accelerated acceds max acceleration, travel time down
        if (Math.abs(aReq) > ACCEL) {
            double scale = Math.sqrt(Math.abs(aReq) / ACCEL);
            t *= scale;
        }

        //Reassemble Bezier curve
        Vector p0 = h.p1;
        Vector p1 = h.p1.plus(h.v1.times(t / 3.0));
        Vector p2 = h.p2.minus(h.v2.times(t / 3.0));
        Vector p3 = h.p2;

        return new Vector[] {p0, p1, p2, p3};
    }

    /**
     * Finds midpoints between each consecutive pair of points.
     *
     * @param pts Array of points
     * @return Array of midpoints
     */
    private Vector[] findMidPoints(Vector[] pts) {
        Vector[] mids = new Vector[pts.length - 1];

        //Go through the points and find each midpoint
        for (int i = 0; i < pts.length - 1; i++) {
            mids[i] = new Vector(
                (pts[i].x + pts[i + 1].x) / 2.0,
                (pts[i].y + pts[i + 1].y) / 2.0
            );
        }

        return mids;
    }

    /**
     * Recursively flattens a Bezier curve into line segments within the error tolerance.
     *
     * @param p   Bezier control points
     * @param fP  List accumulating resulting points
     * @param eps Maximum allowable error
     * @return List of points approximating the curve
     */
    private ArrayList<Vector> flatten(Vector[] p, ArrayList<Vector> fP, double eps) {
        Vector[] l = findMidPoints(p);
        Vector[] q = findMidPoints(l);
        Vector c = findMidPoints(q)[0];

        //calculate deviation of straight-line approximation from curve
        double error = Math.max(
            Math.hypot(2 * p[1].x - p[2].x - p[0].x,
                       2 * p[1].y - p[2].y - p[0].y),
            Math.hypot(2 * p[2].x - p[3].x - p[1].x,
                       2 * p[2].y - p[3].y - p[1].y)
        );

        //If deviation is low enough end, else keep going
        if (error < eps) {
            fP.add(p[3]);
        } else {
            fP = flatten(new Vector[] {p[0], l[0], q[0], c}, fP, eps);
            fP = flatten(new Vector[] {c, q[1], l[2], p[3]}, fP, eps);
        }

        return fP;
    }

    /**
     * Prints all points in the path to the console.
     */
    public void printPoints() {
        for (Vector p : points) {
            System.out.printf("Point(%.2f, %.2f)%n", p.x, p.y);
        }
    }

    /**
     * Draws the path on a graph starting from a given index.
     *
     * @param i     Index to begin drawing from
     * @param graph Graph object to draw onto
     */
    public void draw(int i, Graph graph) {
        ArrayList<Vector> subPath = new ArrayList<>(
            Arrays.asList(points).subList(i, points.length)
        );

        graph.createPath(
            subPath,
            "Planned Path",
            Constants.PATH_ALPHA,
            Constants.PATH_COLOR,
            Constants.PATH_WIDTH
        );
    }
}
