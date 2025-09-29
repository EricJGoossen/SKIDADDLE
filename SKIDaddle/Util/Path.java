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
         * @param p1 start point of the spline
         * @param p2 end point of the spline
         * @param v1 tangent at the start point
         * @param v2 tangent at the end point
         */
        public Hermite(Vector p1, Vector p2, Vector v1, Vector v2) {
            this.p1 = p1;
            this.p2 = p2;
            this.v1 = v1;
            this.v2 = v2;
        }
    }

    /**
     * Represents a cubic Bezier curve defined by four control points.
     */
    public static class Bezier {
        public Vector p0;
        public Vector p1;
        public Vector p2;
        public Vector p3;

        /**
         * Constructs a cubic Bezier curve segment.
         *
         * @param p0 start point of the curve
         * @param p1 first control point
         * @param p2 second control point
         * @param p3 end point of the curve
         */
        public Bezier(Vector p0, Vector p1, Vector p2, Vector p3) {
            this.p0 = p0;
            this.p1 = p1;
            this.p2 = p2;
            this.p3 = p3;
        }

        /**
         * Constructs a cubic Bezier curve from an array of exactly four points.
         *
         * @param points array of four control points
         * @throws IllegalArgumentException if the array length is not 4
         */
        public Bezier(Vector[] points) {
            if (points.length != 4) {
                throw new IllegalArgumentException(
                    "Bezier must have exactly 4 control points: " + points.length
                );
            }

            p0 = points[0];
            p1 = points[1];
            p2 = points[2];
            p3 = points[3];
        }
    }

    /** Constructs an empty path. */
    public Path() {}

    /**
     * Creates a path from a list of points.
     *
     * @param points array of path points
     */
    public Path(Vector[] points) {
        this.points = points;
    }

    /**
     * Creates a path from a Bezier curve by flattening it into line segments.
     *
     * @param b cubic Bezier curve
     */
    public Path(Bezier b) {
        create(b);
    }

    /**
     * Creates a path from a Hermite spline by converting it into a Bezier curve.
     *
     * @param h Hermite spline
     */
    public Path(Hermite h) {
        create(hermiteToBezier(h));
    }

    /**
     * Converts a Bezier curve into an array of points by flattening.
     *
     * @param b cubic Bezier curve
     */
    private void create(Bezier b) {
        Vector[] a = {b.p0, b.p1, b.p2, b.p3};

        ArrayList<Vector> flatPts = flatten(a, new ArrayList<>(), Constants.LINE_APPROX_EPSILON);
        flatPts.add(0, b.p0);

        this.points = flatPts.toArray(new Vector[0]);
    }

    /**
     * Returns the points in the path.
     *
     * @return array of path points
     */
    public Vector[] points() {
        return points;
    }

    /**
     * Converts a Hermite spline into equivalent Bezier curve control points.
     * Uses estimated travel time and acceleration constraints to scale tangents.
     *
     * @param h Hermite spline
     * @return Bezier curve equivalent to the Hermite spline
     * @throws IllegalArgumentException if start and end points are too close together
     */
    private Bezier hermiteToBezier(Hermite h) {
        double d = h.p2.minus(h.p1).hypot();   // distance between p1 and p2
        double s1 = h.v1.hypot();              // magnitude of v1
        double s2 = h.v2.hypot();              // magnitude of v2

        if (d < EPS) {
            throw new IllegalArgumentException(
                "Start and end points are too close together. Provide points with greater separation."
            );
        }

        // Estimate travel time based on average speed
        double vAve = 0.5 * (s1 + s2);
        double t = vAve < EPS ? 0 : d / vAve;

        // Estimate the required acceleration to transition from v1 to v2
        double aReq = (s2 * s2 - s1 * s1) / (2.0 * d);

        // If acceleration exceeds limits, scale travel time
        if (Math.abs(aReq) > ACCEL) {
            double scale = Math.sqrt(Math.abs(aReq) / ACCEL);
            t *= scale;
        }

        // Construct equivalent Bezier curve
        return new Bezier(
            h.p1,
            h.p1.plus(h.v1.times(t / 3.0)),
            h.p2.minus(h.v2.times(t / 3.0)),
            h.p2
        );
    }

    /**
     * Finds midpoints between each consecutive pair of points.
     *
     * @param pts array of points
     * @return array of midpoints
     */
    private Vector[] findMidPoints(Vector[] pts) {
        Vector[] mids = new Vector[pts.length - 1];
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
     * @param fP  list accumulating resulting points
     * @param eps maximum allowable error
     * @return list of points approximating the curve
     */
    private ArrayList<Vector> flatten(Vector[] p, ArrayList<Vector> fP, double eps) {
        Vector[] l = findMidPoints(p);
        Vector[] q = findMidPoints(l);
        Vector c = findMidPoints(q)[0];

        // Calculate deviation of straight-line approximation from curve
        double error = Math.max(
            Math.hypot(2 * p[1].x - p[2].x - p[0].x,
                       2 * p[1].y - p[2].y - p[0].y),
            Math.hypot(2 * p[2].x - p[3].x - p[1].x,
                       2 * p[2].y - p[3].y - p[1].y)
        );

        // If deviation is acceptable, add endpoint; otherwise subdivide recursively
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
     * @param i     index to begin drawing from
     * @param graph graph object to draw onto
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
