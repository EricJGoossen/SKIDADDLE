package Pathing;

import java.util.ArrayList;

import Util.Constants;
import Util.Vector;
import Util.Path.Arc;
import Util.Path.Path;

public final class PathGenerator {
    private static final double ACCEL = Constants.MAX_ACCELERATION;
    private static final double EPS = Constants.ZERO_TOLERANCE;

    public static Path populate(Vector[] points) {
        Arc[] arcs = new Arc[points.length - 1];

        for (int i = 0; i < points.length - 2; i++) {
            arcs[i] = new Arc(
                points[i],
                points[i + 1],
                points[i + 2],
                i
            );
        }

        arcs[arcs.length - 1] = new Arc(
            points[points.length - 2],
            points[points.length - 1],
            null,
            arcs.length - 1
        );

        Path path = new Path(arcs);
        LookAhead.vertexLimits(path);

        return path;
    }

    public static Path populate(Path.Bezier b) {
        ArrayList<Vector> flatPoints = new ArrayList<>();
        flatPoints.add(b.p0);

        ArrayList<Vector> flattened = flatten(
            new Vector[] {b.p0, b.p1, b.p2, b.p3},
            new ArrayList<>(),
            Constants.LINE_APPROX_EPSILON
        );

        flatPoints.addAll(flattened);
        return populate(flatPoints.toArray(new Vector[0]));
    }

    /**
     * Converts a Hermite spline into equivalent Bezier curve control points.
     * Uses estimated travel time and acceleration constraints to scale tangents.
     *
     * @param h Hermite spline
     * @return Bezier curve equivalent to the Hermite spline
     * @throws IllegalArgumentException if start and end points are too close together
     */
    public static Path populate(Path.Hermite h) {
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
        Path.Bezier bezier = new Path.Bezier(
                h.p1,
                h.p1.plus(h.v1.times(t / 3.0)),
                h.p2.minus(h.v2.times(t / 3.0)),
                h.p2
        );

        return populate(bezier);
    }

    /**
     * Finds midpoints between each consecutive pair of points.
     *
     * @param pts array of points
     * @return array of midpoints
     */
    private static Vector[] findMidPoints(Vector[] pts) {
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
    public static ArrayList<Vector> flatten(Vector[] p, ArrayList<Vector> fP, double eps) {
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
}
