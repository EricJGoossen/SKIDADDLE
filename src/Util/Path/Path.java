package Util.Path;

import java.util.ArrayList;
import java.util.Arrays;

import Simulator.Draw.Graph;
import Util.Constants;
import Util.Vector;

public class Path {
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
    
    private Arc[] arcs;

    public Path(Arc[] arcs) {
        this.arcs = arcs;
    }

    public Arc arc(int idx) {
        return arcs[idx];
    }

    public int size() {
        return arcs.length;
    }

    /**
     * Draws the path on a graph starting from a given index.
     *
     * @param i     index to begin drawing from
     * @param graph graph object to draw onto
     */
    public void draw(int i, Graph graph, Vector v) {
        ArrayList<Vector> subPath = new ArrayList<>(arcs.length - i + 1);

        for (int j = i; j < arcs.length; j++) {
            subPath.add(arcs[j].point);
        }

        subPath.add(0, v);

        graph.createPath(
            subPath,
            "Planned Path",
            Constants.PATH_ALPHA,
            Constants.PATH_COLOR,
            Constants.PATH_WIDTH
        );
    }

    @Override
    public String toString() {
        return "Path(arcs=" + Arrays.toString(arcs) + ")";
    }
}
