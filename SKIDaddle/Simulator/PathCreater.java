package Simulator;

import Util.Constants;
import Util.Path;
import Util.Vector;

import java.util.ArrayList;

import Simulator.Draw.Graph;

/**
 * Provides interactive path creation utilities for simulation.
 * Supports both straight-line paths and cubic Bezier spline paths,
 * with the ability to add, move, and remove points through user input.
 */
public class PathCreater {

    /** Index of the point currently being changed. */
    private static int changeIndex = 0;

    /** Whether a point is currently being modified. */
    private static boolean changing = false;

    /** Current number of points added for Bezier path creation. */
    private static int currIndex = 0;

    /** Currently highlighted point in the editor, if any. */
    private static Vector highlightedPoint = null;

    /** List of points used for both line and spline path creation. */
    private static final ArrayList<Vector> linePoints = new ArrayList<>();

    /**
     * Creates or modifies a cubic Bezier spline interactively.
     * User actions are processed via mouse clicks and key presses.
     *
     * Controls:
     * - Click: add new control points (up to 4).
     * - Click near an existing point: select it for editing.
     * - Backspace: remove highlighted point.
     * - R: reset all points and clear the editor.
     *
     * @param draw Graph instance for user interaction
     * @return Generated path, or empty if not enough points
     */
    public static Path splinePath(Graph draw) {
        if (!Constants.SIMULATING || draw == null) {
            throw new IllegalStateException(
                "Must be simulating to use path creater"
                );
        }

        Path path = new Path();

        Vector point = draw.clicked();

        if (draw.rPressed()) {
            linePoints.clear();
            draw.clearPoints();
            path = new Path(new Path.Bezier(linePoints.toArray(new Vector[0])));
            return path;
        }

        if (draw.backspacePressed() && highlightedPoint != null) {
            draw.removePoint(highlightedPoint);
            linePoints.remove(highlightedPoint);
            highlightedPoint = null;
            changing = false;
            path = new Path(new Path.Bezier(linePoints.toArray(new Vector[0])));
            return path;
        }

        if (point == null) return path;

        int idx = inside(linePoints, point);
        if (idx != -1) {
            changeIndex = idx;
            changing = true;
            draw.highlightPoint(linePoints.get(changeIndex));
            highlightedPoint = linePoints.get(changeIndex);
            return path;
        }

        if (changing) {
            draw.clearHighlight();
            draw.removePoint(linePoints.get(changeIndex));
            linePoints.set(changeIndex, point);
            draw.drawPoint(point);
            changing = false;
        } else if (currIndex < 4) {
            linePoints.add(point);
            draw.drawPoint(point);
            currIndex++;
        }

        if (linePoints.size() == 4) {
            path = new Path(new Path.Bezier(linePoints.toArray(new Vector[4])));
        }

        return path;
    }

    /**
     * Creates or modifies a straight-line path interactively.
     * User actions are processed via mouse clicks and key presses.
     *
     * Controls:
     * - Click: add new path points.
     * - Click near an existing point: select it for editing.
     * - Backspace: remove highlighted point.
     * - R: reset all points and clear the editor.
     *
     * @param draw Graph instance for user interaction
     * @return Generated path, or empty if not enough points
     */
    public static Path linePath(Graph draw) {
        if (!Constants.SIMULATING || draw == null) {
            throw new IllegalStateException(
                "Must be simulating to use path creater"
                );
        }
        
        Path path = new Path();

        Vector point = draw.clicked();

        if (draw.rPressed()) {
            linePoints.clear();
            draw.clearPoints();
            draw.clearSplines();
            path = new Path(linePoints.toArray(new Vector[0]));
            return path;
        }

        if (draw.backspacePressed() && highlightedPoint != null) {
            draw.removePoint(highlightedPoint);
            linePoints.remove(highlightedPoint);
            highlightedPoint = null;
            changing = false;
            path = new Path(linePoints.toArray(new Vector[0]));
            return path;
        }

        if (point == null) return path;

        int idx = inside(linePoints, point);
        if (idx != -1) {
            changeIndex = idx;
            changing = true;
            draw.highlightPoint(linePoints.get(changeIndex));
            highlightedPoint = linePoints.get(changeIndex);
            return path;
        }

        if (changing) {
            draw.clearHighlight();
            draw.removePoint(linePoints.get(changeIndex));
            linePoints.set(changeIndex, point);
            draw.drawPoint(point);
            changing = false;
        } else {
            linePoints.add(point);
            draw.drawPoint(point);
        }

        path = new Path(linePoints.toArray(new Vector[0]));
        return path;
    }

    /**
     * Checks if two points are within {@link Constants#POINT_EPSILON}.
     *
     * @param p1 First point
     * @param p2 Second point
     * @return True if the points are considered the same
     */
    private static boolean samePoint(Vector p1, Vector p2) {
        return Math.abs(p1.x - p2.x) < Constants.POINT_EPSILON
            && Math.abs(p1.y - p2.y) < Constants.POINT_EPSILON;
    }

    /**
     * Finds whether a given point is already inside a list of points.
     *
     * @param points List of points
     * @param point  Point to check
     * @return Index of the point if found, -1 otherwise
     */
    private static int inside(ArrayList<Vector> points, Vector point) {
        for (int i = 0; i < points.size(); i++) {
            if (points.get(i) == null) continue;
            if (samePoint(points.get(i), point)) return i;
        }
        return -1;
    }
}
