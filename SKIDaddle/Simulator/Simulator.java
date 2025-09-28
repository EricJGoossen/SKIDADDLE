package Simulator;

import java.util.ArrayList;

import Simulator.Draw.Graph;
import Util.Constants;
import Util.MotionState;
import Util.Path;
import Util.Vector;
import Util.Twist;

/**
 * Simulator for robot motion.
 * <p>
 * Handles pose updates, constraint checking, visualization, and interactive
 * path creation (via {@link Graph}).
 */
public class Simulator {
    /** Last known robot pose */
    private MotionState lastPose;

    /** History of robot poses */
    private final ArrayList<MotionState> poses = new ArrayList<>();

    /** Tolerance for numerical consistency checks */
    private final double acceptableError = 0.1;

    /** Iteration counter */
    private int iterations = 0;

    /** Path editing state */
    private int changeIndex = 0;
    private boolean changing = false;
    private int currIndex = 0;
    private Vector highlightedPoint = null;

    /** Path points for interactive path building */
    private final ArrayList<Vector> linePoints = new ArrayList<>();

    /** Graph drawer for visualization */
    private final Graph draw;

    /** Tolerance for detecting the "same" point */
    private final double pointEpsilon = 1.5;

    // Constructors

    /**
     * Constructs a simulator with an initial pose.
     *
     * @param initialPose starting robot pose
     * @param draw        visualization graph
     */
    public Simulator(MotionState initialPose, Graph draw) {
        this.draw = draw;
        this.lastPose = initialPose;
        this.iterations = 0;
    }

    /**
     * Constructs a simulator without an initial pose.
     *
     * @param draw visualization graph
     */
    public Simulator(Graph draw) {
        this.draw = draw;
    }

    // Core simulation

    /**
     * Resets the simulator to a given initial pose.
     *
     * @param initialPose new starting pose
     */
    public void reset(MotionState initialPose) {
        this.lastPose = initialPose;
        this.poses.clear();
        this.iterations = 0;
    }

    /**
     * Updates the simulator with a new pose, verifying consistency against
     * velocity and acceleration, and updating the drawing.
     *
     * @param pose   new pose
     * @param deltaT timestep in seconds
     */
    public void update(MotionState pose, double deltaT) {
        pose = new MotionState(pose); // Copy to avoid mutating external state
        iterations++;

        Twist velError = pose.pos.minus(lastPose.pos).minus(pose.vel.times(deltaT));
        Twist accelError = pose.vel.minus(lastPose.vel).minus(pose.accel.times(deltaT));

        try {
            // --- Consistency checks ---
            if (velError.lin.hypot() > acceptableError) {
                throw new IllegalStateException(
                        "Linear position delta does not match velocity. i: " + iterations);
            }
            if (velError.ang.hypot() > acceptableError) {
                throw new IllegalStateException(
                        "Angular position delta does not match angular velocity. i: " + iterations);
            }
            if (accelError.lin.hypot() > acceptableError) {
                throw new IllegalStateException(
                        "Linear velocity delta does not match acceleration. i: " + iterations);
            }
            if (accelError.ang.hypot() > acceptableError) {
                throw new IllegalStateException(
                        "Angular velocity delta does not match angular acceleration. i: " + iterations);
            }

            // --- Speed limits ---
            if (pose.vel.lin.hypot() > Constants.MAX_SPEED + acceptableError) {
                throw new IllegalStateException("Linear velocity exceeds max. i: " + iterations);
            }
            if (pose.vel.ang.hypot() > Constants.MAX_ANGULAR_VELOCITY + acceptableError) {
                throw new IllegalStateException("Angular velocity exceeds max. i: " + iterations);
            }

            // --- Acceleration limits ---
            if (pose.accel.lin.hypot() > Constants.MAX_ACCELERATION + acceptableError) {
                throw new IllegalStateException("Linear acceleration exceeds max. i: " + iterations);
            }
            if (pose.accel.ang.hypot() > Constants.MAX_ANGULAR_ACCELERATION + acceptableError) {
                throw new IllegalStateException("Angular acceleration exceeds max. i: " + iterations);
            }
        } catch (IllegalStateException e) {
            e.printStackTrace();
            // Can rethrow if strict enforcement is needed
        }

        // Clamp position to field boundaries
        pose.pos.lin.x = Math.max(-72, Math.min(pose.pos.lin.x, 72));
        pose.pos.lin.y = Math.max(-72, Math.min(pose.pos.lin.y, 72));

        lastPose = pose;
        poses.add(pose);

        // Draw robot if in simulation mode
        if (Constants.SIMULATING) {
            draw.moveRobot(pose.pos.lin.x, pose.pos.lin.y, pose.pos.ang.angle());
        }
    }

    // Helpers

    /** @return true if two points are within {@link #pointEpsilon} */
    private boolean samePoint(Vector p1, Vector p2) {
        return Math.abs(p1.x - p2.x) < pointEpsilon && Math.abs(p1.y - p2.y) < pointEpsilon;
    }

    /** @return index of a point in list, or -1 if not present */
    private int inside(ArrayList<Vector> points, Vector point) {
        for (int i = 0; i < points.size(); i++) {
            if (points.get(i) == null) continue;
            if (samePoint(points.get(i), point)) return i;
        }
        return -1;
    }

    // Path creation (interactive with Graph)

    /**
     * Interactive spline path creation.
     *
     * @param path path object to modify
     */
    public void getSplinePath(Path path) {
        if (!Constants.SIMULATING) return;

        Vector point = draw.clicked();

        if (draw.rPressed()) {
            linePoints.clear();
            draw.clearPoints();
            path.create(linePoints.toArray(new Vector[0]));
            return;
        }

        if (draw.backspacePressed() && highlightedPoint != null) {
            draw.removePoint(highlightedPoint);
            linePoints.remove(highlightedPoint);
            highlightedPoint = null;
            changing = false;
            path.create(linePoints.toArray(new Vector[0]));
            return;
        }

        if (point == null) return;

        int idx = inside(linePoints, point);
        if (idx != -1) {
            changeIndex = idx;
            changing = true;
            draw.highlightPoint(linePoints.get(changeIndex));
            highlightedPoint = linePoints.get(changeIndex);
            return;
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
            path.create(linePoints.toArray(new Vector[4]), Constants.LINE_APPROX_EPSILON);
        }
    }

    /**
     * Interactive straight-line path creation.
     *
     * @param path path object to modify
     */
    public void getLinePath(Path path) {
        if (!Constants.SIMULATING) return;

        Vector point = draw.clicked();

        if (draw.rPressed()) {
            linePoints.clear();
            draw.clearPoints();
            path.create(linePoints.toArray(new Vector[0]));
            return;
        }

        if (draw.backspacePressed() && highlightedPoint != null) {
            draw.removePoint(highlightedPoint);
            linePoints.remove(highlightedPoint);
            highlightedPoint = null;
            changing = false;
            path.create(linePoints.toArray(new Vector[0]));
            return;
        }

        if (point == null) return;

        int idx = inside(linePoints, point);
        if (idx != -1) {
            changeIndex = idx;
            changing = true;
            draw.highlightPoint(linePoints.get(changeIndex));
            highlightedPoint = linePoints.get(changeIndex);
            return;
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

        path.create(linePoints.toArray(new Vector[0]));
    }

    // Accessors

    /**
     * @return copy of the pose history
     */
    public ArrayList<MotionState> getPoses() {
        return new ArrayList<>(poses);
    }
}
