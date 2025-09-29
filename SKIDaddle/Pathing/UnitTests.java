package Pathing;

import Simulator.PathCreater;
import Simulator.Draw.Graph;
import Util.Constants;
import Util.Path;
import Util.PoseVelAcc;
import Util.Vector;
import Util.MotionState;

/**
 * Contains static unit test methods for path generation and traversal.
 * Provides functionality for testing line paths, splines, Hermite-based
 * paths, and interactive path creation in simulation.
 */
public class UnitTests {

    /**
     * Updates the graph and field displays if simulation is enabled.
     *
     * @param draw Graph instance to update
     */
    private static void updateLog(Graph draw) {
        if (Constants.SIMULATING) {
            draw.updateGraph();
            draw.updateField();
        }
    }

    /**
     * Runs a traversal test on a path using default initial pose.
     *
     * @param path   Path to follow
     * @param deltaT Time step for simulation
     * @param sleep  Delay between updates (seconds)
     * @param draw   Graph instance for visualization
     * @param move   Whether the robot should move
     */
    private static void pathTest(Path path, double deltaT, double sleep, Graph draw, boolean move) {
        PoseVelAcc lin = new PoseVelAcc(path.points()[0], new Vector(), new Vector());
        PoseVelAcc ang = new PoseVelAcc(new Vector(0, 1), new Vector(), new Vector());
        MotionState pose = new MotionState(lin, ang);

        traverse(path, pose, deltaT, sleep, draw, move);
    }

    /**
     * Traverses a path starting from a given motion state.
     *
     * @param path   Path to follow
     * @param pose   Initial motion state
     * @param deltaT Time step for simulation
     * @param sleep  Delay between updates (seconds)
     * @param draw   Graph instance for visualization
     * @param move   Whether the robot should move
     */
    private static void traverse(Path path, MotionState pose, double deltaT, double sleep, Graph draw, boolean move) {
        pose.moving = true;

        Angular.Controller angController = Angular.turnToAngle(Math.toRadians(-135));
        Controller driveTo = new Controller(draw, null);
        driveTo.setInitState(pose);

        if (Constants.SIMULATING) draw.setShowRobot(true);

        while (pose.moving && move) {
            pose = driveTo.update(pose, new MotionState(), path, angController, deltaT);
            updateLog(draw);

            try {
                Thread.sleep((long) (sleep * 1000));
            } catch (InterruptedException e) {
                // Ignore interruptions
            }
        }

        System.out.println("done");
    }

    /**
     * Generates and tests a Bezier path from given control points.
     *
     * @param points Bezier control points
     * @param deltaT Time step for simulation
     */
    private static void pathGenTest(Vector[] points, double deltaT) {
        Graph draw = new Graph();
        if (Constants.SIMULATING) draw.setShowRobot(false);

        Path path = new Path(new Path.Bezier(points));
        path.draw(0, draw);
        path.printPoints();
    }

    /**
     * Tests traversal of a line path.
     *
     * @param points Points of the path
     * @param move   Whether the robot should move
     * @param deltaT Time step for simulation
     */
    public static void linePathTest(Vector[] points, boolean move, double deltaT) {
        Graph draw = new Graph();
        Path path = new Path(points);
        pathTest(path, deltaT, deltaT, draw, move);
    }

    /**
     * Tests traversal of a Bezier spline path.
     *
     * @param points Bezier control points
     * @param move   Whether the robot should move
     * @param deltaT Time step for simulation
     */
    public static void splinePathTest(Vector[] points, boolean move, double deltaT) {
        Graph draw = new Graph();
        Path path = new Path(new Path.Bezier(points));
        pathTest(path, deltaT, deltaT, draw, move);
    }

    /**
     * Tests traversal of a Hermite path with specified initial and final velocities.
     *
     * @param p1     Start point
     * @param v1     Initial velocity vector
     * @param p2     End point
     * @param v2     Final velocity vector
     * @param move   Whether the robot should move
     * @param deltaT Time step for simulation
     */
    public static void initialVelTest(Vector p1, Vector v1, Vector p2, Vector v2, boolean move, double deltaT) {
        Graph draw = new Graph();
        Path path = new Path(new Path.Hermite(p1, p2, v1, v2));

        PoseVelAcc lin = new PoseVelAcc(p1, v1, new Vector());
        PoseVelAcc ang = new PoseVelAcc(new Vector(0, 1), new Vector(), new Vector());
        MotionState pose = new MotionState(lin, ang);

        traverse(path, pose, deltaT, deltaT, draw, move);
    }

    /**
     * Interactive line path creation test.
     *
     * @param move   Whether the robot should move
     * @param deltaT Time step for simulation
     */
    public static void linePathCreater(boolean move, double deltaT) {
        Graph draw = new Graph();
        if (Constants.SIMULATING) draw.setShowRobot(false);

        while (true) {
            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
                // Ignore interruptions
            }

            Path path = PathCreater.linePath(draw);
            updateLog(draw);

            if (path.points() == null || path.points().length < 2) continue;
            pathTest(path, deltaT, 0, draw, move);
        }
    }

    /**
     * Interactive Bezier spline path creation test.
     *
     * @param move   Whether the robot should move
     * @param deltaT Time step for simulation
     */
    public static void splinePathCreater(boolean move, double deltaT) {
        Graph draw = new Graph();
        if (Constants.SIMULATING) draw.setShowRobot(false);

        while (true) {
            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
                // Ignore interruptions
            }

            Path path = PathCreater.splinePath(draw);
            updateLog(draw);

            if (path.points() == null || path.points().length < 2) continue;
            pathTest(path, deltaT, 0, draw, move);
        }
    }
}
