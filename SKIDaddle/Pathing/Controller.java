package Pathing;

import Pathing.PIDs.FeedForward;
import Pathing.PIDs.PIDF;
import Simulator.Simulator;
import Simulator.Draw.Graph;
import Util.Constants;
import Util.PoseVelAcc;
import Util.MotionState;
import Util.Vector;
import Util.Path;
import Util.Twist;

import java.awt.Color;

/**
 * High-level robot controller combining linear and angular path following.
 * Applies PIDF feedback and feedforward to compute motor commands.
 * Supports simulation visualization and real-world control.
 */
public class Controller {
    private final PIDF pidf;              // PIDF controller for velocity/position correction
    private final FeedForward ff;         // Feedforward for wheel motion
    private final Graph draw;             // Visualization/logging
    protected final Simulator sim;        // Simulation engine

    private MotorController motor;        // Interface to send wheel velocities

    private Path lastPath;                // Tracks last path for dynamic updates
    private Angular.Controller lastAngle; // Tracks last angular controller

    private Angular angularController;    // Angular motion handler
    private Linear linearController;      // Linear path follower

    /**
     * Functional interface for motor commands.
     * Allows applying wheel velocities from a Kinematics.WheelSpeed object.
     */
    public interface MotorController {
        void update(Kinematics.WheelSpeed speeds);
    }

    /**
     * Constructs a high-level controller.
     *
     * @param draw  Graph object for visualization/logging
     * @param motor MotorController interface
     */
    public Controller(Graph draw, MotorController motor) {
        this.draw = draw;
        this.motor = motor;

        pidf = new PIDF(Constants.PID_P, Constants.PID_I, Constants.PID_D, Constants.PID_FV, Constants.PID_FA);
        ff = new FeedForward(Constants.PID_S, Constants.PID_V, Constants.PID_A);
        sim = new Simulator(Constants.INIT_STATE, draw);

        initLog();
    }

    /**
     * Initializes simulation charts and paths for visualization.
     */
    private void initLog() {
        if (!Constants.SIMULATING) return;

        draw.createPath("Path traveled", 1f, Color.RED, 6f);

        draw.createChart("pos", 1, Color.YELLOW, 3);
        draw.createChart("vel", 1, Color.BLUE, 3);
        draw.createChart("accel", 1, Color.RED, 3);
        draw.createChart("max", 1, Color.GREEN, 3);
        draw.createChart("line", 1, Color.MAGENTA, 3);

        draw.createChart("angle", 1, Color.YELLOW, 3);
        draw.createChart("angleVel", 1, Color.BLUE, 3);
        draw.createChart("angleAccel", 1, Color.RED, 3);
    }

    /**
     * Sets the robot's initial motion state.
     *
     * @param m Initial MotionState
     */
    public void setInitState(MotionState m) {
        sim.reset(m);
    }

    /**
     * Updates the robot's state along a path and applies angular control.
     *
     * @param motion        Current robot state
     * @param sensorMotion  Sensor-based robot state
     * @param path          Path to follow
     * @param angle         Angular controller
     * @param deltaT        Time step
     * @return Updated MotionState
     */
    public MotionState update(MotionState motion, MotionState sensorMotion, Path path, Angular.Controller angle, double deltaT) {
        // Recreate controllers if path or angle changes
        if (lastPath != path) {
            linearController = new Linear(motion.vel.lin.hypot(), path, draw);
        }
        lastPath = path;

        if (lastAngle != angle) {
            angularController = new Angular(angle, draw);
        }
        lastAngle = angle;

        // Compute linear and angular updates
        PoseVelAcc linPose = linearController.followPath(motion.linear(), deltaT);
        PoseVelAcc angPose = angularController.update(motion, deltaT);

        MotionState updatedState = new MotionState(linPose, angPose);

        // Compute motor velocities and apply
        setDriveVel(new MotionState(updatedState), sensorMotion, deltaT);

        return updatedState;
    }

    /**
     * Converts field-relative velocities and accelerations into robot-relative vectors.
     *
     * @param m   MotionState in field coordinates
     * @param ang Robot's forward direction
     * @return MotionState in robot coordinates
     */
    private MotionState robotRelative(MotionState m, Vector ang) {
        Vector forward = ang.norm();
        Vector right = forward.perp();

        Twist vel = new Twist(m.vel.dot(right), m.vel.dot(forward));
        Twist accel = new Twist(m.accel.dot(right), m.accel.dot(forward));

        return new MotionState(m.pos, vel, accel);
    }

    /**
     * Computes and applies wheel velocities based on target trajectory.
     * Adds PIDF correction, converts to robot-relative frame, and applies feedforward.
     *
     * @param target       Desired MotionState
     * @param sensorMotion Sensor-derived MotionState
     * @param deltaT       Time step
     */
    public void setDriveVel(MotionState target, MotionState sensorMotion, double deltaT) {
        sim.update(target, deltaT);

        if (Constants.SIMULATING) return;

        // Add PIDF correction
        target.vel = target.vel.plus(pidf.compute(target, sensorMotion, deltaT));

        // Convert to robot-relative coordinates
        MotionState robotRel = robotRelative(target, sensorMotion.pos.ang);

        // Compute wheel speeds from inverse kinematics
        Kinematics.WheelSpeed finalVels = Kinematics.inverse(robotRel);

        // Apply feedforward
        finalVels = finalVels.feedForward(ff);

        // Send velocities to motors
        motor.update(finalVels);
    }
}
