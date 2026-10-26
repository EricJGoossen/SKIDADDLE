package Simulator;

import java.util.ArrayList;

import Simulator.Draw.Graph;
import Util.Constants;
import Util.MotionState;
import Util.MotorVector;
import Util.Twist;
import Util.AccelerationCommands.MotorCommand;

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
    private final double acceptableAccelError = 0.5;

    /** Iteration counter */
    private int iterations = 0;

    /** Graph drawer for visualization */
    private final Graph draw;

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
        MotorVector motorVel = pose.vel.toMotorSpace(pose.pos.ang);
        MotorVector motorAcc = pose.accel.toMotorSpace(pose.pos.ang);
        iterations++;

        Twist velError = pose.pos.minus(lastPose.pos).minus(pose.vel.times(deltaT));
        Twist accelError = pose.vel.minus(lastPose.vel).minus(pose.accel.times(deltaT));

        try {
            // --- Consistency checks ---
            if (velError.lin.hypot() > acceptableError) {
                throw new IllegalStateException(
                        "Linear position delta does not match velocity. i: " + iterations);
            }
            if (velError.ang > acceptableError) {
                throw new IllegalStateException(
                        "Angular position delta does not match angular velocity. i: " + iterations);
            }
            if (accelError.lin.hypot() > acceptableError) {
                throw new IllegalStateException(
                        "Linear velocity delta does not match acceleration. i: " + iterations);
            }
            if (accelError.ang > acceptableError) {
                throw new IllegalStateException(
                        "Angular velocity delta does not match angular acceleration. i: " + iterations);
            }

            // --- Speed limits ---
            if (Math.abs(motorVel.max()) > Constants.MAX_MOTOR_SPEED + acceptableError) {
                throw new IllegalStateException("Linear velocity exceeds max. i: " + iterations);
            }

            // --- Acceleration limits ---
            if (Math.abs(motorAcc.max()) > Constants.MAX_MOTOR_ACCELERATION + acceptableAccelError) {
                throw new IllegalStateException("Linear acceleration exceeds max. i: " + iterations);
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
            draw.moveRobot(pose.pos.lin.x, pose.pos.lin.y, pose.pos.ang);
        }
    }

    /**
     * @return copy of the pose history
     */
    public ArrayList<MotionState> getPoses() {
        return new ArrayList<>(poses);
    }
}
