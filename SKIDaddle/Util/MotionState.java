package Util;

/**
 * Represents the full motion state of the robot.
 * Stores position, velocity, and acceleration in both linear and angular components,
 * as well as whether the robot is actively moving.
 */
public class MotionState {
    /** Position (linear + angular) */
    public Twist pos;

    /** Velocity (linear + angular) */
    public Twist vel;

    /** Acceleration (linear + angular) */
    public Twist accel;

    /** Whether the robot is actively moving */
    public boolean moving;

    /**
     * Constructs a motion state from separate linear and angular pose states.
     *
     * @param linear  linear pose, velocity, and acceleration
     * @param angular angular pose, velocity, and acceleration
     */
    public MotionState(PoseVelAcc linear, PoseVelAcc angular) {
        this.pos = new Twist(linear.pos, angular.pos);
        this.vel = new Twist(linear.vel, angular.vel);
        this.accel = new Twist(linear.accel, angular.accel);
        this.moving = linear.moving || angular.moving;
    }

    /**
     * Constructs a motion state directly from twist components.
     *
     * @param pos   position twist
     * @param vel   velocity twist
     * @param accel acceleration twist
     */
    public MotionState(Twist pos, Twist vel, Twist accel) {
        this.pos = pos;
        this.vel = vel;
        this.accel = accel;
        this.moving = false;
    }

    /**
     * Copy constructor.
     *
     * @param other the motion state to copy
     */
    public MotionState(MotionState other) {
        this.pos = other.pos;
        this.vel = other.vel;
        this.accel = other.accel;
        this.moving = other.moving;
    }

    /**
     * Constructs a motion state initialized to rest at the origin.
     */
    public MotionState() {
        this.pos = new Twist();
        this.vel = new Twist();
        this.accel = new Twist();
        this.moving = false;
    }

    /**
     * Extracts the linear portion of this motion state as a {@link PoseVelAcc}.
     *
     * @return linear pose, velocity, and acceleration
     */
    public PoseVelAcc linear() {
        return new PoseVelAcc(pos.lin, vel.lin, accel.lin);
    }

    /**
     * Extracts the angular portion of this motion state as a {@link PoseVelAcc}.
     *
     * @return angular pose, velocity, and acceleration
     */
    public PoseVelAcc angular() {
        return new PoseVelAcc(pos.ang, vel.ang, accel.ang);
    }
}
