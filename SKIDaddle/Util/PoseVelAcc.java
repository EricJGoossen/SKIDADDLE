package Util;

/**
 * Represents a combined state of position, velocity, and acceleration,
 * all stored as vectors in 2D space. Includes a flag to indicate if
 * the object is currently moving.
 */
public class PoseVelAcc {
    /** Position vector */
    public Vector pos;

    /** Velocity vector */
    public Vector vel;

    /** Acceleration vector */
    public Vector accel;

    /** Whether the object is actively moving */
    public boolean moving;

    /**
     * Constructs a new pose with explicit position, velocity, and acceleration.
     * Marks this state as moving.
     *
     * @param pos   position vector
     * @param vel   velocity vector
     * @param accel acceleration vector
     */
    public PoseVelAcc(Vector pos, Vector vel, Vector accel) {
        this.pos = pos;
        this.vel = vel;
        this.accel = accel;
        this.moving = true;
    }

    /**
     * Copy constructor.
     * Creates a deep copy of another pose-velocity-acceleration state.
     * Marks this state as moving.
     *
     * @param other the state to copy
     */
    public PoseVelAcc(PoseVelAcc other) {
        this.pos = new Vector(other.pos);
        this.vel = new Vector(other.vel);
        this.accel = new Vector(other.accel);
        this.moving = true;
    }

    /**
     * Constructs a pose at the origin, with zero velocity and acceleration.
     * Marks this state as not moving.
     */
    public PoseVelAcc() {
        this.pos = new Vector(0, 0);
        this.vel = new Vector(0, 0);
        this.accel = new Vector(0, 0);
        this.moving = false;
    }
}
