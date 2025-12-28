package Util;

import Pathing.SpaceKinematics;
import Util.AccelerationCommands.MotorCommand;

/**
 * Represents a 2D twist consisting of linear and angular components.
 * <p>
 * The linear component ({@link #lin}) is typically used for translation,
 * while the angular component ({@link #ang}) is used for rotation.
 * This class provides basic arithmetic operations for combining and
 * manipulating twists.
 */
public class Twist {
    /** Linear component of the twist */
    public Vector lin;

    /** Angular component of the twist */
    public double ang;

    /**
     * Constructs a twist from explicit linear and angular vectors.
     *
     * @param lin linear vector component
     * @param ang angular vector component
     */
    public Twist(Vector lin, double ang) {
        this.lin = lin;
        this.ang = ang;
    }

    /**
     * Copy constructor.
     *
     * @param other the twist to copy
     */
    public Twist(Twist other) {
        this.lin = other.lin;
        this.ang = other.ang;
    }

    /**
     * Constructs a twist with zero linear and angular vectors.
     */
    public Twist() {
        this.lin = new Vector();
        this.ang = 0;
    }

    // ------------------------------------------------------------
    // Basic arithmetic operations
    // ------------------------------------------------------------

    /**
     * Adds this twist to another.
     *
     * @param t the other twist
     * @return a new twist representing the sum
     */
    public Twist plus(Twist t) {
        return new Twist(lin.plus(t.lin), ang + t.ang);
    }

    /**
     * Subtracts another twist from this twist.
     *
     * @param t the other twist
     * @return a new twist representing the difference
     */
    public Twist minus(Twist t) {
        return new Twist(lin.minus(t.lin), ang - t.ang);
    }

    /**
     * Scales this twist by a scalar value.
     *
     * @param n scalar multiplier
     * @return a new twist scaled by {@code n}
     */
    public Twist times(double n) {
        return new Twist(lin.times(n), ang * n);
    }

    /**
     * Scales this twist by a dual (separately for linear and angular parts).
     *
     * @param d dual multiplier
     * @return a new twist scaled by {@code d}
     */
    public Twist times(Dual d) {
        return new Twist(lin.times(d.linear), ang * d.angular);
    }

    /**
     * Divides this twist by a scalar value.
     *
     * @param n scalar divisor
     * @return a new twist divided by {@code n}
     */
    public Twist div(double n) {
        return new Twist(lin.div(n), ang / n);
    }

    /**
     * Divides this twist by a dual (separately for linear and angular parts).
     *
     * @param d dual divisor
     * @return a new twist divided by {@code d}
     */
    public Twist div(Dual d) {
        return new Twist(lin.div(d.linear), ang / d.angular);
    }

    public double sum() {
        return lin.hypot() + ang;
    } 

    public MotorCommand toMotorCommand(double angle) {
        return SpaceKinematics.inverse(this, angle);
    }
}
