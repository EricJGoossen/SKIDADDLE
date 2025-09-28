package Util;

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
    public Vector ang;

    /**
     * Constructs a twist from explicit linear and angular vectors.
     *
     * @param lin linear vector component
     * @param ang angular vector component
     */
    public Twist(Vector lin, Vector ang) {
        this.lin = lin;
        this.ang = ang;
    }

    /**
     * Constructs a twist from dual values.
     * The linear components form a vector from (x.linear, y.linear),
     * while the angular components form a vector from (x.angular, y.angular).
     *
     * @param x dual containing x-axis values
     * @param y dual containing y-axis values
     */
    public Twist(Dual x, Dual y) {
        this.lin = new Vector(x.linear, y.linear);
        this.ang = new Vector(x.angular, y.angular);
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
        this.ang = new Vector();
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
        return new Twist(lin.plus(t.lin), ang.plus(t.ang));
    }

    /**
     * Subtracts another twist from this twist.
     *
     * @param t the other twist
     * @return a new twist representing the difference
     */
    public Twist minus(Twist t) {
        return new Twist(lin.minus(t.lin), ang.minus(t.ang));
    }

    /**
     * Scales this twist by a scalar value.
     *
     * @param n scalar multiplier
     * @return a new twist scaled by {@code n}
     */
    public Twist times(double n) {
        return new Twist(lin.times(n), ang.times(n));
    }

    /**
     * Scales this twist by a dual (separately for linear and angular parts).
     *
     * @param d dual multiplier
     * @return a new twist scaled by {@code d}
     */
    public Twist times(Dual d) {
        return new Twist(lin.times(d.linear), ang.times(d.angular));
    }

    /**
     * Divides this twist by a scalar value.
     *
     * @param n scalar divisor
     * @return a new twist divided by {@code n}
     */
    public Twist div(double n) {
        return new Twist(lin.div(n), ang.div(n));
    }

    /**
     * Divides this twist by a dual (separately for linear and angular parts).
     *
     * @param d dual divisor
     * @return a new twist divided by {@code d}
     */
    public Twist div(Dual d) {
        return new Twist(lin.div(d.linear), ang.div(d.angular));
    }

    /**
     * Computes the dot product of this twist with a vector,
     * producing a dual (linear dot and angular dot).
     *
     * @param v the vector to dot with
     * @return a dual containing (linear dot, angular dot)
     */
    public Dual dot(Vector v) {
        return new Dual(lin.dot(v), ang.dot(v));
    }
}
