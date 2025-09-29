package Util;

/**
 * Represents a 2D vector with common operations for geometry,
 * physics, and robotics applications.
 * <p>
 * Provides basic arithmetic operations, directional utilities,
 * and projection helpers for vector math.
 */
public class Vector {
    /** X-component of the vector */
    public double x;

    /** Y-component of the vector */
    public double y;

    // Constructors

    /**
     * Constructs a vector with given components.
     *
     * @param x x-component
     * @param y y-component
     */
    public Vector(double x, double y) {
        this.x = x;
        this.y = y;
    }

    /**
     * Copy constructor.
     *
     * @param other vector to copy
     */
    public Vector(Vector other) {
        this.x = other.x;
        this.y = other.y;
    }

    /**
     * Constructs a zero vector (0, 0).
     */
    public Vector() {
        this.x = 0;
        this.y = 0;
    }

    // Basic arithmetic operators

    /**
     * Adds this vector to another.
     *
     * @param v vector to add
     * @return a new vector representing the sum
     */
    public Vector plus(Vector v) {
        return new Vector(x + v.x, y + v.y);
    }

    /**
     * Subtracts another vector from this one.
     *
     * @param v vector to subtract
     * @return a new vector representing the difference
     */
    public Vector minus(Vector v) {
        return new Vector(x - v.x, y - v.y);
    }

    /**
     * Scales this vector by a scalar value.
     *
     * @param n scalar multiplier
     * @return a new scaled vector
     */
    public Vector times(double n) {
        return new Vector(x * n, y * n);
    }

    /**
     * Divides this vector by a scalar value.
     * Returns (0, 0) if the divisor is zero.
     *
     * @param n scalar divisor
     * @return a new divided vector
     */
    public Vector div(double n) {
        if (n == 0) return new Vector();
        return new Vector(x / n, y / n);
    }

    // Vector operations

    /**
     * Computes the dot product of this vector with another.
     *
     * @param v other vector
     * @return scalar dot product
     */
    public double dot(Vector v) {
        return x * v.x + y * v.y;
    }

    /**
     * Computes the 2D cross product (scalar value).
     *
     * @param v other vector
     * @return scalar cross product
     */
    public double cross(Vector v) {
        return x * v.y - y * v.x;
    }

    /**
     * Computes the magnitude (length) of this vector.
     *
     * @return vector magnitude
     */
    public double hypot() {
        return Math.sqrt(x * x + y * y);
    }

    /**
     * Returns the unit vector in the same direction as this vector.
     *
     * @return normalized vector (zero vector if magnitude is 0)
     */
    public Vector norm() {
        double h = hypot();
        return h == 0 ? new Vector(0, 0) : div(h);
    }

    // Directional operations

    /**
     * Computes the normalized direction vector pointing from this vector to another.
     *
     * @param p2 target point
     * @return normalized direction vector
     */
    public Vector dir(Vector p2) {
        return p2.minus(this).norm();
    }

    /**
     * Computes the angle of this vector relative to the Y-axis.
     *
     * @return angle in radians
     */
    public double angle() {
        return Math.atan2(x, y);
    }

    /**
     * Returns a perpendicular vector (rotated 90°).
     *
     * @return perpendicular vector
     */
    public Vector perp() {
        return new Vector(y, -x);
    }

    // Projection helpers

    /**
     * Projects this vector onto another.
     *
     * @param v vector to project onto
     * @return projection vector
     */
    public Vector project(Vector v) {
        double denom = v.dot(v);
        if (denom == 0) return new Vector(0, 0);
        return v.times(this.dot(v) / denom);
    }

    /**
     * Computes the scalar magnitude of this vector projected onto another.
     *
     * @param v vector to project onto
     * @return projection magnitude
     */
    public double projMag(Vector v) {
        if (v.hypot() < Constants.ZERO_TOLERANCE) return 0;
        return this.dot(v) / v.hypot();
    }

    /**
     * Computes the convergent component of this vector
     * relative to another (only if projection is positive).
     *
     * @param v reference vector
     * @return convergent projection vector
     */
    public Vector convergent(Vector v) {
        if (projMag(v) <= 0) return new Vector();
        return project(v);
    }

    /**
     * Computes the transverse component of this vector
     * relative to another (perpendicular remainder).
     *
     * @param v reference vector
     * @return transverse vector
     */
    public Vector transverse(Vector v) {
        return this.minus(convergent(v));
    }

    // Debug helpers

    /**
     * Returns a string representation of this vector.
     *
     * @return string in format "(x, y)"
     */
    @Override
    public String toString() {
        return String.format("(%.2f, %.2f)", x, y);
    }
}
