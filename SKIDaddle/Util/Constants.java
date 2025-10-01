package Util;

import java.awt.Color;

/**
 * Centralized constants for robot motion, control, drawing, and simulation.
 * All values are global and immutable.
 */
public class Constants {
    // General
    public static final double ZERO_TOLERANCE = 1e-6; // floating-point tolerance
    public static final boolean SIMULATING = false;    // toggle for simulator mode

    // Linear motion constraints
    public static final double MAX_ACCELERATION = 50.0; // in/s^2
    public static final double MAX_SPEED = 50.0;        // in/s
    public static final double DISTANCE_EPSILON = 5.0;  // acceptable positional error (in)

    // Angular motion constraints
    public static final double MAX_ANGULAR_ACCELERATION = Math.PI; // rad/s^2
    public static final double MAX_ANGULAR_VELOCITY = Math.PI;     // rad/s
    public static final double ANGLE_EPSILON = Math.toRadians(5);  // acceptable angular error (rad)

    // Path generation
    public static final double LINE_APPROX_EPSILON = 0.01; // threshold when a curve is approximated as a line
    public static final double LOOK_AHEAD_FACTOR = 3.0;   // scale factor for look-ahead distance
    public static final MotionState INIT_STATE = new MotionState(); // default starting state

    // Path drawing
    public static final float PATH_ALPHA = 1f;
    public static final Color PATH_COLOR = Color.BLACK;
    public static final float PATH_WIDTH = 4f;

    // Field rendering
    public static final int FIELD_WIDTH = 600;
    public static final int FIELD_HEIGHT = 600;
    public static final int FIELD_OPEN_X = 100;
    public static final int FIELD_OPEN_Y = 100;

    // Chart rendering
    public static final int CHART_WIDTH = 1000;
    public static final int CHART_HEIGHT = 600;

    // PID controller gains
    public static final Dual PID_P = new Dual(); // TUNE
    public static final Dual PID_I = new Dual(); // TUNE
    public static final Dual PID_D = new Dual(); // TUNE
    public static final Dual PID_FV = new Dual(); // TUNE
    public static final Dual PID_FA = new Dual(); // TUNE

    // Motor feedforward gains
    public static final double PID_S = 1; // TUNE
    public static final double PID_V = 0; // TUNE
    public static final double PID_A = 0; // TUNE

    // Robot physical constants
    public static final double TURNING_RADIUS = 0; // TUNE
    public static final double LATERAL_MULTIPLIER = 1; // TUNE 

    // Simulator constants
    public static final double SIM_DELTA_TIME = 0.01;
    public static final double POINT_EPSILON = 1.5;
}
