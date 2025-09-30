package Pathing;

import Util.PoseVelAcc;
import Util.Constants;
import Util.MotionState;
import Util.Vector;
import Simulator.Draw.Graph;

/**
 * Handles angular motion for a robot.
 * Converts angular position and velocity commands into updated PoseVelAcc states.
 * Respects maximum angular acceleration and allows customizable orientation controllers.
 */
public class Angular {
    private static final double ACCEL = Constants.MAX_ANGULAR_ACCELERATION;
    private static final double MAX_VEL = Constants.MAX_ANGULAR_VELOCITY;

    private Controller controller; // Controller lambda for angular reference
    private final Kinematics kin;  // Kinematics helper for speed/accel limits
    private final Graph draw;      // Graph for logging/visualization
     
    /**
     * Functional interface for angular controllers.
     * Returns desired angular vector given current robot pose and timestep.
     */
    public interface Controller {
        Vector getAngular(MotionState pose, double deltaT);
    }

    /**
     * Constructs an Angular motion handler.
     *
     * @param controller Angular controller lambda
     * @param kin        Kinematics object
     * @param draw       Graph for visualization/logging
     */
    protected Angular(Controller controller, Graph draw) {
        this.draw = draw;

        kin = new Kinematics(MAX_VEL, ACCEL);

        setController(controller);
    }

    /**
     * Sets the angular controller lambda.
     * Defaults to returning the current angle if controller is null.
     *
     * @param controller Lambda providing desired orientation
     */
    protected void setController(Controller controller) {
        if (controller == null) {
            controller = (pose, deltaT) -> pose.pos.ang;
        }
        this.controller = controller;
    }

    /**
     * Generates a simple controller to turn the robot to a fixed angle.
     *
     * @param theta Target angle in radians
     * @return Lambda controller returning desired angular vector
     */
    public static Controller turnToAngle(double theta) {
        return (pose, deltaT) -> new Vector(Math.sin(theta), Math.cos(theta));
    }

    /**
     * Wraps an angle into the range [-π, π].
     *
     * @param num Angle in radians
     * @return Equivalent angle within [-π, π]
     */
    private double clamp(double num) {
        if (num > Math.PI) return clamp(num - 2 * Math.PI);
        if (num < -Math.PI) return clamp(num + 2 * Math.PI);
        return num;
    }

    /**
     * Updates angular PoseVelAcc for one timestep.
     * Applies acceleration constraints and the current controller's target orientation.
     *
     * @param initPose Current robot angular state
     * @param deltaT   Time step in seconds
     * @return Updated angular PoseVelAcc
     */
    protected PoseVelAcc update(MotionState initPose, double deltaT) {
        if (deltaT < Constants.ZERO_TOLERANCE) return initPose.angular();

        // Current angle and speed
        Vector initAngle = initPose.pos.ang; 
        double initSpeed = initPose.vel.ang.projMag(initAngle.perp());

        // Desired target orientation
        Vector targetAngle = controller.getAngular(initPose, deltaT);

        // Angle difference
        double deltaTheta = clamp(targetAngle.angle() - initAngle.angle());

        // Determine rotation direction
        double dir = (Math.abs(initSpeed) < Constants.ZERO_TOLERANCE)
            ? Math.signum(deltaTheta)
            : Math.signum(initSpeed * deltaTheta) * Math.signum(initSpeed);

        // Check if robot is already at goal
        if (kin.atGoal(initSpeed, deltaTheta, Constants.ANGLE_EPSILON, deltaT)) {
            PoseVelAcc finalPose = new PoseVelAcc(initAngle, new Vector(), initPose.vel.ang);
            finalPose.moving = false;
            return finalPose;
        }

        // Compute acceleration-limited angular velocity
        double maxDelta = ACCEL * deltaT;
        double rawSpeed = kin.speedProfile(initSpeed * dir, deltaTheta, deltaT) * dir;
        double angularSpeed = initSpeed + Math.max(-maxDelta, Math.min(rawSpeed - initSpeed, maxDelta));
        double angularAccel = (angularSpeed - initSpeed) / deltaT;

        // Update PoseVelAcc
        Vector updatedVel = initAngle.perp().times(angularSpeed);
        Vector updatedAccel = initAngle.perp().times(angularAccel);
        Vector updatedAngle = initAngle.plus(updatedVel.times(deltaT)).norm();

        PoseVelAcc updatedPose = new PoseVelAcc(updatedAngle, updatedVel, updatedAccel);
        log(updatedPose);

        return updatedPose;
    }

    /**
     * Logs angular state for simulation visualization.
     *
     * @param pose Current angular PoseVelAcc
     */
    private void log(PoseVelAcc pose) {
        if (!Constants.SIMULATING || draw == null) return;
        draw.addChartData("angle", Math.abs(Math.toDegrees(pose.pos.angle())));
        draw.addChartData("angleVel", pose.vel.hypot());
        draw.addChartData("angleAccel", pose.accel.hypot());
}
}
