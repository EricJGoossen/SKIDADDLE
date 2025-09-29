package Pathing;

import Simulator.Draw.Graph;
import Util.Constants;
import Util.Path;
import Util.PoseVelAcc;
import Util.Vector;

/**
 * Executes linear path following using piecewise line segments.
 * Respects acceleration, deceleration, and max speed constraints.
 */
public class Linear {
    private static final double ACCEL = Constants.MAX_ACCELERATION;
    private static final double MAX_SPEED = Constants.MAX_SPEED;

    /** Current segment index along the path. */
    private int currSpline;
    /** Maximum velocity allowed on current segment. */
    private Vector maxVel;
    /** Maximum distance along current segment before transitioning. */
    private double maxTransDist;

    /** Segment state: target point, current direction, next segment direction */
    private Vector target;
    private Vector currDir;
    private Vector nextDir;

    /** References */
    private final Graph draw;
    private final Path path;
    private final Kinematics kin;
    private final LookAhead lookAhead;

    /**
     * Constructs a linear path follower.
     *
     * @param initSpeed Initial robot speed
     * @param path      Path to follow
     * @param draw      Graph for visualization
     */
    protected Linear(double initSpeed, Path path, Graph draw) {
        this.draw = draw;
        this.path = path;

        kin = new Kinematics(MAX_SPEED, ACCEL);
        lookAhead = new LookAhead(kin);

        currSpline = 1;

        // Initialize first segment
        target = path.points()[1];
        currDir = path.points()[0].dir(target);

        if (path.points().length > 2) {
            // Precompute next segment transition
            maxVel = lookAhead.compute(path.points(), 0, initSpeed);
            nextDir = target.dir(path.points()[2]);
            maxTransDist = kin.decelTravel(maxVel, currDir, nextDir);
        } else {
            maxVel = new Vector();
            nextDir = currDir;
            maxTransDist = 0;
        }
    }

    /**
     * Advances one step along a path segment.
     *
     * @param dir      Unit vector along the segment
     * @param initPose Current pose, velocity, and acceleration
     * @param speedCap Maximum allowed speed
     * @param deltaT   Time step (s)
     * @return Updated pose
     */
    protected PoseVelAcc lineDrive(Vector dir, PoseVelAcc initPose,
                                   double speedCap, double deltaT) {

        // Parallel and perpendicular components of velocity
        Vector con = initPose.vel.convergent(dir);
        double conMag = con.hypot();
        Vector trans = initPose.vel.transverse(dir);
        double transMag = trans.hypot();

        double maxDelta = ACCEL * deltaT;

        // Reduce transverse velocity
        double transDelta = Math.min(maxDelta, transMag);
        Vector transDeltaVec = trans.norm().times(-transDelta);

        // Remaining acceleration budget along segment direction
        double transInDir = transDeltaVec.dot(dir);
        double conMax = -transInDir + Math.sqrt(
            transInDir * transInDir - transDelta * transDelta + maxDelta * maxDelta
        );

        // Adjust longitudinal velocity towards speedCap
        double targetConDelta = speedCap - conMag;
        double conDelta = Math.min(Math.abs(targetConDelta), conMax)
                        * Math.signum(targetConDelta);
        Vector conDeltaVec = dir.times(conDelta);

        // Combine velocity components
        Vector totalDelta = transDeltaVec.plus(conDeltaVec);
        Vector updatedVel = initPose.vel.plus(totalDelta);

        return new PoseVelAcc(
            initPose.pos.plus(updatedVel.times(deltaT)),
            updatedVel,
            updatedVel.minus(initPose.vel).div(deltaT)
        );
    }

    /**
     * Advances the robot along the current path segment.
     * Transitions to the next segment when necessary.
     *
     * @param initPose Robot pose at the start of this step
     * @param deltaT   Loop time step (s)
     * @return Updated pose after advancing
     */
    protected PoseVelAcc followPath(PoseVelAcc initPose, double deltaT) {
        // Compute remaining distance along segment
        double dist = target.minus(initPose.pos).projMag(currDir);
        double remDist = Math.max(0, dist - maxTransDist);
        double finalDist = path.points()[path.points().length - 1].minus(initPose.pos).hypot();

        // Compute target speed
        double targetSpeed = kin.speedProfile(
            initPose.vel.hypot(),
            maxVel.hypot(),
            remDist,
            deltaT
        );

        // Update pose
        PoseVelAcc updatedPose = lineDrive(currDir, initPose, targetSpeed, deltaT);
        dist = target.minus(updatedPose.pos).projMag(currDir);
        log(updatedPose);

        // Check if robot has stopped
        if (kin.atGoal(updatedPose.vel.hypot(), finalDist, Constants.DISTANCE_EPSILON, deltaT)) {
            updatedPose.moving = false;
            return updatedPose;
        }

        // Predict distance needed to decelerate to next segment
        double decelDist = kin.decelTravel(
            updatedPose.vel.project(currDir), currDir, nextDir
        );

        // Handle segment transition
        if ((decelDist > dist || decelDist < 0) && currSpline < path.points().length - 1) {
            currSpline++;
            target = path.points()[currSpline];
            currDir = nextDir;

            maxVel = lookAhead.compute(path.points(), currSpline, updatedPose.vel.hypot());

            // Only compute nextDir and maxTransDist if another segment exists
            if (currSpline < path.points().length - 1) {
                nextDir = target.dir(path.points()[currSpline + 1]);
                maxTransDist = kin.decelTravel(maxVel, currDir, nextDir);
            } else {
                nextDir = currDir;
                maxTransDist = 0;
            }
        }

        return updatedPose;
    }

    /**
     * Logs path follower state during simulation.
     *
     * @param pose Current robot state
     */
    private void log(PoseVelAcc pose) {
        if (Constants.SIMULATING) {
            draw.addChartData("pos", pose.pos.hypot());
            draw.addChartData("vel", pose.vel.hypot());
            draw.addChartData("accel", pose.accel.hypot());
            draw.addChartData("max", maxVel.hypot());
            draw.addChartData("line", currSpline);

            path.draw(currSpline, draw);
            draw.addPathData("Path traveled", pose.pos);
        }
    }
}