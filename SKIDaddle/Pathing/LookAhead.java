package Pathing;

import Pathing.Kinematics.TravelResult;
import Util.Constants;
import Util.Vector;

/**
 * Computes lookahead velocities for a path, respecting distance and turn constraints.
 */
public class LookAhead {
    private final double maxSpeed = Constants.MAX_SPEED;
    private final Kinematics kin;

    /**
     * Constructs a LookAhead helper.
     *
     * @param kin Kinematics object for deceleration calculations
     */
    protected LookAhead(Kinematics kin) {
        this.kin = kin;
    }

    /**
     * Computes the target velocity at a given start waypoint, factoring lookahead distance.
     *
     * @param points Array of path points
     * @param start  Index of starting point
     * @param speed  Current speed of the robot
     * @return Target velocity vector at start waypoint
     */
    protected Vector compute(Vector[] points, int start, double speed) {
        double stopDist = kin.stopDist(speed) * Constants.LOOK_AHEAD_FACTOR;
        int end = findFromArc(points, start, stopDist);
        return vertexLimits(points, start, end)[0];
    }

    /**
     * Finds the furthest point reachable along the path given a travel distance.
     */
    private int findFromArc(Vector[] points, int start, double dist) {
        double remainingDist = dist;
        int index = start;

        while (remainingDist >= 0 && index + 1 < points.length) {
            index++;
            remainingDist -= points[index].minus(points[index - 1]).hypot();
        }

        return index;
    }

    /**
     * Computes the maximum velocity at each waypoint between start and end points.
     *
     * @param points Array of path points
     * @param start  Starting index
     * @param end    Ending index
     * @return Array of velocity vectors representing maximum allowed velocity at each waypoint
     */
    private Vector[] vertexLimits(Vector[] points, int start, int end) {
        int len = end - start + 1;
        Vector[] segMaxVels = new Vector[len];

        // Set velocity at last waypoint
        if (end + 1 >= points.length) {
            segMaxVels[len - 1] = new Vector(0, 0); // Robot must stop at the end
        } else {
            double max = Math.sqrt(Constants.MAX_SPEED * Constants.MAX_SPEED / 2.0);
            segMaxVels[len - 1] = new Vector(max, max); // Precompute safe turn-limited speed
        }

        // Backward pass to compute per-vertex max entry velocity
        for (int i = len - 2; i >= 0; i--) {
            Vector curr = points[start + i];
            Vector next = points[start + i + 1];

            Vector nextSeg = next.minus(curr);
            double nextDist = nextSeg.hypot();
            Vector nextSegDir = (nextDist > 0) ? nextSeg.div(nextDist) : new Vector(0, 0);

            Vector nextVel = segMaxVels[i + 1];

            // Adjust nextDist based on how far the robot must decelerate to enter future turn
            if (i + 2 < len) {
                Vector futureDir = points[start + i + 2].minus(next).norm();
                double adjust = kin.decelTravel(nextVel, nextSegDir, futureDir);
                nextDist = Math.max(0, nextDist - adjust); // Clamp to zero to avoid negative distance
            }

            // Compute max velocity at current vertex
            if (i > 0) {
                // Use turn-aware speed calculation if not at the first vertex
                segMaxVels[i] = findTurnSpeed(points[start + i - 1], curr, nextSegDir, nextDist, nextVel.hypot());
            } else {
                // For first vertex, compute max speed along straight line to next
                segMaxVels[i] = kin.speedOverDist(nextSegDir, nextDist, nextVel.hypot());
            }
        }

        return segMaxVels;
    }

    /**
     * Computes the maximum entry speed for a turn while respecting deceleration limits.
     *
     * @param lastP      Previous waypoint
     * @param currP      Current waypoint
     * @param nextSegDir Next segment direction
     * @param nextDist   Distance to next waypoint
     * @param exitSpeed  Required speed at end of next segment
     * @return Maximum feasible velocity vector for current waypoint
     */
    private Vector findTurnSpeed(Vector lastP, Vector currP, Vector nextSegDir, double nextDist, double exitSpeed) {
        Vector lastSeg = currP.minus(lastP);
        double lastDist = lastSeg.hypot();
        Vector lastSegDir = lastSeg.div(lastDist);

        // Binary search bounds for feasible speed
        double lower = 0;
        double upper = Math.min(kin.speedOverDist(nextSegDir, nextDist, exitSpeed).hypot(), maxSpeed);

        // Iteratively refine feasible entry speed for the turn
        while (upper - lower > Constants.ZERO_TOLERANCE) {
            double mid = (upper - lower) / 2.0 + lower;

            // Test whether robot can safely enter turn at speed mid
            if (canTransitionAtSpeed(mid, lastSegDir, nextSegDir, lastDist, nextDist, exitSpeed)) {
                lower = mid; // Feasible, can try higher speed
            } else {
                upper = mid; // Not feasible, reduce speed
            }
        }

        return lastSegDir.times(lower); // Return feasible velocity vector along last segment
    }

    /**
     * Checks whether a given entry speed allows safe transition through a turn.
     *
     * @param speed        Candidate entry speed
     * @param lastSegDir   Direction of incoming segment
     * @param nextSegDir   Direction of outgoing segment
     * @param lastDist     Distance remaining in incoming segment
     * @param nextDist     Distance available in outgoing segment
     * @param exitSpeed    Required speed at the end of the next segment
     * @return True if transition is feasible
     */
    private boolean canTransitionAtSpeed(double speed, Vector lastSegDir, Vector nextSegDir,
                                         double lastDist, double nextDist, double exitSpeed) {
        Vector trialVel = lastSegDir.times(speed);
        TravelResult travel = kin.decelTravel(trialVel, exitSpeed, lastSegDir, nextSegDir);
        return travel.next <= nextDist && travel.curr <= lastDist;
    }
}
