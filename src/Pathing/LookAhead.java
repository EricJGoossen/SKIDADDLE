package Pathing;

import Pathing.Kinematics.TravelResult;
import Util.Constants;
import Util.Vector;
import Util.Path.Arc;
import Util.Path.Path;
import Util.Path.Segment;

/**
 * Computes lookahead velocities for a path, respecting distance and turn constraints.
 */
public final class LookAhead {
    private static final double maxSpeed = Constants.MAX_SPEED;

    /**
     * Computes the maximum velocity at each waypoint between start and end points.
     *
     * @param points Array of path points
     * @param start  Starting index
     * @param end    Ending index
     * @return Array of velocity vectors representing maximum allowed velocity at each waypoint
     */
    public static void vertexLimits(Path path) {
        int len = path.size();
        path.arc(len - 1).setVel(new Vector());

        // Backward pass to compute per-vertex max entry velocity
        for (int i = len - 2; i >= 0; i--) {
            Arc arc = path.arc(i);
            Arc nextArc = path.arc(i + 1);

            // Binary search bounds for feasible speed
            double lower = 0;
            double upper = Math.min(Kinematics.speedOverDist(nextArc.current.dir, nextArc.dist(), nextArc.vel.hypot()).hypot(), maxSpeed);

            // Iteratively refine feasible entry speed for the turn
            while (upper - lower > Constants.ZERO_TOLERANCE) {
                double mid = (upper - lower) / 2.0 + lower;

                // Test whether robot can safely enter turn at speed mid
                if (canTransitionAtSpeed(arc, nextArc, mid)) {
                    lower = mid; // Feasible, can try higher speed
                } else {
                    upper = mid; // Not feasible, reduce speed
                }
            }

            arc.vel = arc.current.dir.times(lower);

            arc.transition = Kinematics.decelTravel(arc.vel, arc, nextArc);
        }
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
    private static boolean canTransitionAtSpeed(Arc arc, Arc nextArc, double speed) {
        Vector trialVel = arc.current.dir.times(speed); //TODO: This should be for the next loop interation, not this ucrrent one.
        TravelResult travel = Kinematics.decelTravel(trialVel, arc, nextArc, nextArc.vel.hypot());
        return travel.next <= nextArc.dist() && travel.curr <= arc.current.mag;
    }
}
