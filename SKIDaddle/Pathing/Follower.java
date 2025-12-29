package Pathing;

import Util.Path;
import Util.PathSegment;
import Util.Twist;
import Util.AccelerationCommands.AccelerationCommand;
import Simulator.Simulator;
import Simulator.Draw.Graph;
import Util.Constants;
import Util.MotionState;

import java.awt.Color;

public class Follower {

    final Scheduler scheduler;
    final Kinematics kin;
    final LookAhead lookAhead;
    final Simulator sim;
    final Graph draw;

    MotionState state;
    PathSegment segment;
    int index = 0;

    Path lastPath;

    public Follower(Scheduler scheduler, MotionState state, Graph draw) {
        this.scheduler = scheduler;
        this.state = state;
        this.draw = draw;

        segment = new PathSegment();
        kin = new Kinematics(Constants.MAX_MOTOR_SPEED, Constants.MAX_MOTOR_ACCELERATION);
        lookAhead = new LookAhead(kin);
        sim = new Simulator(new MotionState(state), draw);

        initLog();

        lastPath = null;
    }
    
    public boolean update(MotionState sensorState, Path path, double deltaT) {
        index++;
        if (lastPath != path) {
            segment.set(path, 1);
            segment.maxVel = lookAhead.compute(path.points(), segment.idx, state.vel.lin.hypot());
            segment.transitionDistance = segment.idx < path.points().length - 1 ?
                kin.decelTravel(segment.maxVel, segment.last.dir(segment.current), segment.current.dir(segment.next)) :
                0.0;

            scheduler.init();
        }

        AccelerationCommand[] commands = scheduler.update(state, segment, deltaT);

        if (scheduler.isFinished() && commands.length == 0) {
            return true;
        }

        Twist accel = Allocator.acceleration(commands, state.vel, state.pos.ang, deltaT);

        //State integration
        updateState(accel, deltaT);

        if (segment.idx < path.points().length - 1) {
            segmentTracker(path);
        }

        log(path);
        sim.update(state, deltaT);

        lastPath = path;

        return false;
    }

    public void updateState(Twist accel, double deltaT) {
        state.accel = accel;
        state.vel = state.vel.plus(state.accel.times(deltaT));
        state.pos = state.pos.plus(state.vel.times(deltaT));
    }

    public void segmentTracker(Path path) {
        double dist = segment.distAlongSegment(state.pos.lin);

        // Predict distance needed to decelerate to next segment
        double decelDist = kin.decelTravel(
            state.vel.lin.project(segment.last.dir(segment.current)), segment.last.dir(segment.current), segment.current.dir(segment.next)
        );

        // Handle segment transition
        if ((decelDist > dist || decelDist < 0)) {
            segment.advance(path);

            segment.maxVel = lookAhead.compute(path.points(), segment.idx, state.vel.lin.hypot());

            segment.transitionDistance = segment.idx < path.points().length - 1 ?
                kin.decelTravel(segment.maxVel, segment.last.dir(segment.current), segment.current.dir(segment.next)) :
                0.0;
        }
    }

    private void log(Path path) {
        if (!Constants.SIMULATING || draw == null) return;

        path.draw(segment.idx, draw, state.pos.lin);
        draw.addPathData("Path traveled", state.pos.lin);

        draw.addChartData("vel", Math.abs(state.vel.toMotorSpace(state.pos.ang).max()));
        draw.addChartData("accel", Math.abs(state.accel.toMotorSpace(state.pos.ang).max()));
        draw.addChartData("max", segment.maxVel.hypot());
    }

    /**
     * Initializes simulation charts and paths for visualization.
     */
    private void initLog() {
        if (!Constants.SIMULATING || draw == null) return;

        draw.createPath("Path traveled", 1f, Color.RED, 6f);

        draw.createChart("pos", 1, Color.YELLOW, 3);
        draw.createChart("vel", 1, Color.BLUE, 3);
        draw.createChart("accel", 1, Color.RED, 3);
        draw.createChart("max", 1, Color.GREEN, 3);

        draw.createChart("angle", 1, Color.YELLOW, 3);
    }
}
