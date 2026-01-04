package Pathing;

import Util.Twist;
import Util.AccelerationCommands.AccelerationCommand;
import Util.Path.Arc;
import Simulator.Simulator;
import Simulator.Draw.Graph;
import Util.Constants;
import Util.MotionState;

import java.awt.Color;

import Util.Path.Path;

public class Follower {

    final Scheduler scheduler;
    final Simulator sim;
    final Graph draw;

    MotionState state;
    Arc arc;
    int index = 0;

    Path lastPath;

    public Follower(Scheduler scheduler, MotionState state, Graph draw) {
        this.scheduler = scheduler;
        this.state = state;
        this.draw = draw;

        arc = new Arc();
        sim = new Simulator(new MotionState(state), draw);

        initLog();

        lastPath = null;
    }
    
    public boolean update(MotionState sensorState, Path path, double deltaT) {
        index++;
        if (lastPath != path) {
            arc = path.arc(0);
            arc.transition = arc.index < path.size() - 1 ?
                Kinematics.decelTravel(arc.vel, arc, path.arc(1)) :
                0.0;

            scheduler.init();
        }

        AccelerationCommand[] commands = scheduler.update(state, arc, deltaT);

        if (scheduler.isFinished() && commands.length == 0) {
            return true;
        }

        Twist accel = Allocator.acceleration(commands, state.vel, state.pos.ang, deltaT);

        //State integration
        updateState(accel, deltaT);

        if (arc.index < path.size() - 1) {
            arcTracker(path);
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

    public void arcTracker(Path path) {
        double dist = arc.distAlongSegment(state.pos.lin);
        Arc next = path.arc(arc.index + 1);

        // Predict distance needed to decelerate to next arc
        double decelDist = Kinematics.decelTravel(
            state.vel.lin.project(arc.current.dir), arc, next
        );

        // Handle arc transition
        if ((decelDist > dist || decelDist < 0)) {
            arc = next;

            arc.transition = arc.index < path.size() - 1 ?
                Kinematics.decelTravel(arc.vel, arc, path.arc(arc.index + 1)) :
                0.0;
        }
    }

    private void log(Path path) {
        if (!Constants.SIMULATING || draw == null) return;

        path.draw(arc.index, draw, state.pos.lin);
        draw.addPathData("Path traveled", state.pos.lin);

        draw.addChartData("vel", Math.abs(state.vel.toMotorSpace(state.pos.ang).max()));
        draw.addChartData("accel", Math.abs(state.accel.toMotorSpace(state.pos.ang).max()));
        draw.addChartData("max", arc.vel.hypot());
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
