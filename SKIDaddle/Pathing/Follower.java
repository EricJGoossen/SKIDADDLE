package Pathing;

import Util.Path;
import Util.PathSegment;
import Util.Twist;
import Util.AccelerationCommands.AccelerationCommand;
import Util.AccelerationCommands.MotorCommand;
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

        allocateAcceleration(commands, deltaT);

        //State integration
        updateState(commands, deltaT);

        if (segment.idx < path.points().length - 1) {
            segmentTracker(path);
        }

        log(path);
        sim.update(state, deltaT);

        lastPath = path;

        return false;
    }

    public void allocateAcceleration(AccelerationCommand[] commands, double deltaT) {
        final double angle = state.pos.ang;
        final double maxAccel = Constants.MAX_MOTOR_ACCELERATION;

        MotorCommand vel = state.vel.toMotorCommand(angle);
        double velMag = vel.max();
        double outWardAccel = (Constants.MAX_MOTOR_SPEED - Math.abs(vel.max())) / deltaT;

        MotorCommand[] intents = new MotorCommand[commands.length];
        for (int i = 0; i < commands.length; i++) {
            intents[i] = commands[i].toMotorCommand(angle);
        }

        int numActive = intents.length;
        double budget = 1.0;

        while (budget > 0.0 && numActive > 0) {
            int priority = 0;
            double weightSum = 0.0;

            for (int i = 0; i < numActive; i++) {
                MotorCommand intent = intents[i];

                if (intent.priority > priority) {
                    priority = intent.priority;
                    weightSum = intent.weight;
                } else if (intent.priority == priority) {
                    weightSum += intent.weight;
                }
            }

            if (weightSum == 0.0) break;

            double usedBudget = 0.0;

            for (int i = 0; i < numActive; i++) {
                MotorCommand intent = intents[i];
                if (intent.priority < priority) continue;

                double delta = intent.weight * budget / weightSum;

                double targetAccel = intent.max();
                if (targetAccel * velMag > 0) targetAccel = Math.min(outWardAccel, maxAccel);
                double scaleLimit = Math.abs(targetAccel) / maxAccel;
                
                double newScale = intent.scale + delta;

                if (newScale >= scaleLimit) {
                    delta = scaleLimit - intent.scale;
                    intent.scale = scaleLimit;
                    intent.saturated = true;
                } else {
                    intent.scale = newScale;
                }

                usedBudget += delta;
            }

            budget -= usedBudget;

            for (int i = 0; i < numActive;) {
                if (intents[i].saturated) {
                    intents[i] = intents[numActive - 1];
                    numActive--;
                } else {
                    i++;
                }
            }
        }

        MotorCommand[] scaled = new MotorCommand[commands.length];
        for (int i = 0; i < intents.length; i++) {
            MotorCommand intent = intents[i];

            if (intent.scale == 0.0) {
                scaled[i] = MotorCommand.zero();
            } else {
                scaled[i] = intent.normalize().times(maxAccel * intent.scale);
            }
        }

        for (int i = 0; i < commands.length; i++) {
            commands[i] = commands[i].fromMotorCommand(scaled[i], angle);
        }
    }


    public void updateState(AccelerationCommand[] commands, double deltaT) {
        state.accel = new Twist();

        for (AccelerationCommand command : commands) {
            command.update(state);
        }

        state.vel = state.vel.plus(state.accel.times(deltaT));
        if (Math.abs(state.vel.sum()) < Constants.VEL_EPSILON) state.vel = new Twist();

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

        draw.addChartData("vel", Math.abs(state.vel.lin.x) + Math.abs(state.vel.lin.y));
        draw.addChartData("accel", Math.abs(state.accel.lin.x) + Math.abs(state.accel.lin.y));
        draw.addChartData("max", segment.maxVel.hypot());
        draw.addChartData("line", segment.idx);

        draw.addChartData("angleVel", state.vel.ang);
        draw.addChartData("angleAccel", state.accel.ang);
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
        draw.createChart("line", 1, Color.MAGENTA, 3);

        draw.createChart("angle", 1, Color.YELLOW, 3);
        draw.createChart("angleVel", 1, Color.BLUE, 3);
        draw.createChart("angleAccel", 1, Color.RED, 3);
    }
}
