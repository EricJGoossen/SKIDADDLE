package Pathing.Controllers;

import Pathing.Scheduler.AccelerationController;
import Util.MotionState;
import Util.AccelerationCommands.AccelerationCommand;
import Util.AccelerationCommands.AngularAccelCommand;
import Util.AccelerationCommands.LinearAccelCommand;
import Util.Path.Arc;

public class DefaultController implements AccelerationController{
    public interface CommandGenerator {
        AccelerationCommand[] getCommand(MotionState state, Arc arc, double deltaT);
    }

    private final double weight = 1;
    private final int priority = 0;

    private CommandGenerator generator;

    public DefaultController() {
        this.generator = this::stop;
    }

    public void init() {}

    public AccelerationCommand[] stop(MotionState state, Arc arc, double deltaT) {
        LinearAccelCommand linearCommand = new LinearAccelCommand(state.vel.lin.div(-deltaT), weight, priority);
        AngularAccelCommand angularCommand = new AngularAccelCommand(-state.vel.ang / deltaT, weight, priority);

        AccelerationCommand[] commands = {linearCommand, angularCommand};

        return commands;
    }

    public AccelerationCommand[] update(MotionState state, Arc arc, double deltaT) {
         if (generator == null) {
            throw new IllegalStateException("Default command generator is null");
        }

        return generator.getCommand(state, arc, deltaT);
    }

    public void setGenerator(CommandGenerator generator) {
        this.generator = generator;
    }

    public boolean running(MotionState state, Arc arc, double deltaT) {return true;}

    public boolean interrupt() {return true;}

    public boolean terminate() {return false;}
}
