package Util.AccelerationCommands;

import Util.MotionState;

public abstract class AccelerationCommand {
    public final double weight;
    public final int priority;

    protected AccelerationCommand(double weight, int priority) {
        this.weight = weight;
        this.priority = priority;
    }

    public abstract MotorCommand toMotorCommand(double angle);
    public abstract AccelerationCommand fromMotorCommand(MotorCommand motorCommand, double angle);

    public abstract boolean isZero();
    public abstract void update(MotionState currentState);
}
