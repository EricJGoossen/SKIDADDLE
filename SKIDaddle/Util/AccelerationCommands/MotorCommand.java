package Util.AccelerationCommands;

import Util.MotorVector;

public class MotorCommand {
    public final double weight;
    public final int priority;
    public MotorVector vector;

    public MotorCommand(
            double weight,
            int priority,
            MotorVector vector
    ) {
        this.weight = weight;
        this.priority = priority;
        this.vector = vector;
    }

    public MotorCommand(
        double weight,
        int priority,
        double[] speeds
    ) {
        this(weight, priority, new MotorVector(speeds));
    }

    public MotorCommand(double weight, int priority) {
        this(weight, priority, new MotorVector());
    }
}
