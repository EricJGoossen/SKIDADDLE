package Util.AccelerationCommands;

public class MotorCommand {
    public static final int FL = 0;
    public static final int FR = 1;
    public static final int BL = 2;
    public static final int BR = 3;

    public final double[] speeds;
    public final double weight;
    public final int priority;

    // allocator metadata
    public double scale;
    public boolean saturated;

    public MotorCommand(
            double weight,
            int priority,
            double[] speeds,
            double scale,
            boolean saturated
    ) {
        this.weight = weight;
        this.priority = priority;
        this.speeds = speeds;
        this.scale = scale;
        this.saturated = saturated;
    }

    public MotorCommand(
        double weight,
        int priority,
        double[] speeds
    ) {
        this(weight, priority, speeds, 0.0, false);
    }

    public MotorCommand(double weight, int priority) {
        this(weight, priority, new double[4], 0.0, false);
    }

    public double speed(int i) {
        if (i < 0 || i >= speeds.length) {
            throw new IndexOutOfBoundsException(
                "Motor index " + i + " is out of bounds."
            );
        }

        return speeds[i];
    }

    public double Longitudinal() {
        return ((speeds[FL]
               + speeds[FR]
               + speeds[BL]
               + speeds[BR]) / 4
        );
    }

    public double Lateral(double LAT) {
        return ((speeds[FL]
               - speeds[FR]
               - speeds[BL]
               + speeds[BR]) / (4 * LAT)
        );
    }

    public double Angular(double ROT) {
        return ((speeds[FL]
               - speeds[FR]
               + speeds[BL]
               - speeds[BR]) / (4 * ROT)
        );
    }

    public int motorCount() {
        return speeds.length;
    }

    public double max() {
        double max = 0;
        for (double speed : speeds) {
            if (Math.abs(speed) > Math.abs(max)) {
                max = speed;
            }
        }
        return max;
    }

    public MotorCommand normalize() {
        double result[] = speeds.clone();

        double max = Math.abs(max());
        if (max > 1) {
            for (int i = 0; i < speeds.length; i++) {
                result[i] /= max;
            }
        }

        return new MotorCommand(
            weight, priority, result, scale, saturated
        );
    }

    public MotorCommand plus(MotorCommand m) {
        double result[] = speeds.clone();
        for (int i = 0; i < result.length; i++) {
            result[i] += m.speeds[i];
        }

        return new MotorCommand(
            weight, priority, result, scale, saturated
        );
    }

    public MotorCommand times(double value) {
        double result[] = speeds.clone();

        for (int i = 0; i < result.length; i++) {
            result[i] *= value;
        }

        return new MotorCommand(
            weight, priority, result, scale, saturated
        );
    }

    public static MotorCommand zero() {
        return new MotorCommand(0.0, 0);
    }
}
