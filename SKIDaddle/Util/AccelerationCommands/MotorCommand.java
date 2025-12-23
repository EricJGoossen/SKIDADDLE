package Util.AccelerationCommands;

public class MotorCommand {
    public static final int FL = 0;
    public static final int FR = 1;
    public static final int BL = 2;
    public static final int BR = 3;

    public double[] speeds = new double[4];
    public final double weight;
    public double scale = 0.0f;

    public MotorCommand(double weight, double scale) {
        this.weight = weight;
    }

    public MotorCommand(double weight, double scale, double[] speeds) {
        this.weight = weight;
        this.speeds = speeds;
    }

    public double total() {
        return speeds[FL] + speeds[FR] + speeds[BL] + speeds[BR];
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

    public void normalize() {
        double max = Math.abs(max());
        if (max > 1) {
            for (int i = 0; i < speeds.length; i++) {
                speeds[i] /= max;
            }
        }
    }

    public void times(double value) {
        for (int i = 0; i < speeds.length; i++) {
            speeds[i] *= value;
        }
    }

    public static MotorCommand sum(MotorCommand[] commands) {
        double[] sumSpeeds = new double[4];
        double sumWeight = 0;

        for (MotorCommand command : commands) {
            for (int i = 0; i < 4; i++) {
                sumSpeeds[i] += command.speeds[i];
            }
            sumWeight += command.weight;
        }

        return new MotorCommand(sumWeight, 0, sumSpeeds);
    }
}
