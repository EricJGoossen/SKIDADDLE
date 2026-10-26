package Util;

import Pathing.SpaceKinematics;

public class MotorVector {
    public static final int FL = 0;
    public static final int FR = 1;
    public static final int BL = 2;
    public static final int BR = 3;

    public final double[] speeds;

    // Constructors
    public MotorVector(double[] speeds) {
        if (speeds.length != 4) {
            throw new IllegalArgumentException("MotorVector must have exactly 4 motor speeds.");
        }
        this.speeds = speeds;
    }

    public MotorVector(MotorVector other) {
        this.speeds = other.speeds.clone();
    }

    public MotorVector(double num) {
        this.speeds = new double[] {num, num, num, num};
    }

    public MotorVector() {
        this.speeds = new double[] {0, 0, 0, 0};
    }

    // Getters
    public int motorCount() {
        return speeds.length;
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

    // Operations
    public MotorVector plus(MotorVector v) {
        double result[] = speeds.clone();
        for (int i = 0; i < result.length; i++) {
            result[i] += v.speeds[i];
        }

        return new MotorVector(result);
    }

    public MotorVector plus(double value) {
        double result[] = speeds.clone();

        for (int i = 0; i < result.length; i++) {
            result[i] += value;
        }

        return new MotorVector(result);
    }

    public MotorVector minus(MotorVector v) {
        double result[] = speeds.clone();

        for (int i = 0; i < result.length; i++) {
            result[i] -= v.speeds[i];
        }

        return new MotorVector(result);
    }

    public MotorVector times(double value) {
        double result[] = speeds.clone();

        for (int i = 0; i < result.length; i++) {
            result[i] *= value;
        }

        return new MotorVector(result);
    }

    public MotorVector div(double value) {
        return times(1.0 / value);
    }

    // Vector operations
    public MotorVector normalize() {
        double result[] = speeds.clone();

        double max = Math.abs(max());
        for (int i = 0; i < speeds.length; i++) {
            result[i] /= max;
        }

        return new MotorVector(result);
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

    public int maxIndex() {
        double max = 0;
        int index = 0;
        for (int i = 0; i < speeds.length; i++) {
            if (Math.abs(speeds[i]) > Math.abs(max)) {
                max = speeds[i];
                index = i;
            }
        }
        return index;
    }

    public Twist toTwist(double angle) {
        return SpaceKinematics.forwardTwist(this, angle);
    }
}
