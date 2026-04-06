package Util;

public class Angle {
    private double radians;

    public Angle(double radians) {
        this.radians = radians;
        wrap();
    }

    public Angle() {
        this.radians = 0.0;
    }

    private void wrap() {
        while (radians >  Math.PI) radians -= 2 * Math.PI;
        while (radians < -Math.PI) radians += 2 * Math.PI;
    }

    // Getters/Setters
    public double get() {
        return radians;
    }

    public double abs() {
        return Math.abs(radians);
    }

    public void set(double radians) {
        this.radians = radians;
        wrap();
    }

    // Operations
    public Angle plus(Angle other) {
        return new Angle(this.radians + other.radians);
    }

    public Angle plus(double num) {
        return new Angle(this.radians + num);
    }

    public Angle minus(Angle other) {
        return new Angle(this.radians - other.radians);
    }

    public Angle minus(double num) {
        return new Angle(this.radians - num);
    }

    public Angle times(Angle other) {
        return new Angle(this.radians * other.radians);
    }

    public Angle times(double num) {
        return new Angle(this.radians * num);
    }

    public Angle div(Angle other) {
        return times(1 / other.radians);
    }

    public Angle div(double num) {
        return times(1 / num);
    }
}
