package frc.robot.Mathematics;

public class PID {
    private double proportional;
    private double integral;
    private double derivative;

    public PID (double proportinal, double integral, double derivative) {
        this.proportional = proportinal;
        this.integral = integral;
        this.derivative = derivative;
    }

    public void setP (double p) {
        proportional = p;
    }

    public void setI (double i) {
        integral = i;
    }

    public void setD (double d) {
        derivative = d;
    }

    public double getP () {
        return proportional;
    }

    public double getI () {
        return integral;
    }

    public double getD () {
        return derivative;
    }
}
