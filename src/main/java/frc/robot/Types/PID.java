package frc.robot.Types;

import edu.wpi.first.math.controller.PIDController;

public class PID {
    private double proportional;
    private double integral;
    private double derivative;

    public PID(double proportinal, double integral, double derivative) {
        this.proportional = proportinal;
        this.integral = integral;
        this.derivative = derivative;
    }

    public void setP(double p) {
        proportional = p;
    }

    public void setI(double i) {
        integral = i;
    }

    public void setD(double d) {
        derivative = d;
    }

    public double getP() {
        return proportional;
    }

    public double getI() {
        return integral;
    }

    public double getD() {
        return derivative;
    }

    public PIDController toPIDController() {
        return new PIDController(proportional, integral, derivative);
    }

    @Override
    public int hashCode() {
        final int prime = 31;
        int result = 1;
        long temp;
        temp = Double.doubleToLongBits(proportional);
        result = prime * result + (int) (temp ^ (temp >>> 32));
        temp = Double.doubleToLongBits(integral);
        result = prime * result + (int) (temp ^ (temp >>> 32));
        temp = Double.doubleToLongBits(derivative);
        result = prime * result + (int) (temp ^ (temp >>> 32));
        return result;
    }

    public boolean equals(PID other) {
        return Double.doubleToLongBits(proportional) == Double.doubleToLongBits(other.proportional)
                && Double.doubleToLongBits(integral) == Double.doubleToLongBits(other.integral)
                && Double.doubleToLongBits(derivative) == Double.doubleToLongBits(other.derivative);
    }

    @Override
    public boolean equals(Object obj) {
        if (this == obj)
            return true;

        if (obj == null)
            return false;

        if (getClass() != obj.getClass())
            return false;

        PID other = (PID) obj;
        return equals(other);
    }

    @Override
    public String toString() {
        return "PID [proportional=" + proportional + ", integral=" + integral + ", derivative=" + derivative + "]";
    }
}
