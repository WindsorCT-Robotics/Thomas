package frc.robot.Mathematics;

import frc.robot.Exceptions.ValueConstraintException;

/**
 * Motor power as a percentage, represented as a double ranging from [0.0 to 1.0].
 */
public class MotorPower {
    public static double minPower = -1.0d;
    public static double maxPower = 1.0d;
    private final double raw;

    public MotorPower (double value) throws ValueConstraintException {
        if (minPower <= value && value <= maxPower) {
            raw = value;
        }
        else {
            throw new ValueConstraintException(minPower, maxPower);
        }
    }

    public double getValue() {
        return raw;
    }

    @Override
    public int hashCode() {
        final int prime = 31;
        int result = 1;
        long temp;
        temp = Double.doubleToLongBits(raw);
        result = prime * result + (int) (temp ^ (temp >>> 32));
        return result;
    }

    @Override
    public boolean equals(Object obj) {
        if (this == obj)
            return true;
        if (obj == null)
            return false;
        if (getClass() != obj.getClass())
            return false;
        MotorPower other = (MotorPower) obj;
        if (Double.doubleToLongBits(raw) != Double.doubleToLongBits(other.raw))
            return false;
        return true;
    }
}
