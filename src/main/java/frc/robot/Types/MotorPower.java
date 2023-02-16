package frc.robot.Types;

import frc.robot.Exceptions.ValueConstraintException;

/**
 * Motor power as a percentage, represented as a double ranging from [-1.0 to
 * 1.0].
 */
public class MotorPower {
    public static double minPower = -1.0d;
    public static double maxPower = 1.0d;
    private final double raw;

    /**
     * Creates a new representation of motor power.
     * 
     * @param value Motor power as a percentage with a range from [-1.0 to 1.0].
     * @throws ValueConstraintException Thrown when value is out of range.
     */
    public MotorPower(double value) throws ValueConstraintException {
        if (minPower <= value && value <= maxPower) {
            raw = value;
        } else {
            throw new ValueConstraintException(minPower, maxPower, value);
        }
    }

    /**
     * @return The current motor power.
     */
    public double getValue() {
        return raw;
    }

    /**
     * Adds a double to a Motor Power value.
     * 
     * @param lvalue Motor Power value.
     * @param rvalue Double modifier.
     * @return A new Motor Power value.
     * @throws ValueConstraintException Thrown when the resulting value would be out
     *                                  of range [-1.0 to 1.0].
     */
    public static MotorPower add(MotorPower lvalue, double rvalue) throws ValueConstraintException {
        return new MotorPower(lvalue.raw + rvalue);
    }

    /**
     * Adds a Motor Power to a Motor Power value.
     * 
     * @param lvalue Motor Power value.
     * @param rvalue Motor Power modifier.
     * @return A new Motor Power value.
     * @throws ValueConstraintException Thrown when the resulting value would be out
     *                                  of range [-1.0 to 1.0].
     */
    public static MotorPower add(MotorPower lvalue, MotorPower rvalue) {
        return new MotorPower(lvalue.raw + rvalue.raw);
    }

    /** Generates a hash code based on the value. */
    @Override
    public int hashCode() {
        final int prime = 31;
        int result = 1;
        long temp;
        temp = Double.doubleToLongBits(raw);
        result = prime * result + (int) (temp ^ (temp >>> 32));
        return result;
    }

    /**
     * Compares two Motor Power values.
     * 
     * @param other The other Motor Power value.
     * @return true if both values are identical; false otherwise.
     */
    public boolean equals(MotorPower other) {
        return Double.doubleToLongBits(raw) == Double.doubleToLongBits(other.raw);
    }

    /**
     * Compares a Motor Power value with an object.
     * 
     * @param obj The object to compare. Always unequal if obj is not a MotorPower
     *            instance.
     * @return True if both values are identical; false otherwise.
     */
    @Override
    public boolean equals(Object obj) {
        if (this == obj)
            return true;

        if (obj == null)
            return false;

        if (getClass() != obj.getClass())
            return false;

        MotorPower other = (MotorPower) obj;
        return equals(other);
    }
}
