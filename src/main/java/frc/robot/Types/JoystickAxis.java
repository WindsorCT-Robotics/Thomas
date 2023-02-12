package frc.robot.Types;

import frc.robot.Exceptions.ValueConstraintException;

/**
 * Joystick Axis value as a percentage, represented as a double ranging from
 * [-1.0 to
 * 1.0].
 */
public class JoystickAxis {
    public static double minValue = -1.0d;
    public static double maxValue = 1.0d;
    private final double value;

    /**
     * Creates a new representation of a Joystick Axis.
     * 
     * @param value Joystick value as a percentage with a range from [-1.0 to 1.0].
     * @throws ValueConstraintException Thrown when value is out of range.
     */
    public JoystickAxis(double value) throws ValueConstraintException {
        if (minValue <= value && value <= maxValue) {
            this.value = value;
        } else {
            throw new ValueConstraintException(minValue, maxValue);
        }
    }

    /**
     * @return The current Joystick Axis value.
     */
    public double getValue() {
        return value;
    }

    /**
     * Adds a double to a joystick value.
     * 
     * @param lvalue Joystick value.
     * @param rvalue Double modifier.
     * @return A new joystick value.
     * @throws ValueConstraintException Thrown when the resulting value would be out
     *                                  of range [-1.0 to 1.0].
     */
    public static JoystickAxis add(JoystickAxis lvalue, double rvalue) throws ValueConstraintException {
        return new JoystickAxis(lvalue.value + rvalue);
    }

    /**
     * Adds a Joystick Axis to a Joystick Axis value.
     * 
     * @param lvalue Joystick value.
     * @param rvalue Joystick modifier.
     * @return A new joystick value.
     * @throws ValueConstraintException Thrown when the resulting value would be out
     *                                  of range [-1.0 to 1.0].
     */
    public static JoystickAxis add(JoystickAxis lvalue, JoystickAxis rvalue) {
        return new JoystickAxis(lvalue.value + rvalue.value);
    }

    /** Generates a hash code based on the value. */
    @Override
    public int hashCode() {
        final int prime = 31;
        int result = 1;
        long temp;
        temp = Double.doubleToLongBits(value);
        result = prime * result + (int) (temp ^ (temp >>> 32));
        return result;
    }

    /**
     * Compares two Joystick Axis values.
     * 
     * @param other The other Joystick Axis value.
     * @return true if both values are identical; false otherwise.
     */
    public boolean equals(JoystickAxis other) {
        return Double.doubleToLongBits(value) == Double.doubleToLongBits(other.value);
    }

    /**
     * Compares a joystick axis value with an object.
     * 
     * @param obj The object to compare. Always unequal if obj is not a JoystickAxis
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

        JoystickAxis other = (JoystickAxis) obj;
        return equals(other);
    }
}
