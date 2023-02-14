package frc.robot.Types;

import frc.robot.Exceptions.ValueConstraintException;

/**
 * Joystick Axis value as a percentage, represented as a double ranging from
 * [-1.0 to
 * 1.0].
 */
public class JoystickAxis {
    public static int minValue = -100;
    public static int maxValue = 100;
    private final int value;

    /**
     * Creates a new representation of a Joystick Axis.
     * 
     * @param value Joystick value as a percentage with a range from [-100 to 100].
     * @throws ValueConstraintException Thrown when value is out of range.
     */
    public JoystickAxis(double value) throws ValueConstraintException {
        this((int)(value * 100));
    }

    /**
     * Creates a new representation of a Joystick Axis.
     * 
     * @param value Joystick value as a percentage with a range from [-100 to 100].
     * @throws ValueConstraintException Thrown when value is out of range.
     */
    public JoystickAxis(int value) throws ValueConstraintException {
        if (minValue <= value && value <= maxValue) {
            this.value = value;
        } else {
            throw new ValueConstraintException(minValue, maxValue, value);
        }
    }

    /**
     * @return The current Joystick Axis value.
     */
    public int getValue() {
        return value;
    }

    public double getAsDouble() {
        return ((double)value) / 100.0d;
    }

    /**
     * Adds a double to a joystick value.
     * 
     * @param lvalue Joystick value.
     * @param rvalue Double modifier.
     * @return A new joystick value.
     * @throws ValueConstraintException Thrown when the resulting value would be out
     *                                  of range [-100 to 100].
     */
    public static JoystickAxis add(JoystickAxis lvalue, double rvalue) throws ValueConstraintException {
        return new JoystickAxis(lvalue.value + (int)(rvalue * 100));
    }

        /**
     * Adds an integer to a joystick value.
     * 
     * @param lvalue Joystick value.
     * @param rvalue Integer modifier.
     * @return A new joystick value.
     * @throws ValueConstraintException Thrown when the resulting value would be out
     *                                  of range [-100 to 100].
     */
    public static JoystickAxis add(JoystickAxis lvalue, int rvalue) throws ValueConstraintException {
        return new JoystickAxis(lvalue.value + rvalue);
    }

    /**
     * Adds a Joystick Axis to a Joystick Axis value.
     * 
     * @param lvalue Joystick value.
     * @param rvalue Joystick modifier.
     * @return A new joystick value.
     * @throws ValueConstraintException Thrown when the resulting value would be out
     *                                  of range [-100 to 100].
     */
    public static JoystickAxis add(JoystickAxis lvalue, JoystickAxis rvalue) {
        return new JoystickAxis(lvalue.value + rvalue.value);
    }

    /** Generates a hash code based on the value. */
    @Override
    public int hashCode() {
        final int prime = 31;
        int result = 1;
        result = prime * result + (int) (value ^ (value >>> 32));
        return result;
    }

    /**
     * Compares two Joystick Axis values.
     * 
     * @param other The other Joystick Axis value.
     * @return true if both values are identical; false otherwise.
     */
    public boolean equals(JoystickAxis other) {
        return other.value == value;
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
