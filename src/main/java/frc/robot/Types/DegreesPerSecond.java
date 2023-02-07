package frc.robot.Types;

public class DegreesPerSecond {
    private final double rawValue;

    /** Creates a new value in degrees per second */
    public DegreesPerSecond(double value) {
        rawValue = value;
    }

    /** @return the value as a double */
    public double getDegreesPerSecond() {
        return rawValue;
    }

    /**
     * Determines equality between two values in milliseconds.
     * 
     * @param obj The object to compare. Always unequal if obj is not a
     *            MetersPerSecond instance.
     * @return true if equal; false otherwise.
     */
    @Override
    public boolean equals(Object obj) {
        if (this == obj)
            return true;

        if (obj == null)
            return false;

        if (getClass() != obj.getClass())
            return false;

        MetersPerSecond other = (MetersPerSecond) obj;
        return equals(other);
    }

    public boolean equals(DegreesPerSecond other) {
        return rawValue == other.rawValue;
    }

    @Override
    public int hashCode() {
        // TODO Auto-generated method stub
        return super.hashCode();
    }

    /**
     * @returns the value represented as a string.
     */
    @Override
    public String toString() {
        return Double.toString(rawValue);
    }
}
