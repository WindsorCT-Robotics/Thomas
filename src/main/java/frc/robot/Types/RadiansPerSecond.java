package frc.robot.Types;

public class RadiansPerSecond {
    private final double rawValue;

    /** Creates a new value in degrees per second */
    public RadiansPerSecond(double value) {
        rawValue = value;
    }

    /** @return the value as a double */
    public double getRadiansPerSecond() {
        return rawValue;
    }

    /**
     * Determines equality between two values in milliseconds.
     * 
     * @param obj The object to compare. Always unequal if obj is not a
     *            RadiansPerSecond instance.
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

        RadiansPerSecond other = (RadiansPerSecond) obj;
        return equals(other);
    }

    public boolean equals(RadiansPerSecond other) {
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
