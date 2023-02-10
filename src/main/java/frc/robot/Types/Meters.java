package frc.robot.Types;

/** Represents a distance in meters */
public class Meters {
    private final double rawValue;

    /** Creates a new value in meters */
    public Meters(double value) {
        rawValue = value;
    }

    /** @return the value as a double */
    public double getMeters() {
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

        Meters other = (Meters) obj;
        return equals(other);
    }

    public boolean equals(Meters other) {
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
