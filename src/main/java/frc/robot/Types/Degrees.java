package frc.robot.Types;

/**
 * Represents a value in degrees.
 */
public class Degrees {
    // TODO: Determine min and max for degrees if possible
    private final double raw;

    /** Creates a new value in degrees. */
    public Degrees (double value) {
        raw = value;
    }

    /**
     * @return The value as a double.
     */
    public double getValue () {
        return raw;
    }

    /** Generates a hash based on the value. */
    @Override
    public int hashCode() {
        final int prime = 31;
        int result = 1;
        long temp;
        temp = Double.doubleToLongBits(raw);
        result = prime * result + (int) (temp ^ (temp >>> 32));
        return result;
    }
    
    /** Determines equality between two Degrees.
     * 
     * @param other the Degrees to compare this to.
     * @return true if values are identical; false otherwise
     */
    public boolean equals(Degrees other) {
        return Double.doubleToLongBits(raw) == Double.doubleToLongBits(other.raw);        
    }

    /** Determines equality between two Degrees.
     * @param obj The other object to compare this to. Always unequal if obj is not a Degrees.
     * @return true if values are identical; false otherwise.
     */
    @Override
    public boolean equals(Object obj) {
        if (this == obj)
            return true;

        if (obj == null)
            return false;

        if (getClass() != obj.getClass())
            return false;
        
        Degrees other = (Degrees) obj;
        return equals(other);
    }

    /** @return The value as a string. */
    @Override
    public String toString() {
        return Double.toString(raw);
    }
}