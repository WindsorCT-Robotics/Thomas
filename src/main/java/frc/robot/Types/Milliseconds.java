package frc.robot.Types;

/** Represents a value in milliseconds. */
public class Milliseconds {
    private final int rawValue;

    /** Creates a new value in milliseconds. */
    public Milliseconds(int value) {
        rawValue = value;
    }

    /** @return The value as a signed integer. */
    public int getMilliseconds() {
        return rawValue;
    }

    /** Generates a hash code based on the value. */
    @Override
    public int hashCode() {
        final int prime = 31;
        int result = 1;
        result = prime * result + rawValue;
        return result;
    }

    public boolean equals(Milliseconds other) {
        return rawValue == other.rawValue;
    }

    /**
     * Determines equality between two values in milliseconds.
     * 
     * @param obj The object to compare. Always unequal if obj is not a Milliseconds
     *            instance.
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

        Milliseconds other = (Milliseconds) obj;
        return equals(other);
    }

    /**
     * @returns the value represented as a string.
     */
    @Override
    public String toString() {
        return Integer.toString(rawValue);
    }
}
