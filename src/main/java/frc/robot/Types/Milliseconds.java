package frc.robot.Types;

public class Milliseconds {
    private final int rawValue;

    public Milliseconds(int value) {
        rawValue = value;
    }

    public int getMilliseconds() {
        return rawValue;
    }

    @Override
    public int hashCode() {
        final int prime = 31;
        int result = 1;
        result = prime * result + rawValue;
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
        Milliseconds other = (Milliseconds) obj;
        if (rawValue != other.rawValue)
            return false;
        return true;
    }

    @Override
    public String toString() {
        return Integer.toString(rawValue);
    }
}
