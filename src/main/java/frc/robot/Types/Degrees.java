package frc.robot.Types;

public class Degrees {
    // TODO: Determine min and max for degrees if possible
    private final double raw;

    public Degrees (double value) {
        raw = value;
    }

    /**
     * @return
     */
    public double getValue () {
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
        Degrees other = (Degrees) obj;
        if (Double.doubleToLongBits(raw) != Double.doubleToLongBits(other.raw))
            return false;
        return true;
    }

    @Override
    public String toString() {
        return Double.toString(raw);
    }
}