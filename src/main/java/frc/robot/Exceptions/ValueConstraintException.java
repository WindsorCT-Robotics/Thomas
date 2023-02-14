package frc.robot.Exceptions;

public class ValueConstraintException extends RuntimeException {

    public ValueConstraintException(int lowerBounds, int upperBounds, int actualValue) {
        super(String.format("Value must be between %d and %d. Actual Value: %d", lowerBounds, upperBounds, actualValue));
    }

    public ValueConstraintException(double lowerBounds, double upperBounds, double actualValue) {
        super(String.format("Value must be between %f and %f. Actual Value: %f", lowerBounds, upperBounds, actualValue));
    }
}