package frc.robot.Exceptions;

public class ValueConstraintException extends RuntimeException {

    public ValueConstraintException(int lowerBounds, int upperBounds) {
        super(String.format("Value must be between %d and %d.", lowerBounds, upperBounds));
    }
    public ValueConstraintException(double lowerBounds, double upperBounds) {
        super(String.format("Value must be between %f and %f.", lowerBounds, upperBounds));
    }
}