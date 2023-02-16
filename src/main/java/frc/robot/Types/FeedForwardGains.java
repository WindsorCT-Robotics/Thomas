package frc.robot.Types;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class FeedForwardGains {
    private final double staticGain;
    private final double velocityGain;
    private final double accelerationGain;

    public FeedForwardGains(double staticGain, double velocityGain, double accelerationGain) {
        this.staticGain = staticGain;
        this.velocityGain = velocityGain;
        this.accelerationGain = accelerationGain;
    }

    @Override
    public int hashCode() {
        final int prime = 31;
        int result = 1;
        long temp;
        temp = Double.doubleToLongBits(staticGain);
        result = prime * result + (int) (temp ^ (temp >>> 32));
        temp = Double.doubleToLongBits(velocityGain);
        result = prime * result + (int) (temp ^ (temp >>> 32));
        temp = Double.doubleToLongBits(accelerationGain);
        result = prime * result + (int) (temp ^ (temp >>> 32));
        return result;
    }

    public boolean equals(FeedForwardGains other) {
        return Double.doubleToLongBits(accelerationGain) == Double.doubleToLongBits(other.accelerationGain)
                && Double.doubleToLongBits(staticGain) == Double.doubleToLongBits(other.staticGain)
                && Double.doubleToLongBits(velocityGain) == Double.doubleToLongBits(other.velocityGain);
    }

    @Override
    public boolean equals(Object obj) {
        if (this == obj)
            return true;
        if (obj == null)
            return false;
        if (getClass() != obj.getClass())
            return false;

        FeedForwardGains other = (FeedForwardGains) obj;
        return equals(other);
    }

    public SimpleMotorFeedforward toSimpleMotorFeedforward() {
        return new SimpleMotorFeedforward(staticGain, velocityGain, accelerationGain);
    }

    public double getStaticGain() {
        return staticGain;
    }

    public double getVelocityGain() {
        return velocityGain;
    }

    public double getAccelerationGain() {
        return accelerationGain;
    }
}
