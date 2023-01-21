package frc.robot;

/**
 * Class that organizes gains when assigning values to slots
 */
public class Gains {
	public final double kP;
	public final double kI;
	public final double kD;
	public final double kF;
	public final int integralZone;
	public final double peakOutput;
	
	public Gains(double kP, double kI, double kD, double kF, int kIzone, double kPeakOutput){
		this.kP = kP;
		this.kI = kI;
		this.kD = kD;
		this.kF = kF;
		this.integralZone = kIzone;
		this.peakOutput = kPeakOutput;
	}
}
