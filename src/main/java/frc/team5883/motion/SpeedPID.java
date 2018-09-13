package frc.team5883.motion;

import edu.wpi.first.wpilibj.Timer;
import frc.team5883.robot.Constants;

public class SpeedPID {
	
	double kP;
	double kI;
	double kD; 
	double kF;
	double setSpeed;
	double error;
	double accualSpeed;
	
	double lastT;
	double errSum;
	double prevError;
	
	boolean reset;

	double maxOut = 1;
	double minOut = -1;
	double maxI = 0;
	double minI = 0;
	
	Constants constants;
	
	public SpeedPID() {
		constants = Constants.getConstants();
		this.kP = constants.SpeedPIDkP;
		this.kI = constants.SpeedPIDkI;
		this.kD = constants.SpeedPIDkD;
		this.kF = constants.SpeedPIDkF;
		
		if ( kI == 0 )
			maxI = 0;
		else
			maxI = 1/kI;
		minI = -maxI;
		
		reset();
	}

	public void setSetpoint(double setSpeed) {
		this.setSpeed = setSpeed;
	}
	
	public double getError() {
		if ( reset )
			return Double.MAX_VALUE;
		return error;
	}
	
	public double getInput() {
		return accualSpeed;
	}
	
	public void reset() {
		reset = true;
	}
	
	public double getSetpoint() {
		return setSpeed;
	}
	
	public void setOutputConstraints(double max, double min){
		maxOut = max;
		minOut = min;
	}
	
	public double calculate(double setSpeed, double accualSpeed) {
		double now = Timer.getFPGATimestamp();
		double dt = now-lastT;
		lastT = now;
		return calculate(dt, setSpeed, accualSpeed);
	}
	
	public double calculate(double dT, double setSpeed, double accualSpeed) {
		this.accualSpeed = accualSpeed;
		
		error = setSpeed - accualSpeed;
		
		errSum += error*dT;
		
		if ( errSum > maxI )
			errSum = maxI;
		else if ( errSum < minI )
			errSum = minI;

		double deltaPos = error-prevError;
		prevError = error;

		if ( reset ) {
			deltaPos = 0;
			lastT = Timer.getFPGATimestamp();
			prevError = 0;
			errSum = 0;
			reset = false;
		}
		
		double out = (kP*error) + (kI*errSum) + (kD*(deltaPos/dT)) + (kF*setSpeed);
		
		if ( out > maxOut )
			out = maxOut;
		else if ( out < minOut )
			out = minOut;
		
		return out;
	}
}
