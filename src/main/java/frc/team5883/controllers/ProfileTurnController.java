package frc.team5883.controllers;


import edu.wpi.first.wpilibj.Timer;
import frc.team5883.motion.ProfilePoint;
import frc.team5883.motion.TrapezoidalMotionProfile;
import frc.team5883.robot.Constants;
import frc.team5883.robot.Robot;

public class ProfileTurnController implements DrivetrainController {
	Constants constants;
	
	double startAngle;
	double startT;
	TrapezoidalMotionProfile profile;
	
	double angularError, angularActual, angularSetpoint;
	
	/*
	 * ProfileTurnController constructor
	 * 
	 * @param profile The TrapezoidalMotionProfile to follow
	 * @param theta The angle to maintain while driving straight
	 */
	public ProfileTurnController(TrapezoidalMotionProfile profile) {
		constants = Constants.getConstants();
		
		this.profile = profile;
		refreshConstants();
		start();
	}
	
	
	/*
	 * Configure this controller to begin following the profile, making t=0 the current moment
	 */
	public void start() {
		startT = Timer.getFPGATimestamp();
		startAngle = Robot.driveTrain.getAngle();
	}

	@Override
	public boolean update() {
		double t = Timer.getFPGATimestamp() - startT;
		ProfilePoint point = profile.getAtTime(t);
		
		angularActual = Robot.driveTrain.getAngle();
		angularSetpoint = point.pos;
		
		double feedforward = (point.vel * constants.TurnProfile_kV) + (point.acc * constants.TurnProfile_kA);
		angularError = (angularActual-startAngle) - point.pos;
		double output = (angularError * constants.TurnProfile_kP) + feedforward;
		Robot.driveTrain.setTankDrive(output, -output);

		return t >= profile.getDuration();
	}


	@Override
	public void reset() {
		start();
	}

	@Override
	public void refreshConstants() {
		
	}


	@Override
	public double getLinearError() {
		return 0;
	}


	@Override
	public double getLinearActual() {
		return 0;
	}


	@Override
	public double getLinearSetpoint() {
		return 0;
	}


	@Override
	public double getAngularError() {
		return angularError;
	}


	@Override
	public double getAngularActual() {
		return angularActual;
	}


	@Override
	public double getAngularSetpoint() {
		return angularSetpoint;
	}
}
