package frc.team5883.controllers;


import edu.wpi.first.wpilibj.Timer;
import frc.team5883.motion.ProfilePoint;
import frc.team5883.motion.TrapezoidalMotionProfile;
import frc.team5883.robot.Constants;
import frc.team5883.robot.Robot;

public class LinearAngularFollowerController implements DrivetrainController {
	Constants constants;
	
	double startT;
	TrapezoidalMotionProfile linear;
	TrapezoidalMotionProfile angular;
	double startAngle = 0;
	
	public LinearAngularFollowerController(TrapezoidalMotionProfile linear, TrapezoidalMotionProfile angular) {
		constants = Constants.getConstants();
		
		this.linear = linear;
		this.angular = angular;
		refreshConstants();
		start();
	}
	
	
	/*
	 * Configure this controller to begin following the profile, making t=0 the current moment
	 */
	public void start() {
		startT = Timer.getFPGATimestamp();
		startAngle = Robot.driveTrain.getAngle();
		Robot.driveTrain.resetEncoder();
	}

	
	double linearSetpoint, angularSetpoint;
	double linearActual, angularActual;
	double linearError, angularError;
	@Override
	public boolean update() {
		double t = Timer.getFPGATimestamp() - startT;
		ProfilePoint linearPoint = linear.getAtTime(t);
		ProfilePoint angularPoint = angular.getAtTime(t);
		
		double linearFeedforward = (linearPoint.vel * constants.DriveProfile_kV) + (linearPoint.acc * constants.DriveProfile_kA);

		linearActual = Robot.driveTrain.getDistanceInMeters();
		linearError = linearPoint.pos-linearActual;
		linearSetpoint = linearPoint.pos;
		double linearComponent = (linearError * constants.DriveProfile_kP) +  linearFeedforward;
		
		if ( linearComponent > 1.0 )
			linearComponent = 1.0;
		else if ( linearComponent < -1.0 )
			linearComponent = -1.0;
		
		double angularFeedforward = (angularPoint.vel * constants.TurnProfile_kV) + (angularPoint.acc * constants.TurnProfile_kA);
		angularActual = (Robot.driveTrain.getAngle()-startAngle);
		angularError = angularActual - angularPoint.pos;
		angularSetpoint = angularPoint.pos;
		double angularComponent = (angularError * constants.TurnProfile_kP) + angularFeedforward;
		
		Robot.driveTrain.setTankDrive(linearComponent+angularComponent, linearComponent-angularComponent);
		
		
		return t >= linear.getDuration() &&  
			   t >= angular.getDuration() &&
			  /* angularError < constants.AllowedTurnError && */
			   linearError < constants.AllowedDriveError;
	}

	@Override
	public double getLinearError() {
		return linearError;
	}

	@Override
	public double getLinearActual() {
		return linearActual;
	}

	@Override
	public double getLinearSetpoint() {
		return linearSetpoint;
	}

	@Override
	public void reset() {
		start();
	}

	@Override
	public void refreshConstants() {
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
