package frc.team5883.commands.drivetrain;

import edu.wpi.first.wpilibj.command.Command;
import frc.team5883.controllers.LinearAngularFollowerController;
import frc.team5883.motion.TrapezoidalMotionProfile;
import frc.team5883.robot.Robot;

public class DriveTwoProfile extends Command {

	TrapezoidalMotionProfile linear;
	TrapezoidalMotionProfile angular;
	
	LinearAngularFollowerController controller;
	
	public DriveTwoProfile(TrapezoidalMotionProfile linear, TrapezoidalMotionProfile angular) {
		requires(Robot.driveTrain);
		this.linear = linear;
		this.angular = angular;
	}
	
	public DriveTwoProfile(TrapezoidalMotionProfile linear, TrapezoidalMotionProfile angular, double timeout) {
		this(linear, angular);
		setTimeout(timeout);
	}
	
	@Override
	protected void initialize() {
		Robot.driveTrain.resetEncoder();
		controller = new LinearAngularFollowerController(linear, angular);
		
		Robot.driveTrain.setController(controller);
	}
	
	@Override
	protected boolean isFinished() {
		return controller.update() || isTimedOut();
	}
	
	protected void end() {
		Robot.driveTrain.setController(null);
		Robot.driveTrain.setTankDrive(0, 0);
	}
	
	public DriveTwoProfile timeout(double t) {
		setTimeout(t);
		return this;
	}
}
