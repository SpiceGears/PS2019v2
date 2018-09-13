package frc.team5883.commands.drivetrain;


import frc.team5883.controllers.ProfileTurnController;
import frc.team5883.motion.TrapezoidalMotionProfile;
import frc.team5883.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ProfileTurnCommand extends Command {


	TrapezoidalMotionProfile profile;
	
	public ProfileTurnCommand(double distance, double maxV, double maxAcc) {
		requires(Robot.driveTrain);
		profile = new TrapezoidalMotionProfile(distance, maxV, maxAcc);
	}
	
	@Override
	protected void initialize() {
		Robot.driveTrain.resetGyro();
		ProfileTurnController controller = new ProfileTurnController(profile);
		
		Robot.driveTrain.setController(controller);
	}
	
	@Override
	protected boolean isFinished() {
		return Robot.driveTrain.getDrivetrainController().update() || isTimedOut();
	}
	
	protected void end() {
		Robot.driveTrain.setController(null);
		Robot.driveTrain.setTankDrive(0, 0);
	}
	
	public ProfileTurnCommand timeout(double t) {
		setTimeout(t);
		return this;
	}
}
