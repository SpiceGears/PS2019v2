package frc.team5883.commands.drivetrain;

import edu.wpi.first.wpilibj.command.Command;
import frc.team5883.controllers.ProfileDriveController;
import frc.team5883.motion.TrapezoidalMotionProfile;
import frc.team5883.robot.Constants;
import frc.team5883.robot.Robot;

public class DriveDistance extends Command {

	TrapezoidalMotionProfile profile;
	double vf = 0;
	double distance;
	Constants constants;
	double driveError = 0;
	double distansToTrapezoid;
	
	public DriveDistance(double distance, double maxV, double maxAcc, double driveError) {
		requires(Robot.driveTrain);
		if(distance <0) {
			driveError *= -1;
		}
		distansToTrapezoid = distance + driveError;
		
		
		profile = new TrapezoidalMotionProfile(distansToTrapezoid, maxV, maxAcc);
		this.distance = distance;
		this.driveError = driveError;
	}
	
	public DriveDistance(double distance, double maxV, double maxAcc, double vi, double vf, double driveError) {
		requires(Robot.driveTrain);
		if(distance <0) {
			driveError *= -1;
		}
		distansToTrapezoid = distance + driveError;
		
		
		profile = new TrapezoidalMotionProfile(distansToTrapezoid, maxV, maxAcc, vi, vf);
		this.vf = vf;
		this.distance = distance;
	}
	
	@Override
	protected void initialize() {
		Robot.driveTrain.resetEncoder();
		ProfileDriveController controller = new ProfileDriveController(profile, Robot.driveTrain.getAngle());
		constants = Constants.getConstants();
		Robot.driveTrain.setController(controller);
	}

	@Override
	protected boolean isFinished() {
		
		return Math.abs(Robot.driveTrain.getDistanceInMeters()) >= Math.abs(distance)-(constants.ToleranceDriveError);
	}
	
	protected void end() {
			Robot.driveTrain.setTankDrive(0, 0);
			Robot.driveTrain.setController(null);
	}
	
	public DriveDistance timeout(double t) {
		setTimeout(t);
		return this;
	}
}
