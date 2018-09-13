package frc.team5883.commands.drivetrain;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team5883.robot.Robot;

public class SpeedCommand extends Command {
    double setPoint = 0;
    double speed = 0;
    double maxSpeed = 0;
    public SpeedCommand(double setPoint, double speed) {
    	requires(Robot.driveTrain);
    	this.setPoint = setPoint;
    	this.speed = speed;
    	this.maxSpeed = speed;
    }

    protected void initialize() {
    	Robot.driveTrain.resetEncoder();
    	Robot.driveTrain.resetGyro();
    }

    protected void execute() {
    	if(Robot.driveTrain.getDistanceInMeters() >= setPoint-0.1*setPoint) {
    		SmartDashboard.putNumber("Distance", Robot.driveTrain.getDistanceInMeters());
    	    speed = (setPoint-Robot.driveTrain.getDistanceInMeters());
    	    if(speed <= 0) {
    	    	speed = 0;
    	    }
    	    else if(speed > maxSpeed){
    	    	speed = maxSpeed;
    	    }
    	}
    	Robot.driveTrain.setSpeedRobot(speed);
    }

    protected boolean isFinished() {
    	boolean isDrove = Robot.driveTrain.getDistanceInMeters() >= setPoint;
        return isDrove;
    }

    protected void end() {
    	Robot.driveTrain.stopDrive();
    }

    protected void interrupted() {
    	end();
    }
}
