
package frc.team5883.robot;

import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.VictorSP;

import edu.wpi.first.wpilibj.livewindow.LiveWindow;

public class RobotMap {

	public static SpeedController driveTrainSpeedController3;
    public static SpeedController driveTrainSpeedController0;
    public static SpeedController driveTrainSpeedController1;
    public static SpeedController driveTrainSpeedController2;
    public static RobotDrive driveTrainRobotDrive41;

    public static void init() {

    	driveTrainSpeedController3 = new VictorSP(1);
        LiveWindow.addActuator("DriveTrain", "Speed Controller 3", (VictorSP) driveTrainSpeedController3);
        
        driveTrainSpeedController0 = new VictorSP(2);
        LiveWindow.addActuator("DriveTrain", "Speed Controller 0", (VictorSP) driveTrainSpeedController0);
        
        driveTrainSpeedController1 = new VictorSP(3);
        LiveWindow.addActuator("DriveTrain", "Speed Controller 1", (VictorSP) driveTrainSpeedController1);
        
        driveTrainSpeedController2 = new VictorSP(0);
        LiveWindow.addActuator("DriveTrain", "Speed Controller 2", (VictorSP) driveTrainSpeedController2);
        
        
        driveTrainRobotDrive41 = new RobotDrive(driveTrainSpeedController0, driveTrainSpeedController1,
        driveTrainSpeedController2, driveTrainSpeedController3);

    }
}
