package frc.team5883.auto;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team5883.Converter;
import frc.team5883.robot.Robot;

public class RobotPose {

    private Notifier notifier;

    private double prev_x;
    private double prev_y;

    private double prev_dist;

    public RobotPose() {
        notifier = new Notifier(() -> {
            double currentAngle = Robot.driveTrain.getAngle();
            double curentDistance = Robot.driveTrain.getAvgPosition() - prev_dist;

            prev_dist = Robot.driveTrain.getAvgPosition();

            double x = curentDistance * Math.sin(Math.toRadians(currentAngle));
            double y = curentDistance * Math.cos(Math.toRadians(currentAngle));

            prev_x += x;
            prev_y += y;
            SmartDashboard.putNumber("Current X", Converter.encTicksToIn(prev_x));
            SmartDashboard.putNumber("Current Y", Converter.encTicksToIn(prev_y));
            SmartDashboard.putNumber("Actual Path Heading", Math.toRadians(Robot.driveTrain.getAngle()));

        });
    }

    public void startNotifier(double period) {
        Robot.driveTrain.resetEncoder();
        Robot.driveTrain.resetGyro();
        notifier.startPeriodic(period);
    }


}
