package frc.team5883.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.team5883.Converter;
import frc.team5883.robot.Robot;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;

import java.io.File;

public class PathReader extends Command {

    private Trajectory left;
    private Trajectory right;

    private double startTime;
    private int index;


    public double turnSensitivity;

    public PathReader(String leftFile, String rightFile, double turnSensitivity) {
        requires(Robot.driveTrain);
        left = Pathfinder.readFromCSV(new File(leftFile));
        right = Pathfinder.readFromCSV(new File(rightFile));
        this.turnSensitivity = turnSensitivity;
    }

    @Override
    protected void initialize() {

        startTime = Timer.getFPGATimestamp() * 1000.0;
//        Robot.driveTrain.changeToVelocity();
//
//        Robot.driveTrain.lDriveMaster.setProfile(1);
//        Robot.driveTrain.rDriveMaster.setProfile(1);
//
        Robot.driveTrain.resetGyro();
//        Robot.driveTrain.navX.zeroYaw();
//        Robot.driveTrain.navX.zeroYaw();
//        Robot.driveTrain.navX.zeroYaw();

    }

    @Override
    protected void execute() {

        index = ((int) Math.floor(((Timer.getFPGATimestamp() * 1000.0) - startTime) / 10));

        if (isFinished()) return;
        double leftVelo = left.segments[index].velocity;
        double rightVelo = right.segments[index].velocity;

        double gyro_heading = Robot.driveTrain.getAngle();// Assuming gyro angle is given in degrees
        double desired_heading = Pathfinder.r2d(left.segments[index].heading);
        double angle_difference = (desired_heading - gyro_heading) % 180;// Make sure to bound this from -180 to 180, otherwise you will get super large values
        System.out.printf("Desired Heading = %03.2f ; Gyro Heading = %03.2f ; Angle Difference = %03.2f ; Turn Sensitivity = %.4f \n", desired_heading, gyro_heading, angle_difference, turnSensitivity);
        double turn = turnSensitivity * angle_difference;
        Robot.driveTrain.driveSet(-Converter.ftPerSecondToNativeUnitsPer100Ms((rightVelo - turn)), -Converter.ftPerSecondToNativeUnitsPer100Ms((leftVelo + turn)));

    }

    @Override
    protected boolean isFinished() {
        return index + 1 >= left.segments.length || index + 1 >= right.segments.length;
    }

    protected void end() {
        Robot.driveTrain.stopDrive();
    }

}

