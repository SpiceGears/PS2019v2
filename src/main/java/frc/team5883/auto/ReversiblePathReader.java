package frc.team5883.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.team5883.Converter;
import frc.team5883.robot.Robot;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;

import java.io.File;

public class ReversiblePathReader extends Command {


    private Trajectory left;
    private Trajectory right;

    private double startTime;
    private int index;


    private double turnSensitivity;

    public enum Direction {
        FORWARD, BACKWARD;
    }

    private Direction direction;

    public ReversiblePathReader(String leftFile, String rightFile, double turnSensitivity, Direction direction) {
        requires(Robot.driveTrain);
        left = Pathfinder.readFromCSV(new File(leftFile));
        right = Pathfinder.readFromCSV(new File(rightFile));
        this.turnSensitivity = turnSensitivity;
        this.direction = direction;
    }

    public ReversiblePathReader(String baseFilePath, double turnSensitivity, Direction direction) {

        this("/home/lvuser/paths/" + baseFilePath + "_left_detailed.csv", "/home/lvuser/paths/" + baseFilePath + "_right_detailed.csv", turnSensitivity, direction);
        requires(Robot.driveTrain);

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

        switch (direction) {
            case FORWARD:
                index = ((int) Math.floor(((Timer.getFPGATimestamp() * 1000.0) - startTime) / 10));
                break;

            case BACKWARD:
                index = left.segments.length - ((int) Math.floor(((Timer.getFPGATimestamp() * 1000.0) - startTime) / 10)) - 1;
                break;
        }

        if (isFinished()) return;

        double leftVelo = left.segments[index].velocity;
        double rightVelo = right.segments[index].velocity;

        double gyro_heading = Robot.driveTrain.getAngle();// Assuming gyro angle is given in degrees
        double desired_heading = -Pathfinder.r2d(left.segments[index].heading);
        double angle_difference = Pathfinder.boundHalfDegrees(desired_heading - gyro_heading);// Make sure to bound this from -180 to 180, otherwise you will get super large values
//        System.out.printf("Desired Heading = %03.2f ; Gyro Heading = %03.2f ; Angle Difference = %03.2f ; Turn Sensitivity = %.4f \n", desired_heading, gyro_heading, angle_difference, turnSensitivity);

        double turn = turnSensitivity * angle_difference;
        System.out.printf("Turn = %03.2f ; Left Velocity = %03.2f ; Right Velocity = %03.2f ; \n", turn, leftVelo, rightVelo);

        switch (direction) {
            case FORWARD:
                //Robot.driveTrain.scaledDriveSet(-Converter.ftPerSecondToNativeUnitsPer100Ms((rightVelo)) + turn, -Converter.ftPerSecondToNativeUnitsPer100Ms((leftVelo)) - turn);
                break;

            case BACKWARD:
                //Robot.driveTrain.scaledDriveSet(Converter.ftPerSecondToNativeUnitsPer100Ms((leftVelo)) + turn, Converter.ftPerSecondToNativeUnitsPer100Ms((rightVelo)) - turn);
                break;
        }

    }

    @Override
    protected boolean isFinished() {
        switch (direction) {
            case FORWARD:
                return index + 1 >= left.segments.length || index + 1 >= right.segments.length;

            case BACKWARD:
                return index - 1 <= 0;
        }
        return false;
    }

    protected void end() {

        //Robot.driveTrain.changeToPercentVBus();
    }

}
