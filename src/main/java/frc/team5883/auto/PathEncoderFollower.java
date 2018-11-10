package frc.team5883.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team5883.robot.Constants;
import frc.team5883.robot.Robot;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.followers.EncoderFollower;

import java.io.File;

public class PathEncoderFollower extends Command {

    EncoderFollower leftFollow;
    EncoderFollower rightFollow;

    private Trajectory leftTraj;
    private Trajectory rightTraj;

    private double startTime;
    private int index;


    public double turnSensitivity;

    public PathEncoderFollower(String leftFile, String rightFile, double turnSensitivity) {
        requires(Robot.driveTrain);
        leftTraj = Pathfinder.readFromCSV(new File(leftFile));
        rightTraj = Pathfinder.readFromCSV(new File(rightFile));

        leftFollow = new EncoderFollower(leftTraj);
        rightFollow = new EncoderFollower(rightTraj);

        this.turnSensitivity = turnSensitivity;
    }

    public PathEncoderFollower(String baseFilePath, double turnSensitivity) {
        this("/home/lvuser/paths/" + baseFilePath + "_left.csv", "/home/lvuser/paths/" + baseFilePath + "_right.csv", turnSensitivity);
        String leftFile = "/home/lvuser/paths/" + baseFilePath + "_left.csv";
        String rightFile = "/home/lvuser/paths/" + baseFilePath + "_right.csv";

        requires(Robot.driveTrain);

    }


    @Override
    protected void initialize() {
        //Robot.driveTrain.changeToPercentVBus();

        leftFollow.configureEncoder(Robot.driveTrain.getLPosition(), Constants.encTickPerRev, Constants.wheelDiameterMetres);
        leftFollow.configurePIDVA(0.1, 0.04, 0, 1 / 10, 0);
        rightFollow.configureEncoder(Robot.driveTrain.getRPosition(), Constants.encTickPerRev, Constants.wheelDiameterMetres);
        rightFollow.configurePIDVA(0.1, 0.04, 0, 1 / 10, 0);


        startTime = Timer.getFPGATimestamp() * 1000.0;
//        Robot.twilightDrive.lDriveMaster.setProfile(1);
//        Robot.twilightDrive.rDriveMaster.setProfile(1);
        Robot.driveTrain.resetEncoder();
        Robot.driveTrain.resetGyro();
        Robot.driveTrain.resetGyro();
        Robot.driveTrain.resetGyro();
        Robot.driveTrain.resetGyro();
        SmartDashboard.putNumber("TEST", 2);
    }

    private int oldIndex = 0;
    @Override
    protected void execute() {
        index = ((int) Math.floor(((Timer.getFPGATimestamp() * 1000.0) - startTime) / 50));
        if(index > oldIndex){
            oldIndex = index;
        } else {
            Timer.delay(0.01);
        }
        //if (isFinished()) return;


        double leftSet = leftFollow.calculate(Robot.driveTrain.getLPosition());
        double rightSet = rightFollow.calculate(Robot.driveTrain.getRPosition());
        //SmartDashboard.putNumber("RIGHT VELOCITY ENCODER FOLLOWER", Robot.driveTrain.getRPosition());

        SmartDashboard.putNumber("Left Encoder", Robot.driveTrain.getLPosition());
        SmartDashboard.putNumber("Right Encoder", Robot.driveTrain.getRPosition());

        SmartDashboard.putNumber("index", index);
        SmartDashboard.putNumber("leftTraj.segments.length", leftTraj.segments.length);

        SmartDashboard.putNumber("Left Calculate", leftSet);
        SmartDashboard.putNumber("Right Calculate", rightSet);


//        double gyro_heading = Robot.driveTrain.getAngle();// Assuming gyro angle is given in degrees
//        double desired_heading = Pathfinder.r2d(leftTraj.segments[index].heading);
 //       double angle_difference = Pathfinder.boundHalfDegrees(desired_heading - gyro_heading);// Make sure to bound this from -180 to 180, otherwise you will get super large values
//        System.out.printf("Desired Heading = %03.2f ; Gyro Heading = %03.2f ; Angle Difference = %03.2f ; Turn Sensitivity = %.4f \n", desired_heading, gyro_heading, angle_difference, turnSensitivity);
 //       double turn = turnSensitivity * angle_difference;

        Robot.driveTrain.driveSet(leftSet, rightSet);

    }

    @Override
    protected boolean isFinished() {
        return index + 1 >= leftTraj.segments.length || index + 1 >= rightTraj.segments.length;
        //return false;
    }

    protected void end() {

        Robot.driveTrain.stopDrive();
    }

}
