package frc.team5883.subsystems;

import frc.team5883.commands.drivetrain.Drive;
import frc.team5883.controllers.DrivetrainController;
import frc.team5883.motion.SpeedPID;
import frc.team5883.robot.Constants;
import frc.team5883.robot.Robot;
import frc.team5883.robot.RobotMap;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveTrain extends Subsystem {

    private final SpeedController speedController3 = RobotMap.driveTrainSpeedController3;
    private final SpeedController speedController0 = RobotMap.driveTrainSpeedController0;
    private final SpeedController speedController1 = RobotMap.driveTrainSpeedController1;
    private final SpeedController speedController2 = RobotMap.driveTrainSpeedController2;
    private final RobotDrive robotDrive41 = RobotMap.driveTrainRobotDrive41;
    public final Encoder encoderRight = new Encoder(18, 19);
    private final Encoder encoderLeft = new Encoder(16, 17, true);
    public ADXRS450_Gyro gyro = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);
    
    private Constants constants;
    private long oldTime = 0;
    private double speed = 0;
    private double powerForSpeed = 0; 
    private boolean onTarget;
    private SpeedPID speedPID;
    private double turn = getAngle()* 0.05;
    DrivetrainController controller;
    
    public DriveTrain() {
    	
        speedPID = new SpeedPID();
    	encoderRight.setMaxPeriod(.1);
    	encoderRight.setMinRate(10);
    	encoderRight.setDistancePerPulse(1);
    	encoderRight.setReverseDirection(false);
    	encoderRight.setSamplesToAverage(7);
    	encoderLeft.setMaxPeriod(.1);
    	encoderLeft.setMinRate(10);
    	encoderLeft.setDistancePerPulse(1);
    	encoderLeft.setSamplesToAverage(7);
    	constants = Constants.getConstants();
    	//SmartDashboard.putNumber("Dystans w metrach", getDistanceInMeters());

    }

    public void initDefaultCommand() {

    	setDefaultCommand(new Drive());
	}

    public double getDistanceInMeters(){
    	return encoderRight.getDistance()/4332;
    	
    }
    
    public void setSpeedRobot(double speed) {
        SmartDashboard.putNumber("Speed z Trapezu", speed);
        
        if (getActualSpeed() < speed) {
        	powerForSpeed += 0.01;
        }else if (getActualSpeed() >= speed) {
        	powerForSpeed -= 0.01;
        } else {
        	powerForSpeed = 0;
        }
        
        SmartDashboard.putNumber("Speed na silniki DriveTrain", speedPID.calculate(speed, getActualSpeed()));
        double turn = (gyro.getAngle()*constants.Gyro_kP)*(1/getActualSpeed());
        
        if(turn>constants.maxTurnValue) {
        	turn = constants.maxTurnValue;
        } else if(turn < -constants.maxTurnValue) {
        	turn = -constants.maxTurnValue;
        }
        
    	robotDrive41.arcadeDrive(speedPID.calculate(speed, getActualSpeed()), turn);
        //robotDrive41.arcadeDrive(powerForSpeed, gyro.getAngle()*constants.Gyro_kP);
    }

    
    public void driveStraight(double speed, double dystansMeter){
    	
    	double angle = gyro.getAngle();
    	robotDrive41.arcadeDrive(speed, angle*0.08);
    	
    }
    
    public void driveWithJoysticks(){
    	
    	//SmartDashboard.putNumber("encoderRight DriveTrain", encoderRight.getDistance());
    	long newTime = System.currentTimeMillis();
    	double deltaTime = newTime - oldTime;
    	double joySpeedValue = Robot.oi.getDriverJoystick().getRawAxis(1);
    	 //SmartDashboard.putNumber("encoderRight", getDistanceInMeters());
    	  
    	 //This value reach 100% power motor in 0,5s
    	 double speedAddicional = 0.1;
    	 speedAddicional = (deltaTime/50) * speedAddicional;
    	 
    	 if(Math.abs(joySpeedValue) < 0.2){
    		 joySpeedValue = 0;
    	 }

    	if(-joySpeedValue > speed){
    		SmartDashboard.putNumber("delta50", deltaTime);
    		speed = speed + speedAddicional;
    		oldTime = System.currentTimeMillis();
    	} else if (-joySpeedValue < speed){
    		speed = speed - speedAddicional; 
    		oldTime = System.currentTimeMillis();
    	} else {
    		speed = speed;
    		oldTime = System.currentTimeMillis();
    	 }
    	
    	SmartDashboard.putNumber("Aktualna predkosc robota", getActualSpeed());
    	 
    	 if (speed > 1){
    		 speed = 1;
    	 }
    	 else if
    		 (speed < -1){
    		speed = -1;
    	 }
    	 
    	 
    	 
    	 double turn = Robot.oi.getDriverJoystick().getRawAxis(4) * constants.DRIVERotacion_kP;
     	
    	 
    	 
    	 SmartDashboard.putData(encoderRight);
    	 SmartDashboard.putData(encoderLeft);

    	 
    	 
    	 if(Math.abs(turn) < 0.1){
    		 turn = 0;
    	 }
    	 
    	 if(speed < 0.2) {
    		 turn *= 2.33; 
    	 }
    	 robotDrive41.tankDrive(speed + turn, speed - turn);
    	 
    }
    
    public void updateAuto() {
    	SmartDashboard.putNumber("Gyro", getAngle());
    	
    	if (controller != null) {
    		onTarget = controller.update();
    		SmartDashboard.putNumber("Update", 10);
    	} else {
    		onTarget = false;
    	}
    	
    	//SmartDashboard.putBoolean("ONTARGET", onTarget);
    	
	}
    
    @SuppressWarnings("deprecation")
	public void stopDrive(){
    	robotDrive41.arcadeDrive(0, 0);
    }
    public void resetGyro(){
    	gyro.reset();
    	Timer.delay(0.2);
    }
    public void resetSpeed(){
    	speed = 0;
    }
    
    public void resetEncoder(){
    	encoderRight.reset();
    }
    
    public double getActualSpeed(){
    	double speedometer = encoderRight.getRate()/4332;
    	SmartDashboard.putNumber("Speedometer", speedometer);
    	return speedometer;
    }
    
    public void setTankDrive(double left, double right) {
    	robotDrive41.tankDrive(left, right);
    }
    public double getAngle() {
    	return gyro.getAngle();
    }

	public void setController(DrivetrainController controller) {
		this.controller = controller;	
	}
	
	public DrivetrainController getDrivetrainController() {
		return controller;
	}
	
	public void arcadeDrive(double leftSpeed, double rightSpeed) {
		speedController0.set(leftSpeed);
		speedController1.set(leftSpeed);
		speedController2.set(rightSpeed);
		speedController3.set(rightSpeed);

		}
	public boolean isOnTarget() {
		return onTarget;
	}
	
	}
	




