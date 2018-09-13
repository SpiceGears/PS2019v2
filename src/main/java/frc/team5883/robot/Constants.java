package frc.team5883.robot;


public class Constants {
	
	private static Constants instance;
	
	public static Constants getConstants() {		
		if ( instance == null ) {
				instance = new Constants();
		}
		return instance;
	}
	
	private String gameData = null;
	
	public double MaxSpeedOnScaledrive = 0.4;
	public static final double DRIVERotacion_kP = 0.3;
	public double maxTurnValue = 0.35;
	
	public double DriveProfile_kV = 0.119;
	public double DriveProfile_kA = 0;
	public double DriveProfile_kP = 1;
	public double DriveProfile_theta_kP = 0.03;
	public double ToleranceDriveError = 0.1;
	
	public double Gyro_kP = 0.19;
	
	public double SpeedPIDkP = 0.8;
	public double SpeedPIDkI = 0.3;
	public double SpeedPIDkD = 0.000;
	public double SpeedPIDkF = 0.1;
	
	public double TurnProfile_kV = 0.02;
	public double TurnProfile_kA = 0;
	public double TurnProfile_kP = -0.049;
	
	public double TurnPID_kP = 0.045; // 0.059
	public double TurnPID_kI = 0.00000015; // 0
	public double TurnPID_kD = 0.0045; // 0.0045
	
	public double MaxDriveVelocity = 8000;
	
	public double LeftDriveVelocityP = 0.3;
	public double LeftDriveVelocityD = 0;
	public double LeftDriveVelocityF = 1023.0 / MaxDriveVelocity;
	
	public double RightDriveVelocityP = 0.3;
	public double RightDriveVelocityD = 0;
	public double RightDriveVelocityF = 1023.0 / MaxDriveVelocity;
	
	public double MaxDriveVelocityLow = 3000;
	
	public double LeftDriveVelocityPLow = 0;
	public double LeftDriveVelocityDLow = 0;
	public double LeftDriveVelocityFLow = 1023.0 / MaxDriveVelocityLow;

	public double RightDriveVelocityPLow = 0;
	public double RightDriveVelocityDLow = 0;
	public double RightDriveVelocityFLow = 1023.0 / MaxDriveVelocityLow;
	
	public double DriveRampRate = 120;
	
	
	public double PivotTurnPID_kP = 0.003; //0.008
	public double PivotTurnPID_kI = 0.0002; //0.0003
	public double PivotTurnPID_kD = 0.003; //0.004

	public double AllowedDriveError = 0.25; // 0.25
	public double AllowedTurnError = 1.8;
	public double AllowedTurnPIDError = 1.3;
	public double AllowedPivotTurnError = 4.0;

	//Elevator
		public double elevatorIntake = 0;
		public double elevatorSwitch = 0.40+ 0.05;
		public double elevatorScale = 0.71 + 0.07;
		
		public double elevator_kP = 1;
		public double elevator_kI = 0.01;
		public double elevator_kD = 71;
		
		public double allowedElevatorError = 0.02;
		public double ElevatorProfile_kV = 0.5;
		public double ElevatorProfile_kA = 0;
		public double ElevatorProfile_kP = 1;
		public double ElevatorProfile_theta_kP = 0.03;
		
		public double ElevatorProfile_MaxV = 2;
		public double ElevatorProfile_MaxA = 0.7;
		
		public double debug = 0;
		public double climbingPrepare = 1000;
		public double climbing = 2000;
		
		
		public double turnTrapezoidalMultiply = 1.3;

		public String getGameData() {
			return gameData;
		}
		public void setGameData(String gameData) {
			this.gameData = gameData;
		}

}