package frc.team5883.robot;


import edu.wpi.first.wpilibj.Joystick;

public class OI {

	private Joystick driver;
	private Joystick operator;

    public OI() {
		driver = new Joystick(0);
		operator = new Joystick(1);
    }

	public Joystick getDriverJoystick() {
		return driver;
	}

	public Joystick getOperatorJoystick() {
		return operator;
	}

}

