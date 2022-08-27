package org.usfirst.frc.team3042.robot;

import org.usfirst.frc.team3042.lib.Log;
import org.usfirst.frc.team3042.robot.commands.Drivetrain_XStance;
import org.usfirst.frc.team3042.robot.subsystems.Drivetrain;

import edu.wpi.first.wpilibj2.command.InstantCommand;

/** OI ************************************************************************
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot. */
public class OI {	
	/** Configuration Constants ***********************************************/
	private static final int USB_GAMEPAD = RobotMap.USB_GAMEPAD;
	private static final int USB_JOY_LEFT = RobotMap.USB_JOYSTICK_LEFT;
	private static final int USB_JOY_RIGHT = RobotMap.USB_JOYSTICK_RIGHT;
	private static final int JOYSTICK_X_AXIS = Gamepad.JOY_X_AXIS;
	private static final int JOYSTICK_Y_AXIS = Gamepad.JOY_Y_AXIS;
	private static final int JOYSTICK_Z_AXIS = Gamepad.JOY_Z_AXIS;
	
	/** Instance Variables ****************************************************/
	Log log = new Log(RobotMap.LOG_OI, "OI");
	public static Gamepad gamepad, joyLeft, joyRight;
	public static boolean isLowScale = false;

	Drivetrain drivetrain = Robot.drivetrain;

	int driveAxisX, driveAxisY, driveAxisZ;

	/** OI ********************************************************************
	 * Assign commands to the buttons and triggers*/
	public OI() {
		log.add("OI Constructor", Log.Level.TRACE);
		
		gamepad = new Gamepad(USB_GAMEPAD);
		
		// Setup Driving Controls //
		joyLeft = new Gamepad(USB_JOY_LEFT);
		joyRight = new Gamepad(USB_JOY_RIGHT);
		
		driveAxisX = JOYSTICK_X_AXIS;
		driveAxisY = JOYSTICK_Y_AXIS;
		driveAxisZ = JOYSTICK_Z_AXIS;

		joyLeft.button1.whenPressed(new InstantCommand(drivetrain::zeroGyro, drivetrain)); // Zero the gyro, this is helpful for field-oriented driving
		
		gamepad.X.whenPressed(new Drivetrain_XStance());
	}
	
	/** Access to the driving axes values *****************************
	 * A negative can be added to make pushing forward positive/negative. */
	public double getXSpeed() {
		double joystickValue = joyRight.getRawAxis(driveAxisY);
		if (Math.abs(joystickValue) < 0.01) { // This is our deadband
			return 0.0;
		}
		else {
			return joystickValue * RobotMap.kPhysicalMaxSpeedMetersPerSecond * -1;
		}	
	}
	public double getYSpeed() {
		double joystickValue = joyRight.getRawAxis(driveAxisX);
		if (Math.abs(joystickValue) < 0.01) { // This is our deadband
			return 0.0;
		}
		else {
			return joystickValue * RobotMap.kPhysicalMaxSpeedMetersPerSecond * -1;
		}	
	}
	public double getZSpeed() {
		double joystickValue = joyLeft.getRawAxis(driveAxisX);
		if (Math.abs(joystickValue) < 0.01) { // This is our deadband
			return 0.0;
		}
		else {
			return joystickValue * RobotMap.kPhysicalMaxSpeedMetersPerSecond;
		}	
	}	
}