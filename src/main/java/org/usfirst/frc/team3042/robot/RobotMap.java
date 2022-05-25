package org.usfirst.frc.team3042.robot;

import org.usfirst.frc.team3042.lib.Log;

import edu.wpi.first.math.util.Units;

/** RobotMap ******************************************************************
 * This is the robot configuration file. */
public class RobotMap {	
	/** Robot Size Parameters **************************************************/
	public static final double TRACK_WIDTH = 0.57785; // Distance between centers of right and left wheels on robot (in meters) // TODO: Measure the actual value for this!
    public static final double WHEEL_BASE = 0.517525; // Distance between centers of front and back wheels on robot (in meters) // TODO: Measure the actual value for this!

	/** CAN ID numbers ********************************************************/
	public static final int kFrontLeftDriveMotorPort = 1; //TODO: Determine the actual CAN ID of this motor
	public static final int kFrontLeftTurningMotorPort = 2; //TODO: Determine the actual CAN ID of this motor
	public static final int kFrontRightDriveMotorPort = 3; //TODO: Determine the actual CAN ID of this motor
    public static final int kFrontRightTurningMotorPort = 4; //TODO: Determine the actual CAN ID of this motor
	public static final int kBackLeftDriveMotorPort = 5; //TODO: Determine the actual CAN ID of this motor
    public static final int kBackLeftTurningMotorPort = 6; //TODO: Determine the actual CAN ID of this motor
	public static final int kBackRightDriveMotorPort = 7; //TODO: Determine the actual CAN ID of this motor
    public static final int kBackRightTurningMotorPort = 8; //TODO: Determine the actual CAN ID of this motor
	public static final int kFrontLeftDriveAbsoluteEncoderPort = 9; //TODO: Determine the actual CAN ID of this CANcoder
	public static final int kFrontRightDriveAbsoluteEncoderPort = 10; //TODO: Determine the actual CAN ID of this CANcoder
	public static final int kBackLeftDriveAbsoluteEncoderPort = 11; //TODO: Determine the actual CAN ID of this CANcoder
	public static final int kBackRightDriveAbsoluteEncoderPort = 12; //TODO: Determine the actual CAN ID of this CANcoder

	/** Drivetrain Settings ***************************************************/
	public static final double JOYSTICK_DRIVE_SCALE = 1; // Determines the max driving speed of the robot
	public static final double JOYSTICK_DRIVE_SCALE_LOW = 0.2; // Determines driving speed of the robot when in slow mode
	public static final double VELOCITY_MAX_MPS = 4;
	public static final double ACCELERATION_MAX_MPS = 2;
	public static final double kP_X_CONTROLLER = 9.6421; // TODO: Find this value by characterizing the drivetrain with SysID, and then by using guess & check afterwards
    public static final double kP_Y_CONTROLLER = 9.6421; // TODO: Find this value by characterizing the drivetrain with SysID, and then by using guess & check afterwards	
    public static final double kP_THETA_CONTROLLER = 9.6421; // TODO: Find this value by characterizing the drivetrain with SysID, and then by using guess & check afterwards
	public static final double kMAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = VELOCITY_MAX_MPS / Math.hypot(TRACK_WIDTH / 2, WHEEL_BASE /2);
	public static final double kMAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED = ACCELERATION_MAX_MPS / Math.hypot(TRACK_WIDTH / 2, WHEEL_BASE /2);
	//swerve part :3
    public static final boolean kFrontLeftDriveEncoderReversed = false;
    public static final boolean kFrontLeftTurningEncoderReversed = false;
	public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
    public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 0; //TODO: Measure this offset on the physical SwerveBot after it is built
    
    public static final boolean kFrontRightDriveEncoderReversed = false;
    public static final boolean kFrontRightTurningEncoderReversed = false;
	public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
    public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 0; //TODO: Measure this offset on the physical SwerveBot after it is built
    
    public static final boolean kBackLeftDriveEncoderReversed = false;
	public static final boolean kBackLeftTurningEncoderReversed = false;
	public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
    public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 0; //TODO: Measure this offset on the physical SwerveBot after it is built
    
    public static final boolean kBackRightDriveEncoderReversed = false;
    public static final boolean kBackRightTurningEncoderReversed = false;
	public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;
    public static final double kBackRightDriveAbsoluteEncoderOffsetRad = 0; //TODO: Measure this offset on the physical SwerveBot after it is built

	/** Swerve Module Settings ************************************************/
	public static final double kWheelDiameterMeters = Units.inchesToMeters(4); // Convert wheel diameter in inches to meters
	public static final double kDriveMotorGearRatio = 1 / 6.75; // Gear Ratio of the Drive Motor
	public static final double kTurnMotorGearRatio = 1 / 12.8; // Gear Ratio of the Turning Motor
	public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters; // Convert rotations to meters
	public static final double kTurningEncoderRot2Rad = kTurnMotorGearRatio * 2 * Math.PI; // Convert rotations to radians
	public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60; // Convert RPM to meters/second
	public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60; // Convert RPM to radians/sec
	public static final double kPhysicalMaxSpeedMetersPerSecond = Units.feetToMeters(14.5); // Convert max speed from feet/sec to meters/sec
	public static final double kP_Turning = 1.0; // TODO: We might need to tune this value through testing!
	public static final double nominalVoltage = 12.0;
    public static final double driveCurrentLimit = 80.0;
    public static final double steerCurrentLimit = 20.0;
	
	/** Drivetrain Gyro Drive Settings ****************************************/
	public static final double kP_GYRO = 0.01;
	public static final double kI_GYRO = 0.0;
	public static final double kD_GYRO = 0.015;
	public static final double ANGLE_TOLERANCE = 2.0;
	public static final double MAX_POWER_GYRO = 0.4;
	
	/** USB ports *************************************************************/					
	public static final int USB_JOYSTICK_LEFT 	= 0;
	public static final int USB_JOYSTICK_RIGHT 	= 1;
	public static final int USB_GAMEPAD 		= 2;

	/** Logger Settings *******************************************************/
	public static final String 		LOG_FILE_FORMAT 					= "yyyy-MM-dd-hhmmss";
	public static final String 		LOG_TIME_FORMAT 					= "hh:mm:ss:SSS";
	public static final String 		LOG_DIRECTORY_PATH 					= "/home/lvuser/logs/";
	public static final String 		LOG_TIME_ZONE 						= "America/Chicago";
	public static final boolean 	LOG_TO_CONSOLE 						= true;
	public static final boolean 	LOG_TO_FILE 						= false;
	public static final Log.Level 	LOG_GLOBAL 							= Log.Level.DEBUG;
	public static final Log.Level 	LOG_ROBOT 							= Log.Level.TRACE;
	public static final Log.Level	LOG_OI 								= Log.Level.TRACE;
	public static final Log.Level	LOG_AXIS_TRIGGER 					= Log.Level.ERROR;
	public static final Log.Level	LOG_POV_BUTTON						= Log.Level.ERROR;
	
	/** Subsystems ************************************************************/
	public static final Log.Level	LOG_DRIVETRAIN						= Log.Level.TRACE;
}