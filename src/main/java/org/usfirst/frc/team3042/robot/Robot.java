package org.usfirst.frc.team3042.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import org.usfirst.frc.team3042.lib.Log;
import org.usfirst.frc.team3042.robot.commands.autonomous.AutonomousMode_Default;
import org.usfirst.frc.team3042.robot.subsystems.Drivetrain;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.cameraserver.CameraServer;

/** Robot *********************************************************************
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource directory. */
public class Robot extends TimedRobot { 
	/** Configuration Constants ***********************************************/
	private static final Log.Level LOG_LEVEL = RobotMap.LOG_ROBOT;

	/** Create Subsystems *****************************************************/
	private Log log = new Log(LOG_LEVEL, "Robot");
	public static final Drivetrain drivetrain = new Drivetrain();
	public static final OI oi = new OI();;

	static ProfiledPIDController thetaController = new ProfiledPIDController(RobotMap.kP_THETA_CONTROLLER, 0, 0, drivetrain.getkThetaControllerConstraints());
	public static final PowerDistribution pdp = new PowerDistribution();
	
	CommandBase autonomousCommand;
	SendableChooser<CommandBase> chooser = new SendableChooser<CommandBase>();

	double goalAngle;
	UsbCamera camera1;
	Timer currentTimer = new Timer();

	/** robotInit *************************************************************
	 * This function is run when the robot is first started up and should be used for any initialization code. */
	public void robotInit() {
		log.add("Robot Init", Log.Level.TRACE);

		thetaController.enableContinuousInput(-Math.PI, Math.PI);

		drivetrain.zeroGyro();
		
		// Autonomous Routines //
		chooser.setDefaultOption("Default Auto", new AutonomousMode_Default());
		SmartDashboard.putData("Auto Mode", chooser);

		// Start up the webcam and configure its resolution and framerate
		camera1 = CameraServer.startAutomaticCapture(0);
		camera1.setResolution(320, 240);
		camera1.setFPS(15);
	}

	/** disabledInit **********************************************************
	 * This function is called once each time the robot enters Disabled mode.
	 * You can use it to reset any subsystem information you want to clear when the robot is disabled. */
	public void disabledInit() {
		log.add("Disabled Init", Log.Level.TRACE);
	}

	/** disabledPeriodic ******************************************************
	 * Called repeatedly while the robot is in disabled mode. */
	public void disabledPeriodic() {
		CommandScheduler.getInstance().run();

		SmartDashboard.putString("BackLeft State", drivetrain.getBackLeft().getState().toString());
		SmartDashboard.putString("FrontLeft State", drivetrain.getFrontLeft().getState().toString());
		SmartDashboard.putString("BackRight State", drivetrain.getBackRight().getState().toString());
		SmartDashboard.putString("FrontRight State", drivetrain.getFrontRight().getState().toString());
	}

	/** autonomousInit ********************************************************
	 * Runs once at the start of autonomous mode. */
	public void autonomousInit() {
		log.add("Autonomous Init", Log.Level.TRACE);

		autonomousCommand = chooser.getSelected();
		
		// schedule the autonomous command
		if (autonomousCommand != null) {
			autonomousCommand.schedule();
		}
	}

	/** autonomousPeriodic ****************************************************
	 * This function is called periodically during autonomous */
	public void autonomousPeriodic() {
		CommandScheduler.getInstance().run();
	}
	
	/** teleopInit ************************************************************
	 * This function is called when first entering teleop mode. */
	public void teleopInit() {
		log.add("Teleop Init", Log.Level.TRACE);

		// This makes sure that the autonomous command stops running when teleop starts. 
		//If you want the autonomous command to continue until interrupted by another command, remove this line or comment it out.
		if (autonomousCommand != null) {
			autonomousCommand.cancel();
		}  
		goalAngle = drivetrain.getGyroAngle();
	}

	/** teleopPeriodic ********************************************************
	 * This function is called periodically during operator control */
	public void teleopPeriodic() {
		CommandScheduler.getInstance().run();
		SmartDashboard.putNumber("Gyro Angle", drivetrain.getGyroAngle()); // The current gyroscope angle
		       
        SmartDashboard.putString("BackLeft State", drivetrain.getBackLeft().getState().toString());
		SmartDashboard.putString("FrontLeft State", drivetrain.getFrontLeft().getState().toString());
		SmartDashboard.putString("BackRight State", drivetrain.getBackRight().getState().toString());
		SmartDashboard.putString("FrontRight State", drivetrain.getFrontRight().getState().toString());

		double ySpeed = oi.getYSpeed();
		double xSpeed = oi.getXSpeed();
		double zSpeed = oi.getZSpeed();

		drivetrain.drive(xSpeed, ySpeed, zSpeed, true);
	} 

	public static SequentialCommandGroup constructTrajectoryCommand(String pathName, double velocityMax, double accelMax) { // Give this a path name and it will return a PPSwerveControllerCommand for that path :)
		
		PathPlannerTrajectory path = PathPlanner.loadPath(pathName, velocityMax, accelMax); 

		// Add kinematics to ensure max speed is actually obeyed
		PPSwerveControllerCommand swerveControllerCommand = new PPSwerveControllerCommand(path, drivetrain::getPose, drivetrain.getkDriveKinematics(),

		// Position contollers
		new PIDController(RobotMap.kP_X_CONTROLLER, 0, 0),
		new PIDController(RobotMap.kP_Y_CONTROLLER, 0, 0),
		thetaController,

		drivetrain::setModuleStates, drivetrain);

		PathPlannerState initialState = (PathPlannerState)path.sample(0); // Define the initial state of the trajectory

		return new SequentialCommandGroup(
			new InstantCommand(() -> drivetrain.resetOdometry(new Pose2d(initialState.poseMeters.getTranslation(), initialState.holonomicRotation))),
			swerveControllerCommand,
			new InstantCommand(() -> drivetrain.stopModules())
		);
	}
}