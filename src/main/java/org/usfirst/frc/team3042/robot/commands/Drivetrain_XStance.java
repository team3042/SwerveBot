package org.usfirst.frc.team3042.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableRegistry;

import org.usfirst.frc.team3042.lib.Log;
import org.usfirst.frc.team3042.robot.Robot;
import org.usfirst.frc.team3042.robot.RobotMap;
import org.usfirst.frc.team3042.robot.subsystems.Drivetrain;

/** Drivetrain Gyro Straight **************************************************
 * Command for */
public class Drivetrain_XStance extends CommandBase {
	/** Configuration Constants ***********************************************/
	private static final Log.Level LOG_LEVEL = RobotMap.LOG_DRIVETRAIN;
	
	/** Instance Variables ****************************************************/
	Drivetrain drivetrain = Robot.drivetrain;
	Log log = new Log(LOG_LEVEL, SendableRegistry.getName(drivetrain));

	SwerveModuleState FrontLeftState = new SwerveModuleState(0, Rotation2d.fromDegrees(-45));
	SwerveModuleState FrontRightState = new SwerveModuleState(0, Rotation2d.fromDegrees(45));
	SwerveModuleState BackLeftState = new SwerveModuleState(0, Rotation2d.fromDegrees(45));
	SwerveModuleState BackRightState = new SwerveModuleState(0, Rotation2d.fromDegrees(-45));
	
	/** Drivetrain Gyro Straight **********************************************
	 * Required subsystems will cancel commands when this command is run */
	public Drivetrain_XStance() {
		log.add("Constructor", Log.Level.TRACE);
		addRequirements(drivetrain);
	}
	
	/** initialize ************************************************************
	 * Called just before this Command runs the first time */
	public void initialize() {
		log.add("Initialize", Log.Level.TRACE);
		drivetrain.stopModules();

		drivetrain.getFrontLeft().setDesiredState(FrontLeftState);
		drivetrain.getFrontRight().setDesiredState(FrontRightState);
		drivetrain.getBackLeft().setDesiredState(BackLeftState);
		drivetrain.getBackRight().setDesiredState(BackRightState);
	}

	/** execute ***************************************************************
	 * Called repeatedly when this Command is scheduled to run */
	public void execute() {}
	
	/** isFinished ************************************************************	
	 * Make this return true when this Command no longer needs to run execute() */
	public boolean isFinished() {
		return true;
	}

	// Called once the command ends or is interrupted.
	public void end(boolean interrupted) {
		log.add("End", Log.Level.TRACE);
	}
}