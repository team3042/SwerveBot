package org.usfirst.frc.team3042.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.usfirst.frc.team3042.robot.RobotMap;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule {

    private static final double kDriveEncoderRot2Meter = RobotMap.kDriveEncoderRot2Meter;
    private static final double kDriveEncoderRPM2MeterPerSec = RobotMap.kDriveEncoderRPM2MeterPerSec;
    private static final double kTurningEncoderRot2Rad = RobotMap.kTurningEncoderRot2Rad;
    private static final double kTurningEncoderRPM2RadPerSec = RobotMap.kTurningEncoderRPM2RadPerSec;

    private final CANSparkMax driveMotor;
    private final CANSparkMax turningMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turningEncoder;

    private final PIDController turningPidController;

    private final CANCoder absoluteEncoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;

    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
                        int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {

        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        absoluteEncoder = new CANCoder(absoluteEncoderId);

        driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        turningMotor = new CANSparkMax(turningMotorId, MotorType.kBrushless);

        driveMotor.restoreFactoryDefaults();
        turningMotor.restoreFactoryDefaults();

        driveMotor.setInverted(driveMotorReversed);
        turningMotor.setInverted(turningMotorReversed);

        driveMotor.setIdleMode(IdleMode.kBrake);
        turningMotor.setIdleMode(IdleMode.kBrake);

        driveEncoder = driveMotor.getEncoder();
        turningEncoder = turningMotor.getEncoder();
        
        driveEncoder.setPositionConversionFactor(kDriveEncoderRot2Meter); 
        driveEncoder.setVelocityConversionFactor(kDriveEncoderRPM2MeterPerSec); 
        turningEncoder.setPositionConversionFactor(kTurningEncoderRot2Rad); 
        turningEncoder.setVelocityConversionFactor(kTurningEncoderRPM2RadPerSec); 
   
        turningPidController = new PIDController(RobotMap.kPTurning, 0, 0);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders();
    }

    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }
    public double getTurningPosition() {
        return turningEncoder.getPosition();
    }
    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }
    public double getTurningVelocity() {
        return turningEncoder.getVelocity();
    }

    public double getAbsoluteEncoderRadians() {
        double angle = absoluteEncoder.getAbsolutePosition();
        angle *= (Math.PI / 180);
        angle -= absoluteEncoderOffsetRad;
        return angle * (absoluteEncoderReversed ? -1.0 :1.0);
    }

    public void resetEncoders() {
        driveEncoder.setPosition(0);
        turningEncoder.setPosition(getAbsoluteEncoderRadians());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }

        state = SwerveModuleState.optimize(state, getState().angle);

        driveMotor.set(state.speedMetersPerSecond / RobotMap.kPhysicalMaxSpeedMetersPerSecond);// give me PIE
        turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
        SmartDashboard.putString("swerve[" + absoluteEncoder.getDeviceID() + "] state", state.toString());
    }

    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }
}