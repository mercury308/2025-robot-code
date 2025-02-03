package frc.robot.subsystems.drive;

import static java.lang.Math.PI;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import static frc.robot.constants.Constants.MODULE_DRIVE_KF;
import static frc.robot.constants.Constants.MODULE_DRIVE_KP;
import static frc.robot.constants.Constants.RobotConstants.L3_DRIVE_RATIO;
import static frc.robot.constants.Constants.RobotConstants.L3_TURN_RATIO;
import static frc.robot.constants.Constants.RobotConstants.SWERVE_WHEEL_RAD;
import frc.robot.constants.SwerveModuleConfiguration;

/**
 * Module IO implementation for SparkMax drive motor controller, SparkMax turn motor controller (NEO
 * or NEO 550), and analog absolute encoder connected to the RIO
 *
 * <p>NOTE: This implementation should be used as a starting point and adapted to different hardware
 * configurations (e.g. If using a CANcoder, copy from "ModuleIOTalonFX")
 *
 * <p>To calibrate the absolute encoder offsets, point the modules straight (such that forward
 * motion on the drive motor will propel the robot forward) and copy the reported values from the
 * absolute encoders using AdvantageScope. These values are logged under
 * "/Drive/ModuleX/TurnAbsolutePositionRad"
 */
public class ModuleIOSparkMax implements ModuleIO {
	// Gear ratios for SDS MK4i L2, adjust as necessary
	private static final double DRIVE_GEAR_RATIO = 1 / L3_DRIVE_RATIO;
	private static final double TURN_GEAR_RATIO = 1 / L3_TURN_RATIO;

	private final SparkMax driveSparkMax;
	private final SparkMax turnSparkMax;
	private final SparkMaxConfig driveConfig;
	private final SparkMaxConfig turnConfig;

	private final SparkClosedLoopController drivePID;

	private final RelativeEncoder driveEncoder;
	private final RelativeEncoder turnRelativeEncoder;
	private final CANcoder turnAbsoluteEncoder;

	private final boolean isTurnMotorInverted = true;
	private final Rotation2d absoluteEncoderOffset;

	public ModuleIOSparkMax(SwerveModuleConfiguration config) {
		driveSparkMax = new SparkMax(config.DRIVE_MOTOR, MotorType.kBrushless);
		turnSparkMax = new SparkMax(config.TURN_MOTOR, MotorType.kBrushless);
		driveConfig = new SparkMaxConfig();
		turnConfig = new SparkMaxConfig();
		turnAbsoluteEncoder = new CANcoder(config.ENCODER);
		drivePID = driveSparkMax.getClosedLoopController();
		absoluteEncoderOffset = config.offset; // MUST BE CALIBRATED
		driveSparkMax.clearFaults();
		turnSparkMax.clearFaults();

		driveSparkMax.setCANTimeout(250);
		turnSparkMax.setCANTimeout(250);

		// New way of setting up motor configurations

		driveConfig.smartCurrentLimit(30).voltageCompensation(12.0);
		driveConfig.encoder.velocityConversionFactor(
				Units.rotationsPerMinuteToRadiansPerSecond(1) / DRIVE_GEAR_RATIO * SWERVE_WHEEL_RAD);
		driveConfig.closedLoop.p(MODULE_DRIVE_KP).velocityFF(MODULE_DRIVE_KF);

		turnConfig.inverted(isTurnMotorInverted).smartCurrentLimit(20).voltageCompensation(12.0);
		turnConfig
				.encoder
				.velocityConversionFactor(
						Units.rotationsPerMinuteToRadiansPerSecond(1) / DRIVE_GEAR_RATIO * SWERVE_WHEEL_RAD)
				.uvwMeasurementPeriod(10)
				.uvwAverageDepth(2);

		driveSparkMax.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
		turnSparkMax.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

		driveEncoder = driveSparkMax.getEncoder();
		driveEncoder.setPosition(0.0);

		turnRelativeEncoder = turnSparkMax.getEncoder();
		turnRelativeEncoder.setPosition(0.0);

		driveSparkMax.setCANTimeout(0);
		turnSparkMax.setCANTimeout(0);
		// drivePID.setFF(MODULE_DRIVE_KF);
	}

	@Override
	public void updateInputs(ModuleIOInputs inputs) {
		inputs.drivePositionRad = driveEncoder.getPosition() * 2 * PI / DRIVE_GEAR_RATIO;
		inputs.driveVelocityMetersPerSec = driveEncoder.getVelocity();
		inputs.driveAppliedVolts = driveSparkMax.getAppliedOutput() * driveSparkMax.getBusVoltage();
		inputs.driveCurrentAmps = new double[] {driveSparkMax.getOutputCurrent()};

		inputs.turnAbsolutePosition = new Rotation2d(Units.rotationsToRadians(
						turnAbsoluteEncoder.getAbsolutePosition().getValueAsDouble()))
				.minus(absoluteEncoderOffset);
		inputs.turnAbsolutePositionRad = inputs.turnAbsolutePosition.getRadians();
		inputs.turnPosition = Rotation2d.fromRotations(turnRelativeEncoder.getPosition() / TURN_GEAR_RATIO);
		inputs.turnVelocityRadPerSec =
				Units.rotationsPerMinuteToRadiansPerSecond(turnRelativeEncoder.getVelocity()) / TURN_GEAR_RATIO;
		inputs.turnAppliedVolts = turnSparkMax.getAppliedOutput() * turnSparkMax.getBusVoltage();
		inputs.turnCurrentAmps = new double[] {turnSparkMax.getOutputCurrent()};
	}

	@Override
	public void setDriveVoltage(double volts) {
		driveSparkMax.setVoltage(volts);
	}

	@Override
	public void setDriveVelocity(double mps) {
		drivePID.setReference(mps, ControlType.kVelocity);
	}

	@Override
	public void setTurnVoltage(double volts) {
		turnSparkMax.setVoltage(volts);
	}

	@Override
	public void setDriveBrakeMode(boolean enable) {
		driveConfig.idleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
		driveSparkMax.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
	}

	@Override
	public void setTurnBrakeMode(boolean enable) {
		turnConfig.idleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
		turnSparkMax.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
	}

	@Override
	public void logTargetState(ModuleIOInputs inputs, SwerveModuleState state, double compensatedVel) {
		inputs.targetRad = state.angle.getRadians();
		inputs.targetVel = state.speedMetersPerSecond;
		inputs.compensatedTargetVel = compensatedVel;
	}
}
