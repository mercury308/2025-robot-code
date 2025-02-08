package frc.robot.subsystems.drive;

import static java.lang.Math.PI;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

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
 * Module IO implementation for TalonFX drive motor controller, TalonFX turn motor controller, and
 * CANcoder absolute encoder connected to the RIO.
 */
public class ModuleIOTalonFX implements ModuleIO {
	// Gear ratios for SDS MK4i L2, adjust as necessary
	private static final double DRIVE_GEAR_RATIO = 1 / L3_DRIVE_RATIO;
	private static final double TURN_GEAR_RATIO = 1 / L3_TURN_RATIO;

	private final TalonFX driveTalonFX;
	private final TalonFX turnTalonFX;
	private final TalonFXConfiguration driveConfig;
	private final TalonFXConfiguration turnConfig;
	private final TalonFXConfigurator driveConfigurator;
	private final TalonFXConfigurator turnConfigurator;

	private final CANcoder turnAbsoluteEncoder;

	private final boolean isTurnMotorInverted = true;
	private final Rotation2d absoluteEncoderOffset;

	public ModuleIOTalonFX(SwerveModuleConfiguration config) {
		driveTalonFX = new TalonFX(config.DRIVE_MOTOR);
		turnTalonFX = new TalonFX(config.TURN_MOTOR);

		driveConfigurator = driveTalonFX.getConfigurator();
		turnConfigurator = turnTalonFX.getConfigurator();

		turnAbsoluteEncoder = new CANcoder(config.ENCODER);
		absoluteEncoderOffset = config.offset; // MUST BE CALIBRATED

		driveConfigurator.apply(new TalonFXConfiguration());
		turnConfigurator.apply(new TalonFXConfiguration());

		driveConfig = new TalonFXConfiguration();
		turnConfig = new TalonFXConfiguration();
		// Configuring motor parameters

		driveTalonFX.clearStickyFaults();
		turnTalonFX.clearStickyFaults();

		driveConfig.Slot0.kP = MODULE_DRIVE_KP;
		driveConfig.Slot0.kD = MODULE_DRIVE_KF; // Drive FeedForwards
		driveConfig.Slot0.kV = 
			(Units.rotationsPerMinuteToRadiansPerSecond(1) / DRIVE_GEAR_RATIO * SWERVE_WHEEL_RAD);
		driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;	
       		driveConfig.CurrentLimits.withSupplyCurrentLimit(30);
		driveConfigurator.apply(driveConfig);

		turnConfig.Slot0.kP = MODULE_DRIVE_KP;
		turnConfig.Slot0.kD = MODULE_DRIVE_KF; // Turn FeedForwards
        	turnConfig.MotorOutput.Inverted = 
           	 isTurnMotorInverted ? InvertedValue.Clockwise_Positive
            	: InvertedValue.CounterClockwise_Positive;
		turnConfig.CurrentLimits.StatorCurrentLimitEnable = true;	
        	turnConfig.CurrentLimits.withStatorCurrentLimit(20);
		turnConfigurator.apply(turnConfig);

		driveTalonFX.setNeutralMode(NeutralModeValue.Brake);
		turnTalonFX.setNeutralMode(NeutralModeValue.Brake);
	}

	@Override
	public void updateInputs(ModuleIOInputs inputs) {
		inputs.drivePositionRad = driveTalonFX.getPosition().getValueAsDouble() * 2 * PI / DRIVE_GEAR_RATIO;
		inputs.driveVelocityMetersPerSec = driveTalonFX.getVelocity().getValueAsDouble();
		inputs.driveAppliedVolts = driveTalonFX.getDutyCycle().getValueAsDouble()
				* driveTalonFX.getSupplyVoltage().getValueAsDouble();
		inputs.driveCurrentAmps = new double[] {driveTalonFX.getStatorCurrent().getValueAsDouble()};

		inputs.turnAbsolutePosition = new Rotation2d(
						Units.rotationsToRadians(turnAbsoluteEncoder.getAbsolutePosition().getValueAsDouble()))
				.minus(absoluteEncoderOffset);
		inputs.turnAbsolutePositionRad = inputs.turnAbsolutePosition.getRadians();
		inputs.turnPosition = Rotation2d.fromRotations(turnTalonFX.getPosition().getValueAsDouble() / TURN_GEAR_RATIO);
		inputs.turnVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(
						turnTalonFX.getVelocity().getValueAsDouble())
				/ TURN_GEAR_RATIO;
		inputs.turnAppliedVolts = turnTalonFX.getDutyCycle().getValueAsDouble()
				* turnTalonFX.getSupplyVoltage().getValueAsDouble();
		inputs.turnCurrentAmps = new double[] {turnTalonFX.getStatorCurrent().getValueAsDouble()};
	}

	@Override
	public void setDriveVoltage(double volts) {
		driveTalonFX.setVoltage(volts);
	}

	@Override
	public void setDriveVelocity(double mps) {
		driveTalonFX.set(mps); // Needs to be set to meters per second control mdode
	}

	@Override
	public void setTurnVoltage(double volts) {
		turnTalonFX.setVoltage(volts);
	}

	@Override
	public void setDriveBrakeMode(boolean enable) {
		driveTalonFX.setNeutralMode(enable ? NeutralModeValue.Brake : NeutralModeValue.Coast);
	}

	@Override
	public void setTurnBrakeMode(boolean enable) {
		turnTalonFX.setNeutralMode(enable ? NeutralModeValue.Brake : NeutralModeValue.Coast);
	}

	@Override
	public void logTargetState(ModuleIOInputs inputs, SwerveModuleState state, double compensatedVel) {
		inputs.targetRad = state.angle.getRadians();
		inputs.targetVel = state.speedMetersPerSecond;
		inputs.compensatedTargetVel = compensatedVel;
	}
}
