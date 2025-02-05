package frc.robot.subsystems.drive;


import static java.lang.Math.PI;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
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


    private final CANCoder turnAbsoluteEncoder;

    private final boolean isTurnMotorInverted = true;
    private final Rotation2d absoluteEncoderOffset;

    public ModuleIOTalonFX(SwerveModuleConfiguration config) {
        driveTalonFX = new TalonFX(config.DRIVE_MOTOR);
        turnTalonFX = new TalonFX(config.TURN_MOTOR);

        driveConfig = new TalonFXConfiguration();
        turnConfig = new TalonFXConfiguration();

        driveConfigurator = driveTalonFX.getConfigurator();
        turnConfigurator = turnTalonFX.getConfigurator();

        turnAbsoluteEncoder = new CANCoder(config.ENCODER);
        absoluteEncoderOffset = config.offset; // MUST BE CALIBRATED
        driveTalonFX.clearStickyFaults();
        turnTalonFX.clearStickyFaults();

        driveTalonFX.configFactoryDefault();
        turnTalonFX.configFactoryDefault();

        // Configuring motor parameters

        driveConfig.Slot0.kP = MODULE_DRIVE_KP;
        driveConfig.Slot0.kD = MODULE_DRIVE_KF;
        driveConfigurator.apply(driveConfig);

        turnConfig.Slot0.kP = MODULE_DRIVE_KP;
        turnConfig.Slot0.kD = MODULE_DRIVE_KF;
        turnConfigurator.apply(turnConfig);

        driveTalonFX.setNeutralMode(NeutralModeValue.Brake);
        turnTalonFX.setNeutralMode(NeutralModeValue.Brake);

        driveTalonFX.setSelectedSensorPosition(0);
        turnTalonFX.setSelectedSensorPosition(0);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        inputs.drivePositionRad = driveTalonFX.getSelectedSensorPosition() * 2 * PI / DRIVE_GEAR_RATIO;
        inputs.driveVelocityMetersPerSec = driveTalonFX.getSelectedSensorVelocity();
        inputs.driveAppliedVolts = driveTalonFX.getMotorOutputVoltage();
        inputs.driveCurrentAmps = new double[] {driveTalonFX.getStatorCurrent()};

        inputs.turnAbsolutePosition = new Rotation2d(Units.rotationsToRadians(
                        turnAbsoluteEncoder.getAbsolutePosition()))
                .minus(absoluteEncoderOffset);
        inputs.turnAbsolutePositionRad = inputs.turnAbsolutePosition.getRadians();
        inputs.turnPosition = Rotation2d.fromRotations(turnTalonFX.getSelectedSensorPosition() / TURN_GEAR_RATIO);
        inputs.turnVelocityRadPerSec =
                Units.rotationsPerMinuteToRadiansPerSecond(turnTalonFX.getSelectedSensorVelocity()) / TURN_GEAR_RATIO;
        inputs.turnAppliedVolts = turnTalonFX.getMotorOutputVoltage();
        inputs.turnCurrentAmps = new double[] {turnTalonFX.getStatorCurrent()};
    }

    @Override
    public void setDriveVoltage(double volts) {
        driveTalonFX.set(ControlMode.PercentOutput, volts / 12.0);
    }

    @Override
    public void setDriveVelocity(double mps) {
        driveTalonFX.set(mps);
    }

    @Override
    public void setTurnVoltage(double volts) {
        turnTalonFX.set(ControlMode.PercentOutput, volts / 12.0);
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
