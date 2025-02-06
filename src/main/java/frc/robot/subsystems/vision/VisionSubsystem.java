package frc.robot.subsystems.vision;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import static frc.robot.RobotContainer.drive;
import static frc.robot.RobotContainer.imu;
import frc.robot.constants.LimelightConfiguration;

public class VisionSubsystem extends SubsystemBase {
	private LimelightIO io;
	private LimelightConfiguration config;
	private final LimelightInputsAutoLogged inputs = new LimelightInputsAutoLogged();
	private Pose2d robotToField = new Pose2d();
	private double mt2Timestamp = 0.0;
	private boolean doRejectUpdate = false;

	public void init(LimelightConfiguration _config) {
		config = _config;
		io = new LimelightIO(config.Name);

		// System.out.println("Initialized limelight with name, " + config.Name + "");
	}

	@Override
	public void periodic() {
		io.updateInputs(inputs);
		Logger.processInputs(config.Name, inputs);
		LimelightHelpers.SetRobotOrientation(
				config.Name, drive.getPose().getRotation().getDegrees(), 0, 0, 0, 0, 0);
		LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(config.Name);

		if (Math.abs(imu.getAngularVelocity()) > 720) {
			doRejectUpdate = true;
		}
		if (!hasTarget()) {
			doRejectUpdate = true;
		}
		if (!doRejectUpdate) {
			robotToField = mt2.pose;
			mt2Timestamp = mt2.timestampSeconds;
			drive.addLimelightMeasurement(robotToField, mt2Timestamp);
			// System.out.println("Sent measurement");
		}
	}

	public boolean hasTarget() {
		return inputs.hasTarget;
	}

	public double getYawRadians() {
		return inputs.yaw;
	}

	public double getPitchRadians() {
		return inputs.pitch;
	}

	public Optional<Pose2d> getEstimatePose() {
		return Optional.of(this.robotToField);
	}

	public int getTargetID() {
		return inputs.iD;
	}
}
