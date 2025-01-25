package frc.robot.subsystems.drive;

import java.io.IOException;
import java.util.Optional;

import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import static frc.robot.RobotContainer.drive;
import static frc.robot.RobotContainer.imu;
import static frc.robot.constants.Constants.PATH_FOLLOWER_CONFIG;
import static frc.robot.constants.Constants.ROBOT_CONFIG;
import static frc.robot.constants.Constants.RobotConstants.ROBOT_LENGTH;
import static frc.robot.constants.Constants.RobotConstants.ROBOT_WIDTH;
import static frc.robot.constants.Constants.RobotConstants.SWERVE_MAXSPEED;
import frc.robot.constants.SwerveModuleConfiguration;
import frc.robot.util.Util;

public class SwerveSubsystem extends SubsystemBase {
	public SwerveModule[] modules = new SwerveModule[] {
		new SwerveModule(new ModuleIOSparkMax(SwerveModuleConfiguration.NW), "NW"),
		new SwerveModule(new ModuleIOSparkMax(SwerveModuleConfiguration.NE), "NE"),
		new SwerveModule(new ModuleIOSparkMax(SwerveModuleConfiguration.SW), "SW"),
		new SwerveModule(new ModuleIOSparkMax(SwerveModuleConfiguration.SE), "SE"),
	};
	public SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
			new Translation2d(ROBOT_LENGTH / 2, ROBOT_WIDTH / 2),
			new Translation2d(ROBOT_LENGTH / 2, -ROBOT_WIDTH / 2),
			new Translation2d(-ROBOT_LENGTH / 2, ROBOT_WIDTH / 2),
			new Translation2d(-ROBOT_LENGTH / 2, -ROBOT_WIDTH / 2));

	public SwerveDrivePoseEstimator pose_est;
	/**
	 * Initializes the SwerveSubsystem with the given initial pose.
	 *
	 * @param init_pose The initial pose of the robot.
	 */
	public void init(Pose2d init_pose) {
		pose_est = RobotState.pose_est;
				
		AutoBuilder.configure(
				RobotState.getPose(),
				(pose) -> pose_est.resetPosition(new Rotation2d(imu.yaw()), getPositions(), pose),
				this::getVelocity,
				this::drive,
				PATH_FOLLOWER_CONFIG,
				ROBOT_CONFIG,
				() -> Util.isRed(),
				this);

		PathPlannerLogging.setLogActivePathCallback((activePath) -> {
			Logger.recordOutput("Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
		});
		PathPlannerLogging.setLogTargetPoseCallback((targetPose) -> {
			Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
		});

	}

	/**
	 * Returns an array of SwerveModulePosition objects representing the positions of the swerve modules.
	 *
	 * @return an array of SwerveModulePosition objects representing the positions of the swerve modules
	 */
	public SwerveModulePosition[] getPositions() {
		SwerveModulePosition[] pos = new SwerveModulePosition[4];
		for (int i = 0; i < 4; i++) pos[i] = modules[i].getPosition();
		return pos;
	}

	/**
	 * Returns the velocity of the swerve subsystem in terms of ChassisSpeeds.
	 *
	 * @return The velocity of the swerve subsystem as ChassisSpeeds.
	 */
	public ChassisSpeeds getVelocity() {
		SwerveModuleState[] pos = new SwerveModuleState[4];
		for (int i = 0; i < 4; i++) pos[i] = modules[i].getMeasuredState();
		return kinematics.toChassisSpeeds(pos);
	}

	public ChassisSpeeds desired_speeds = new ChassisSpeeds();

	/**
	 * Drives the swerve subsystem at the specified speed.
	 *
	 * @param speed the desired speed of the swerve subsystem
	 */
	public void drive(ChassisSpeeds speed) {
		speed = ChassisSpeeds.discretize(speed, 0.02);
		desired_speeds = speed;
		SwerveModuleState[] states = kinematics.toSwerveModuleStates(speed);
		SwerveDriveKinematics.desaturateWheelSpeeds(states, SWERVE_MAXSPEED);
		for (int i = 0; i < 4; i++){
			modules[i].setState(states[i]);
		}
	}

	public void getOffsets() {
		for (SwerveModule module : modules) module.fixOffset();
	}

	public void zero() {
		for (int i = 0; i < 4; i++) modules[i].setState(new SwerveModuleState());
	}

	public void cross() {
		for (int i = 0; i < 4; i++) modules[i].setState(new SwerveModuleState(0, new Rotation2d(45)));
	}

	/**
	 * Updates the logging information for the SwerveSubsystem.
	 * Records various outputs such as speed setpoints, actual speeds, pose estimates, module setpoints, module speeds, and note positions.
	 */
	private void updateLogging() {
		Logger.recordOutput("/Swerve/speed_setpoint", new double[] {
			desired_speeds.vxMetersPerSecond, desired_speeds.vyMetersPerSecond, desired_speeds.omegaRadiansPerSecond
		});
		ChassisSpeeds speeds = getVelocity();
		Logger.recordOutput(
				"/Swerve/speeds",
				new double[] {speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond});

		double[] states = new double[8];
		for (int i = 0; i < 4; i++) states[i * 2 + 1] = modules[i].getTargetState().speedMetersPerSecond;
		for (int i = 0; i < 4; i++)
			states[i * 2] = modules[i].getTargetState().angle.getRadians();
		Logger.recordOutput("/Swerve/module_setpoint", states);

		for (int i = 0; i < 4; i++) states[i * 2 + 1] = modules[i].getMeasuredState().speedMetersPerSecond;
		for (int i = 0; i < 4; i++)
			states[i * 2] = modules[i].getMeasuredState().angle.getRadians();
		Logger.recordOutput("/Swerve/module_speeds", states);

	}

	private Optional<EstimatedRobotPose> est_pos;

	/**
	 * Updates the state of the SwerveSubsystem periodically.
	 * This method is called repeatedly to perform necessary updates and calculations.
	 * It updates the pose estimation, vision measurements, and logging.
	 */


	/**
	 * Returns the current estimated pose of the robot.
	 *
	 * @return the current estimated pose of the robot
	 */

	/**
	 * Returns the field oriented Y velocity of the robot in meters per second.
	 * The Y velocity is calculated based on the current velocity of the robot and its orientation.
	 *
	 * @return the Y velocity of the robot in meters per second
	 */
	public double getYVel() {
		ChassisSpeeds velocity = getVelocity();
		double theta = RobotState.getPose().getRotation().getRadians();
		return Math.cos(theta) * velocity.vyMetersPerSecond + Math.sin(theta) * velocity.vxMetersPerSecond;
	}

	/**
	 * Returns the field oriented X velocity of the robot in meters per second.
	 * The X velocity is calculated based on the current velocity of the robot and its orientation.
	 *
	 * @return the X velocity of the robot in meters per second
	 *
	 */
	public double getXVel() {
		ChassisSpeeds velocity = drive.getVelocity();
		double theta = RobotState.getPose().getRotation().getRadians();
		return Math.sin(theta) * velocity.vyMetersPerSecond + Math.cos(theta) * velocity.vxMetersPerSecond;
	}

	

	public Command followPath(String fileString) throws FileVersionException, IOException, ParseException {
		PathPlannerPath path  = PathPlannerPath.fromPathFile(fileString);
		return AutoBuilder.followPath(path);
	}
}