package frc.robot.commands.drive;

import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import static frc.robot.RobotContainer.drive;
import static frc.robot.RobotContainer.photon;
import frc.robot.util.Util;
import static frc.robot.util.Util.getAdjustedPose;

public class AlignToReef extends Command {

	// TODO: Adjust PID gains

	private PIDController xPID = new PIDController(3, 0.1, 0.6);
	private PIDController yPID = new PIDController(5, 0.2, 0.5);
	private ProfiledPIDController wPID = new ProfiledPIDController(1.5, 0., 0.4, new Constraints(Units.degreesToRadians(540),4*Math.PI));

	private Optional<Pose2d> target_pose;
	private Optional<Pose2d> stored_pose = Optional.empty();

	private Pose2d adj_pose;

	public AlignToReef() {
		addRequirements(drive);
		wPID.enableContinuousInput(0, 2 * Math.PI);
	}

	@Override
	public void initialize() {
		target_pose = photon.getAprilTagPose();
		if (target_pose.isEmpty()
				&& stored_pose
						.isEmpty()) { // For reliability, if not receiving new pose from PhotonVision, use previously
			// saved pose if any as reference
			return;
		} else if (target_pose.isEmpty() && !stored_pose.isEmpty()) {
			// System.out.println(drive.getPose().getTranslation().getDistance(stored_pose.get().getTranslation()));
			target_pose = stored_pose;
		}

		adj_pose = getAdjustedPose(target_pose.get());
	}

	@Override
	public void execute() {

		Pose2d current_pose = drive.getPose();
		double curr_X = current_pose.getX();
		double curr_Y = current_pose.getY();

		double adj_X = adj_pose.getX();
		double adj_Y = adj_pose.getY();

		double curr_rot = current_pose.getRotation().getRadians();
		double target_rot = target_pose.get().getRotation().getRadians();


		Logger.recordOutput("/Odom/adjusted pose", adj_pose);
		Logger.recordOutput("/Odom/adjusted_pose/x", adj_X);
		Logger.recordOutput("/Odom/adjusted_pose/y", adj_Y);
		Logger.recordOutput("/Odom/adjusted_pose/w", adj_pose.getRotation().getRadians());

		Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
			current_pose,
			List.of(),
			adj_pose,
			drive.getConfig());

		SwerveControllerCommand swerveCommand = new SwerveControllerCommand(
			trajectory,
			drive::getPose, 
			drive.getKinematics(),
			xPID,
			yPID,
			wPID,
			drive::setModuleStates,
		 	drive);
		
		swerveCommand.schedule();
	}

	@Override
	public boolean isFinished() {

		if (target_pose.isEmpty()) {
			System.out.println("NO TARGET, REPOSITION AND TRY AGAIN");
			return true;
		}

		Pose2d current_pose = drive.getPose();
		double dX = Math.abs(current_pose.getX() - adj_pose.getX());
		double dY = Math.abs(current_pose.getY() - adj_pose.getY()); // Translational difference

		double angle_offset =
				Math.abs(Util.convertAngle(current_pose.getRotation().getRadians())
						- Util.convertAngle(adj_pose.getRotation().getRadians())); // Angular difference

		if (dX < 0.04 && dY < 0.04 && angle_offset <= (4 * Math.PI) / 360) {
			System.out.println("Aligned");
			return true;
		}

		System.out.println("Still working on it " + " Angular Dist: " + angle_offset);
		return false;
	}

	@Override
	public void end(boolean interrupted) {
		super.end(interrupted);
		drive.drive(new ChassisSpeeds());
		stored_pose = Optional.empty();
	}
}
