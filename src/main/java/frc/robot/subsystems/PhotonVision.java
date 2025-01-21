package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.Constants;
import java.io.IOException;
import java.io.UncheckedIOException;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PhotonVision {
	private PhotonCamera april_cam;
	AprilTagFieldLayout fieldLayout;

	Transform3d robotToCam = new Transform3d(
			new Translation3d(Units.inchesToMeters(15), Units.inchesToMeters(0), 0.01),
			new Rotation3d(0, Units.degreesToRadians(0), 0));
	PhotonPoseEstimator photonPoseEstimator;

//	private Pose2d camRobot = new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(11), new Rotation2d());

	public PhotonVision() {
		try {
			fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
		} catch (UncheckedIOException e) {
			System.out.println("Couldn't Find April Tag Layout File");
			e.printStackTrace();
		}

		//note_cam = new PhotonCamera("Global_Shutter_Camera (1)");

		april_cam = new PhotonCamera("Global_Shutter_Camera");
		photonPoseEstimator =
				new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,  robotToCam);
		photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
	}
	

	/**
	 * Returns an optional EstimatedRobotPose object representing the estimated global pose of the robot.
	 * If the April camera is not available, not connected, or does not detect at least 2 targets, an empty optional is returned.
	 *
	 * @return an optional EstimatedRobotPose object representing the estimated global pose of the robot, or an empty optional if the pose cannot be estimated
	 */
	public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
		if (april_cam == null) return Optional.empty();
		if (!april_cam.isConnected()) return Optional.empty();
		//if (april_cam.getLatestResult().getTargets().size() < 2) return Optional.empty();
		PhotonTrackedTarget tag = april_cam.getLatestResult().getBestTarget();
		if(tag.getPoseAmbiguity() > 0.5) return Optional.empty(); // Reject pose update if ambiguity is above certain threshold
		return photonPoseEstimator.update(april_cam.getLatestResult());
	}
	// Returns the pose of an AprilTag relative to CAMERA
	public Optional<Pose2d> getAprilTagPose(){
		if (april_cam == null) return Optional.empty();
		if (!april_cam.isConnected()) return Optional.empty();
		//if (april_cam.getLatestResult().getTargets().size() < 2) return Optional.empty();

		PhotonTrackedTarget target = april_cam.getLatestResult().getBestTarget();
		if(target == null){
			//System.out.println("NO TARGETS IN SIGHT");
			return Optional.empty();
		}

		return Optional.of(
				fieldLayout
				.getTagPose(
					target
					.getFiducialId())
					.get()
					.toPose2d());
		
	}

}