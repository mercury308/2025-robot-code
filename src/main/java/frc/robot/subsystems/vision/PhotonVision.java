package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PhotonVision {
	private PhotonCamera april_cam1;
	private PhotonCamera april_cam2;
	private PhotonCamera april_cam3;

	AprilTagFieldLayout fieldLayout;

	Transform3d robotToCam1 = new Transform3d(
			new Translation3d(Units.inchesToMeters(-15), Units.inchesToMeters(4.5), Units.inchesToMeters(10)),
			new Rotation3d(0, Units.degreesToRadians(180), 0));
	Transform3d robotToCam2 = new Transform3d(
			new Translation3d(Units.inchesToMeters(-15), Units.inchesToMeters(4.5), Units.inchesToMeters(11)),
			new Rotation3d(0, Units.degreesToRadians(180), 0));
	Transform3d robotToCam3 = new Transform3d(
			new Translation3d(Units.inchesToMeters(-15), Units.inchesToMeters(4.5), Units.inchesToMeters(11)),
			new Rotation3d(0, Units.degreesToRadians(180), 0));

	PhotonPoseEstimator photonPoseEstimator1;
	PhotonPoseEstimator photonPoseEstimator2;
	PhotonPoseEstimator photonPoseEstimator3;

	//	private Pose2d camRobot = new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(11), new Rotation2d());

	public PhotonVision() {
		april_cam1 = new PhotonCamera("Global_Shutter_Camera");

		photonPoseEstimator1 =
				new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCam1);
		photonPoseEstimator1.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
		photonPoseEstimator2 =
				new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCam2);
		photonPoseEstimator2.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
		photonPoseEstimator3 =
				new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCam3);
		photonPoseEstimator3.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
	}

	private Optional<EstimatedRobotPose> getEstimate(PhotonCamera april_cam, PhotonPoseEstimator photonPoseEstimator) {
		if (april_cam == null) return Optional.empty();
		if (!april_cam.isConnected()) return Optional.empty();
		// if (april_cam.getLatestResult().getTargets().size() < 2) return Optional.empty();
		PhotonTrackedTarget tag = april_cam.getLatestResult().getBestTarget();
		if (tag == null) return Optional.empty();
		if (tag.getPoseAmbiguity() > 0.5)
			return Optional.empty(); // Reject pose update if ambiguity is above certain threshold
		return photonPoseEstimator.update(april_cam.getLatestResult());
	}

	/**
	 * Returns an optional EstimatedRobotPose object representing the estimated global pose of the robot.
	 * If the April camera is not available, not connected, or does not detect at least 2 targets, an empty optional is returned.
	 *
	 * @return an optional EstimatedRobotPose object representing the estimated global pose of the robot, or an empty optional if the pose cannot be estimated
	 */
	public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
		return getEstimate(april_cam1, photonPoseEstimator1);
	}
}
