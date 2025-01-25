package frc.robot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class RobotState {
        
        public static SwerveDrivePoseEstimator pose_est;

        private SwerveSubsystem drive;
        private VisionSubsystem vision;
        private Pose2d init_pose;
        private IMU imu;

        public RobotState(SwerveSubsystem _drive, VisionSubsystem _vision, Pose2d _init_pose){
                drive = _drive;
                vision = _vision;
                init_pose = _init_pose;

                pose_est = new SwerveDrivePoseEstimator(
                        drive.kinematics,
                        new Rotation2d(imu.yaw()),
                        drive.getPositions(),
                        init_pose,
                        VecBuilder.fill(0.1, 0.1, 0.1),
                        VecBuilder.fill(0.5, 0.5, 0.3));
        }



        private void updateLogging() {
	
		Logger.recordOutput("/Odom/pose", getPose());
		Logger.recordOutput("/Odom/rot", pose_est.getEstimatedPosition().getRotation());

		Logger.recordOutput("/Odom/x", pose_est.getEstimatedPosition().getX());
		Logger.recordOutput("/Odom/y", pose_est.getEstimatedPosition().getY());
		Logger.recordOutput("/Odom/rot_raw", pose_est.getEstimatedPosition().getRotation().getRadians());


	}

        public static Pose2d getPose() {
		Pose2d est_pose = pose_est.getEstimatedPosition();
		return est_pose;
	}

        public void updateFromLimelight(){
                
        }

}
