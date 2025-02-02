package frc.robot.constants;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.util.Units;

public final class Constants {

	public static final class RobotConstants {
		public static final double ROBOT_LENGTH = Units.inchesToMeters(29);
		public static final double ROBOT_WIDTH = Units.inchesToMeters(29);

		public static final double SWERVE_MAXSPEED = 4.42;
		public static final double ANGULAR_MAX_SPEED = SWERVE_MAXSPEED / (Math.hypot(ROBOT_LENGTH, ROBOT_WIDTH) / 2);

		public static final double SPEED_MULT = 0.75;
		public static final double TURBO_SPEED_MULT = 1;

		public static final double ANGULAR_SPEED_MULT = 0.75;
		public static final double TURBO_ANGULAR_SPEED_MULT = 1;

		public static final double SWERVE_WHEEL_RAD = 2 * 2.54 / 100;
		public static final double L2_DRIVE_RATIO = 1 / 6.75; // input RPM * gearing = output RPM
		public static final double L2_TURN_RATIO = 7. / 150;
	}
	
	public static final double MODULE_DRIVE_KP = 0.05;
	public static final double MODULE_DRIVE_KF = 0.23;
	public static final double MODULE_TURN_KP = 3;

	public static final double DRIVER_TURN_KP = 1.;

	public static final double CAMERA_HEIGHT = Units.inchesToMeters(13.5);

	public static final PPHolonomicDriveController PATH_FOLLOWER_CONFIG = new PPHolonomicDriveController(
			new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
			new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
			);

	public static RobotConfig ROBOT_CONFIG;

	static {
		try {
			ROBOT_CONFIG = RobotConfig.fromGUISettings();
		} catch (Exception e) {
			e.printStackTrace();
		}
	}
}
