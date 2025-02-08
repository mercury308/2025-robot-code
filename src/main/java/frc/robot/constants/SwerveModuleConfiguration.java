package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Enum representing the configuration of a swerve module.
 * Includes motor ports, encoder ports, and offset angles.
 */
public enum SwerveModuleConfiguration {
	NW(18, 9, 10, 0),
	NE(17, 7, 8, 0),
	SW(15, 11, 12, 0),
	SE(16, 5, 6, 0);
	public int ENCODER, TURN_MOTOR, DRIVE_MOTOR;
	public Rotation2d offset;

	/**
	 * Constructs a SwerveModuleConfiguration with the specified parameters.
	 *
	 * @param encoder     the encoder ID
	 * @param turnMotor   the turn motor ID
	 * @param drive_motor the drive motor ID
	 * @param offset_rot  the offset rotation angle in radians
	 */
	SwerveModuleConfiguration(int encoder, int turnMotor, int drive_motor, double offset_rot) {
		ENCODER = encoder;
		TURN_MOTOR = turnMotor;
		DRIVE_MOTOR = drive_motor;
		offset = new Rotation2d(offset_rot);
	}
}
