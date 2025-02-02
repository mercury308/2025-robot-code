package frc.robot;

import frc.robot.commands.drive.DefaultDrive;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.commands.drive.AlignToReef;
import frc.robot.util.LocalADStarAK;

import java.util.Set;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.commands.drive.AlignToReef;
import frc.robot.commands.drive.DefaultDrive;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.subsystems.vision.PhotonVision;
import frc.robot.util.LocalADStarAK;

public class RobotContainer {
	public static SwerveSubsystem drive = new SwerveSubsystem();
	public static IMU imu = new IMU();
	public static PhotonVision photon = new PhotonVision();

	public static CommandJoystick left_js = new CommandJoystick(4);
	public static CommandJoystick right_js = new CommandJoystick(3);
	public static CommandJoystick ds = new CommandJoystick(2);

	public static LoggedDashboardChooser<Command> autoChooser;

	public RobotContainer() {}

	public static void initSubsystems() {

		drive.setDefaultCommand(new DefaultDrive(() -> left_js.getY(), () -> left_js.getX(), () -> -right_js.getX()));
		drive.init(new Pose2d());

		Pathfinding.setPathfinder(new LocalADStarAK());
    		autoChooser = new LoggedDashboardChooser<>("Auto Routine", AutoBuilder.buildAutoChooser());

		autoChooser.addOption("Goto Tag 20", new AlignToReef(20));
		autoChooser.addOption("Goto Tag 21", new AlignToReef(21));
		autoChooser.addOption("Goto Tag 22", new AlignToReef(22));

		configureBindings();
	}

	private static void configureBindings() {
		// right_js.button(4).onTrue(new AlignToReef(21));
		right_js.button(4).onTrue(new DeferredCommand(() -> autoChooser.get(), Set.of(drive)));

	}

	public Command getAutonomousCommand() {
		return autoChooser.get();
	}
}
