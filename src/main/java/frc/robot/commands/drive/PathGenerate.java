package frc.robot.commands.drive;

import static frc.robot.constants.Constants.RobotConstants.SWERVE_MAXSPEED;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.spline.CubicHermiteSpline;
import edu.wpi.first.math.spline.Spline;
import edu.wpi.first.math.spline.PoseWithCurvature;

import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;

public class PathGenerate {
    public static void generateTrajectory(Translation2d start, Translation2d goal){
        List<Translation2d> control_points = new ArrayList<Translation2d>();
        // TODO: FIGURE OUT HOW TO GENERATE CONTROL POINTS
        
        /*  The start and goal are our starting and ending positions that are used as the two main spline control points
            The spline class will linearly interpolate between these points and the provided control points to generate a 
            smooth curve that becomes our trajectory. 
        */

        //TODO: Spline class is annoying and does not take Translation2d as input so need to convert to Spline's ControlVector type

        TrajectoryConfig config = new TrajectoryConfig(3.7,7.8);
        var trajectory = TrajectoryGenerator.generateTrajectory(
            start,
            control_points,
            goal,
            config);
    }
}
