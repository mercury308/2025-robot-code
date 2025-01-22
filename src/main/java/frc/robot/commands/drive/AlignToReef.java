package frc.robot.commands.drive;

import static frc.robot.RobotContainer.*;
import static frc.robot.constants.Constants.RobotConstants.*;
import static frc.robot.util.Util.*;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.util.Util;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.trajectory.TrajectoryConfig;



public class AlignToReef extends Command{
    
    // TODO: Adjust PID gains

    private PIDController xPID = new PIDController(3, 0.1, 0.6);
    private PIDController yPID = new PIDController(5, 0.2, 0.5);
    private PIDController wPID = new PIDController(1.5, 0., 0.4);

    private Optional<Pose2d> target_pose;
    private Optional<Pose2d> stored_pose = Optional.empty();

    private Pose2d adj_pose;
    
    public AlignToReef(){
        addRequirements(drive); 
        wPID.enableContinuousInput(0, 2*Math.PI); 
    }   

    @Override 
    public void initialize(){
        target_pose = photon.getAprilTagPose();
        if(target_pose.isEmpty() && stored_pose.isEmpty()){ // For reliability, if not receiving new pose from PhotonVision, use previously saved pose if any as reference
            return;
        }else if(target_pose.isEmpty() && !stored_pose.isEmpty()){
            //System.out.println(drive.getPose().getTranslation().getDistance(stored_pose.get().getTranslation()));
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
        double  target_rot = target_pose.get().getRotation().getRadians();

        double vX = MathUtil.clamp(xPID.calculate(curr_X, adj_X), -1, 1); // Have PID adjust current translation to match target
        double vY = MathUtil.clamp(yPID.calculate(curr_Y, adj_Y), -1, 1); // Have PID adjust current translation to match target
        double vW = wPID.calculate(curr_rot, target_rot);

    
        Logger.recordOutput("/Odom/adjusted pose", adj_pose);
        Logger.recordOutput("/Odom/adjusted_pose/x", adj_X);
        Logger.recordOutput("/Odom/adjusted_pose/y", adj_Y);
        Logger.recordOutput("/Odom/adjusted_pose/w", adj_pose.getRotation().getRadians());


       // Logger.recordOutput("/Odom/target pose", target_pose.get());
       // Logger.recordOutput("/Swerve/distance", current_pose.getTranslation().getDistance(adj_pose.getTranslation()));
       if(Math.abs(current_pose.getY() - adj_pose.getY()) > 0.04){
            drive.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(vX, 0, vW, drive.getPose().getRotation()) // drive with calculated velocities
            );
       }else if(Math.abs(current_pose.getX() - adj_pose.getX()) > 0.04){
            drive.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(0, vY, 0, drive.getPose().getRotation())
            );
       }
        stored_pose = Optional.of(adj_pose); // store current pose for potential future reference
    }
    
    @Override
    public boolean isFinished(){

        if(target_pose.isEmpty()){
            System.out.println("NO TARGET, REPOSITION AND TRY AGAIN");
            return true;
        }

        Pose2d current_pose = drive.getPose();
        double dX = Math.abs(current_pose.getX() - adj_pose.getX());
        double dY = Math.abs(current_pose.getY() - adj_pose.getY()); // Translational difference
       
        double angle_offset =  Math.abs(
            Util.convertAngle(
                current_pose
                .getRotation()
                .getRadians()) 
            - Util.convertAngle(
                adj_pose
                .getRotation()
                .getRadians())); // Angular difference


        if(dX < 0.04 && dY < 0.04 && angle_offset <= (4*Math.PI)/360){
            System.out.println("Aligned");
            return true;
        }
        
       System.out.println("Still working on it " + " Angular Dist: " + angle_offset);
        return false;
		
	}

    @Override
    public void end(boolean interrupted){
        super.end(interrupted);
        drive.drive(new ChassisSpeeds());
        stored_pose = Optional.empty();

    }

}
