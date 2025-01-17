package frc.robot.commands.drive;

import static frc.robot.RobotContainer.*;
import static frc.robot.constants.Constants.RobotConstants.*;

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



public class LateralAlign extends Command{
    
    private PIDController yPID = new PIDController(1.5, 0., 0);

    private double phi = Math.PI/6;

    private Optional<Pose2d> target_pose;
    private Optional<Pose2d> stored_pose = Optional.empty();
    private Pose2d adjPose2d;

    public LateralAlign(){
        addRequirements(drive);
    }   

    @Override 
    public void initialize(){
        target_pose = photon.getAprilTagPose();
        if(target_pose.isEmpty() && stored_pose.isEmpty()){ // For reliability, if not receiving new pose from PhotonVision, use previously saved pose if any as reference
            return;
        }else if(target_pose.isEmpty() && !stored_pose.isEmpty()){
            target_pose = stored_pose;
        }

        adjPose2d = target_pose.get().transformBy(new Transform2d(new Translation2d(), new Rotation2d(Math.PI)));

    }


    @Override
    public void execute() {


        Pose2d current_pose = drive.getPose();
        
        double current_x = current_pose.getX();
        double target_x = adjPose2d.getX();

        double current_y = current_pose.getY();
        double target_y = adjPose2d.getY();

        phi = Math.atan2(Math.abs(target_y-current_y),Math.abs(target_x-current_x));
        
        
        drive.drive(
            ChassisSpeeds.fromFieldRelativeSpeeds(0,0 , 0, drive.getPose().getRotation()) // drive with calculated velocities
        );
        stored_pose = target_pose; // store current pose for potential future reference
    }
    
    @Override
    public boolean isFinished(){
        if(target_pose.isEmpty()) return false;
        
        if(phi <= Math.PI/6){
            System.out.println("Within bearing range");
            return true;
        }
        return false;
	}

    @Override
    public void end(boolean interrupted){
        super.end(interrupted);
        drive.drive(new ChassisSpeeds());
        stored_pose = Optional.empty();

    }

}

