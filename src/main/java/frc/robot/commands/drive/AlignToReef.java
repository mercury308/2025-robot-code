package frc.robot.commands.drive;

import static frc.robot.RobotContainer.*;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.util.Util;
import edu.wpi.first.math.kinematics.ChassisSpeeds;



public class AlignToReef extends Command{
    
    // TODO: Adjust PID gains

    private PIDController xPID = new PIDController(1.6, 0, 0.5);
    private PIDController yPID = new PIDController(1.6, 0, 0.5);
    private PIDController wPID = new PIDController(1, 0, 0);

    private Optional<Pose2d> target_pose;
    private Optional<Pose2d> stored_pose;

    public AlignToReef(){
        addRequirements(drive);
        wPID.enableContinuousInput(0, 2*Math.PI); 
    }   

    // Method that will adjust the target position based on the robots position relative to the field when compared to the tags field relative position

    public Pose2d getAdjustedPose(Pose2d current, Pose2d target){
       Pose2d returnable;
       double X = target.getX(); 
       double Y = target.getY();
        if(current.getY() < target.getY()){
            Y -= 0.305;
        }else{
            Y += 0.305;
        }

        if(current.getX() < target.getX()){
            X -= 0.305;
        }else{
            X += 0.305;
        }

        returnable = new Pose2d(X, Y, current.getRotation());
        return returnable;
        
    }

    
    @Override
    public void execute() {
        target_pose = photon.getAprilTagPose();
        if(target_pose.isEmpty() && stored_pose.isEmpty()){ // For reliability, if not receiving new pose from PhotonVision, use previously saved pose if any as reference
            return;
        }else if(target_pose.isEmpty() && !stored_pose.isEmpty()){
            target_pose = stored_pose;
        }

        Pose2d current_pose = drive.getPose();

        Pose2d adj_target = getAdjustedPose(current_pose, target_pose.get());

        double curr_X = current_pose.getX();
        double curr_Y = current_pose.getY();

        double adj_X = adj_target.getX();
        double adj_Y = adj_target.getY();

        double curr_rot = current_pose.getRotation().getRadians();
        double  target_rot = Math.PI + target_pose.get().getRotation().getRadians();

        double vX = xPID.calculate(curr_X, adj_X); // Have PID adjust current translation to match target
        double vY = yPID.calculate(curr_Y, adj_Y);
        double vW = wPID.calculate(curr_rot, target_rot);

        Logger.recordOutput("/Odom/target pose", adj_target);

        drive.drive(
            ChassisSpeeds.fromFieldRelativeSpeeds(vX, vY, vW, drive.getPose().getRotation()) // drive with calculated velocities
        );
        stored_pose = target_pose; // store current pose for potential future reference

    }
    @Override
    public boolean isFinished(){

        if(target_pose.isEmpty()) return false;

        Pose2d current_pose = drive.getPose();
        double dist = Math.abs(current_pose.getTranslation().getDistance(getAdjustedPose(current_pose, target_pose.get()).getTranslation())); // Translational difference
        double angle_offset =  Math.abs(Util.convertAngle(current_pose.getRotation().getRadians()) - Util.convertAngle(target_pose.get().getRotation().getRadians())); // Angular difference


        if(dist <= 0.5 && angle_offset <= 1/(2*Math.PI)){
            return true;
        }

        return false;
		
	}

    @Override
    public void end(boolean interrupted){

        super.end(interrupted);
        drive.drive(new ChassisSpeeds());
        stored_pose = null;

    }

}
