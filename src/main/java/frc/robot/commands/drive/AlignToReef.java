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
    
    private PIDController xPID = new PIDController(1.6, 0, 0.5);
    private PIDController yPID = new PIDController(1.6, 0, 0.5);
    private PIDController wPID = new PIDController(1, 0, 0);

    private Optional<Pose2d> target_pose;
    private Optional<Pose2d> stored_pose;

    public AlignToReef(){
        addRequirements(drive);
        wPID.enableContinuousInput(0, 2*Math.PI);
    }   


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
        if(target_pose.isEmpty() && stored_pose.isEmpty()){
            //System.out.println("Photon vision returned null");
            return;
        }else if(target_pose.isEmpty() && !stored_pose.isEmpty()){
            target_pose = stored_pose;
        }

        Pose2d current_pose = drive.getPose();

        Pose2d adj_target = getAdjustedPose(current_pose, target_pose.get());

        double vX = xPID.calculate(current_pose.getX(), adj_target.getX());
        double vY = yPID.calculate(current_pose.getY(), adj_target.getY());
        double vW = wPID.calculate(current_pose.getRotation().getRadians(), Math.PI + target_pose.get().getRotation().getRadians());

        Logger.recordOutput("/Odom/target pose", adj_target);
        //System.out.println("TARGET X: " + adj_target.getX() + " TARGET Y: " + adj_target.getY());
        //System.out.println("CURRENT X " + current_pose.getX() + " CURRENT Y: " + current_pose.getY());
        //System.out.println(current_pose.getTranslation().getDistance(adj_target.getTranslation()));


        drive.drive(
            ChassisSpeeds.fromFieldRelativeSpeeds(vX, vY, vW, drive.getPose().getRotation())
        );
        stored_pose = target_pose;

    }
    @Override
    public boolean isFinished(){

        if(target_pose.isEmpty()) return false;

        Pose2d current_pose = drive.getPose();
        
        if(Math.abs(current_pose.getTranslation().getDistance(getAdjustedPose(current_pose, target_pose.get()).getTranslation())) <= 0.5
        && Math.abs(Util.convertAngle(current_pose.getRotation().getRadians()) - Util.convertAngle(target_pose.get().getRotation().getRadians())) <= 5/(2*Math.PI)){
            return true;
        }
        
        return false
        ;
		
	}

}
