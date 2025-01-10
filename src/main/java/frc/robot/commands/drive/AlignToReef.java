package frc.robot.commands.drive;

import static frc.robot.RobotContainer.*;

import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.util.Util;
import edu.wpi.first.math.kinematics.ChassisSpeeds;



public class AlignToReef extends Command{
    
    private PIDController xPID = new PIDController(0, 0, 0);
    private PIDController yPID = new PIDController(0, 0, 0);
    private PIDController wPID = new PIDController(0, 0, 0);

    private Optional<Pose2d> target_pose;

    public AlignToReef(){
        addRequirements(drive);
        wPID.enableContinuousInput(0, 2*Math.PI);
    }   
    
    @Override
    public void execute() {
        target_pose = photon.getAprilTagPose();
        if(target_pose.isEmpty()) return;

        Pose2d current_pose = drive.getPose();
        double vX = xPID.calculate(current_pose.getX(), target_pose.get().getX());
        double vY = yPID.calculate(current_pose.getY(), target_pose.get().getY());
        double vW = wPID.calculate(current_pose.getRotation().getRadians(), target_pose.get().getRotation().getRadians());

        MathUtil.clamp(vX, -1, 1);
        MathUtil.clamp(vY, -1, 1);
        MathUtil.clamp(vW, -1, 1);

        drive.drive(
            ChassisSpeeds.fromFieldRelativeSpeeds(vX, vY, vW, drive.getPose().getRotation())
        );


    }
    @Override
    public boolean isFinished(){

        if(target_pose.isEmpty()) return false;

        Pose2d current_pose = drive.getPose();
        
        if(current_pose.getTranslation().getDistance(target_pose.get().getTranslation()) < 0.05
        && Math.abs(Util.convertAngle(current_pose.getRotation().getRadians()) - Util.convertAngle(target_pose.get().getRotation().getRadians())) <= 1/(2*Math.PI)){
            return true;
        }
        
        return false;
		
	}

}
