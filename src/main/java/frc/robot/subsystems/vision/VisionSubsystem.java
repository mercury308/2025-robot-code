package frc.robot.subsystems.vision;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import static frc.robot.RobotContainer.drive;
import static frc.robot.RobotContainer.imu;
import frc.robot.constants.LimelightConfiguration;
public class VisionSubsystem extends SubsystemBase{
        private LimelightIO io;
        private LimelightConfiguration config;    
        private final LimelightInputsAutoLogged inputs = new LimelightInputsAutoLogged();    

        private AprilTagFieldLayout field;
        private Pose2d robotToApriltag = new Pose2d();
        private double mt2Timestamp = 0.0;
        private boolean doRejectUpdate = false;

        public void init(){;
                config = new LimelightConfiguration();
                io = new LimelightIO(config.Name);  
                LimelightHelpers.SetRobotOrientation(config.Name, drive.getPose().getRotation().getDegrees(), 0,0,0,0,0);
        }        


        @Override

        public void periodic(){
                io.updateInputs(inputs);
                Logger.processInputs(config.Name, inputs);

                LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(config.Name);
                if(Math.abs(imu.getAngularVelocity()) > 720){
                        doRejectUpdate = true;
                }else if(!hasTarget()){
                        doRejectUpdate = true;
                }

                if(!doRejectUpdate){
                        robotToApriltag = mt2.pose;
                        mt2Timestamp = mt2.timestampSeconds;
                        drive.addLimelightMeasurement(robotToApriltag, mt2Timestamp);
                }
                

        }

        public boolean hasTarget(){
                return inputs.hasTarget;
        }

        public double getYawRadians(){
                return inputs.yaw;
        }

        public double getPitchRadians(){
                return inputs.pitch;
        }

        public Optional<Pose2d> getEstimatePose(){
                return Optional.of(this.robotToApriltag);
        }

        
}
