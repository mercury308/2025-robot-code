package frc.robot.subsystems.vision;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Robot;
import frc.robot.RobotState;
import frc.robot.constants.LimelightConfiguration;

public class VisionSubsystem extends SubsystemBase{
        private final VisionIO io;
        private final LimelightConfiguration config;    
        private final LimelightInputsAutoLogged inputs = new LimelightInputsAutoLogged();    

        private AprilTagFieldLayout field;
        private Pose2d robotToApriltag = new Pose2d();
        private double mt2Timestamp = 0.0;
        private boolean doRejectUpdate = false;


        public VisionSubsystem(VisionIO _io, LimelightConfiguration _config){
                io = _io;
                config = _config;
                try {
                        field = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
                } catch (Exception e) {
                        System.out.println("COULDNT FIND APRILTAG FIELD LAYOUT");
                        e.printStackTrace();
                }       
                LimelightHelpers.SetRobotOrientation(config.Name, RobotState.getPose().getRotation().getDegrees(), 0,0,0,0,0);
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

        public Optional<Double> getLastMT2Timestamp(){
                return Optional.of(this.mt2Timestamp);
        }
        

}
