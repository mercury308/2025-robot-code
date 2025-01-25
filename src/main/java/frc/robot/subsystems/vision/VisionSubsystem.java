package frc.robot.subsystems.vision;

import frc.robot.RobotContainer.*;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.LimelightConfiguration;

public class VisionSubsystem extends SubsystemBase{
        private final VisionIO io;
        private final LimelightConfiguration config;    
        private final LimelightInputsAutoLogged inputs = new LimelightInputsAutoLogged();    


        private AprilTagFieldLayout field;
        private Pose2d cameraToApriltag = new Pose2d();
        private double horizontalDistanceMeters = 0.;

        public VisionSubsystem(VisionIO _io, LimelightConfiguration _config){
                io = _io;
                config = _config;
                try {
                        field = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
                } catch (Exception e) {
                        System.out.println("COULDNT FIND APRILTAG FIELD LAYOUT");
                        e.printStackTrace();
                }       
        }        

        @Override

        public void periodic(){
                io.updateInputs(inputs);
                Logger.processInputs(config.Name, inputs);

                cameraToApriltag = null;
                horizontalDistanceMeters = Double.NaN;

                if(hasTarget()){
                        double _yaw = getYawRadians();
                        double _pitch = getPitchRadians();
                        Optional<Pose3d> tag_pose = field.getTagPose((int) inputs.iD);

                        if(!tag_pose.isEmpty()){

                                Pose3d pose = tag_pose.get();
                                double tag_z = pose.getZ() - config.LimelightHeightOffsetMeters;
                                double tag_y = pose.getY() - config.LimelightLengthOffsetMeters;
                                double tag_x = pose.getX() - config.LimelightWidthOffsetMeters;

                                horizontalDistanceMeters = tag_z/Math.tan(_pitch);

                                cameraToApriltag = new Pose2d(
                                        horizontalDistanceMeters * Math.cos(_yaw),
                                        horizontalDistanceMeters * Math.sin(_yaw),
                                        new Rotation2d(_yaw)
                                );



                        }

                        drive.pose_est.addVisionMeasurement(cameraToApriltag);


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

        public double getHorizontalDistance(){
                return horizontalDistanceMeters;
        }


}
