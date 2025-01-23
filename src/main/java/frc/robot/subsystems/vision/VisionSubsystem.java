package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.LimelightConfiguration;

public class VisionSubsystem extends SubsystemBase{
        private final VisionIO io;
        private final LimelightConfiguration config;        

        public VisionSubsystem(VisionIO _io, LimelightConfiguration _config){
                io = _io;
                config = _config;
        }

        private Pose2d cameraToApriltag = new Pose2d();
        

        @Override

        public void periodic(){
                io.updateInputs();
                Logger.processInputs(config.Name)
        }


}
