package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.epilogue.Logged;

public interface VisionIO {

        @AutoLog
        public static class LimelightInputs{

                // double representing time since last update to IO

                public double lastTimeStamp = 0.0;

                // boolean representing whether or not target is in sight

                public boolean hasTarget = false;
        
                // double representing pitch from camera lens to target in radians

                public double pitch = 0.0;

                // double representing yaw from camera lens to target in radians

                public double yaw = 0.0;

                // int representing fiducial ID of target 

                public int iD = -1;

                public double[] pose;

        }

        public default void updateInputs(LimelightInputs inputs){}

}
