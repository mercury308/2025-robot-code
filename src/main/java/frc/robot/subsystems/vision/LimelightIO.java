package frc.robot.subsystems.vision;
import java.util.EnumSet;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;


public class LimelightIO implements VisionIO{

        private NetworkTableEntry _tx;
        private NetworkTableEntry _ty;
        private NetworkTableEntry _id;
        private NetworkTableEntry _pose;
        private NetworkTableEntry _validEntry;

        private double lastTimeStamp = 0.0;
        private double yawRads = 0.0;
        private double pitchRads = 0.0;
        private double tagID = -1;
        private boolean has_taget = false;
        private double[] pose = new double[6];

        public LimelightIO(String limelight){
                NetworkTableInstance table = NetworkTableInstance.getDefault();

                _validEntry = table.getTable(limelight).getEntry("tv");
                _pose = table.getTable(limelight).getEntry("botpose_wpiblue");
                _tx = table.getTable(limelight).getEntry("tx");
                _ty = table.getTable(limelight).getEntry("ty");
                _id = table.getTable(limelight).getEntry("tid");
                
                DoubleSubscriber pipelineLatencySub = table.getTable(limelight).getDoubleTopic("tl").subscribe(0.0);
                DoubleSubscriber captureLatencySub = table.getTable(limelight).getDoubleTopic("cl").subscribe(0.0);
                table.addListener(pipelineLatencySub, EnumSet.of(NetworkTableEvent.Kind.kValueAll), networkTableEvent -> {

                        var latencies = (pipelineLatencySub.getAtomic().value + captureLatencySub.getAtomic().value)/1000;
                        double timeStamp = Timer.getFPGATimestamp() - latencies;

                        synchronized(LimelightIO.this){
                                this.lastTimeStamp = timeStamp;
                                has_taget = _validEntry.getDouble(0.0) == 1.0;
                                yawRads = -Units.degreesToRadians(_tx.getDouble(0.0));
                                pitchRads = Units.degreesToRadians(_ty.getDouble(0.0));
                                tagID = _id.getDouble(-1.0);
                                pose = _pose.getDoubleArray(new double[6]);
                        }


                });

        }

        @Override

        public synchronized void updateInputs(LimelightInputs inputs){
                inputs.lastTimeStamp = this.lastTimeStamp;
                inputs.pitch = this.pitchRads;
                inputs.yaw = this.yawRads;
                inputs.hasTarget = this.has_taget;
                inputs.pose = this.pose;
                inputs.iD = (int) this.tagID;
        }


}
