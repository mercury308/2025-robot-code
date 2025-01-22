package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class ModuleIOInputsAutoLogged extends ModuleIO.ModuleIOInputs implements LoggableInputs, Cloneable {
	@Override
	public void toLog(LogTable table) {
		table.put("DrivePositionRad", drivePositionRad);
		table.put("DriveVelocityMetersPerSec", driveVelocityMetersPerSec);
		table.put("DriveAppliedVolts", driveAppliedVolts);
		table.put("DriveCurrentAmps", driveCurrentAmps);
		table.put("TurnAbsolutePosition", turnAbsolutePosition);
		table.put("TurnAbsolutePositionRad", turnAbsolutePositionRad);
		table.put("TurnPosition", turnPosition);
		table.put("TurnVelocityRadPerSec", turnVelocityRadPerSec);
		table.put("TurnAppliedVolts", turnAppliedVolts);
		table.put("TurnCurrentAmps", turnCurrentAmps);
		table.put("TargetRad", targetRad);
		table.put("TargetVel", targetVel);
		table.put("CompensatedTargetVel", compensatedTargetVel);
	}

	@Override
	public void fromLog(LogTable table) {
		drivePositionRad = table.get("DrivePositionRad", drivePositionRad);
		driveVelocityMetersPerSec = table.get("DriveVelocityMetersPerSec", driveVelocityMetersPerSec);
		driveAppliedVolts = table.get("DriveAppliedVolts", driveAppliedVolts);
		driveCurrentAmps = table.get("DriveCurrentAmps", driveCurrentAmps);
		turnAbsolutePosition = table.get("TurnAbsolutePosition", turnAbsolutePosition);
		turnAbsolutePositionRad = table.get("TurnAbsolutePositionRad", turnAbsolutePositionRad);
		turnPosition = table.get("TurnPosition", turnPosition);
		turnVelocityRadPerSec = table.get("TurnVelocityRadPerSec", turnVelocityRadPerSec);
		turnAppliedVolts = table.get("TurnAppliedVolts", turnAppliedVolts);
		turnCurrentAmps = table.get("TurnCurrentAmps", turnCurrentAmps);
		targetRad = table.get("TargetRad", targetRad);
		targetVel = table.get("TargetVel", targetVel);
		compensatedTargetVel = table.get("CompensatedTargetVel", compensatedTargetVel);
	}

	public ModuleIOInputsAutoLogged clone() {
		ModuleIOInputsAutoLogged copy = new ModuleIOInputsAutoLogged();
		copy.drivePositionRad = this.drivePositionRad;
		copy.driveVelocityMetersPerSec = this.driveVelocityMetersPerSec;
		copy.driveAppliedVolts = this.driveAppliedVolts;
		copy.driveCurrentAmps = this.driveCurrentAmps.clone();
		copy.turnAbsolutePosition = this.turnAbsolutePosition;
		copy.turnAbsolutePositionRad = this.turnAbsolutePositionRad;
		copy.turnPosition = this.turnPosition;
		copy.turnVelocityRadPerSec = this.turnVelocityRadPerSec;
		copy.turnAppliedVolts = this.turnAppliedVolts;
		copy.turnCurrentAmps = this.turnCurrentAmps.clone();
		copy.targetRad = this.targetRad;
		copy.targetVel = this.targetVel;
		copy.compensatedTargetVel = this.compensatedTargetVel;
		return copy;
	}
}
