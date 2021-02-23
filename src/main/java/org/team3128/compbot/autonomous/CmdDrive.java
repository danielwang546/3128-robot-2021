package org.team3128.compbot.autonomous;

import org.team3128.compbot.subsystems.Constants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import org.team3128.compbot.subsystems.Arm.ArmState;
import org.team3128.compbot.subsystems.FalconDrive;
import org.team3128.common.utility.Log;
import edu.wpi.first.wpilibj.Timer;

import java.util.HashSet;
import java.util.Set;

import org.team3128.common.drive.DriveSignal;

public class CmdDrive implements Command {

    private Set<Subsystem> requirements;

    FalconDrive drive;
    double timeoutMs, startTime;

    public CmdDrive(FalconDrive drive, double timeoutMs) {

        this.requirements = new HashSet<Subsystem>();

        this.drive = drive;
        this.requirements.add(drive);

        this.timeoutMs = timeoutMs;
        
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return requirements;
    }

    @Override
    public void initialize() {
        drive.setWheelPower(new DriveSignal(0.3, 0.3));
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public boolean isFinished() {
        if (((Timer.getFPGATimestamp() - startTime) >= timeoutMs)) {
            return true;
        } else {
            return false;
        }
    }

    @Override
    public void end(boolean interrupted) {
        drive.setWheelPower(new DriveSignal(0, 0));
    }
}