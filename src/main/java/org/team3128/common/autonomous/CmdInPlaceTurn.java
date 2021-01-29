// Authors: Mika, Thomas, Mason, Sohan
package org.team3128.common.autonomous;

import org.team3128.common.drive.Drive;
import org.team3128.common.drive.DriveSignal;
import org.team3128.common.hardware.gyroscope.Gyro;
import org.team3128.compbot.subsystems.FalconDrive;

import edu.wpi.first.wpilibj.command.CommandGroup;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.Set;
import java.util.HashSet;

public class CmdInPlaceTurn implements Command {
    double startAngle, angle, timeoutMs, power;
    FalconDrive drive;
    Gyro gyro;

    private Set<Subsystem> requirements;

    public CmdInPlaceTurn(FalconDrive drive, Gyro gyro, double angle, double power, double timeoutMs) {
        this.requirements = new HashSet<Subsystem>();
        this.drive = drive;
        this.requirements.add(drive);
        this.angle = angle;
        this.gyro = gyro;
        this.power = power;
        this.timeoutMs = timeoutMs;
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return requirements;
    }

    @Override
    public void initialize() {
        startAngle = drive.getAngle();
        if (angle > 0) {
            drive.setWheelPower(new DriveSignal(-power, power));
        }
        else if (angle < 0) {
            drive.setWheelPower(new DriveSignal(power, -power));
        }
    }

    @Override
    public boolean isFinished() {
        return drive.getAngle() - startAngle >= angle;
    }

    @Override
    public void end(boolean interrupted) {
        drive.setWheelPower(new DriveSignal(0, 0));
    }
}