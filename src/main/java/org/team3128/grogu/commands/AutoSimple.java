package org.team3128.grogu.commands;

import org.team3128.common.drive.DriveCommandRunning;
import org.team3128.common.hardware.limelight.Limelight;
import org.team3128.grogu.subsystems.FalconDrive;
import org.team3128.grogu.subsystems.PathFinding;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoSimple extends SequentialCommandGroup {

    public AutoSimple(Limelight shooterLimelight, DriveCommandRunning cmdRunning, double goalHorizontalOffset, PathFinding m_robotContainer, FalconDrive mRobotDrive) {       
        addCommands(
            new CmdAlignShoot(shooterLimelight, cmdRunning, goalHorizontalOffset, 26),
            m_robotContainer.getAutonomousCommandSimple(mRobotDrive)
            );
    }
}