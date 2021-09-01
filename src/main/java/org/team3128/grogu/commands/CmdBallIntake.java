package org.team3128.grogu.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import org.team3128.common.utility.math.Pose2D;
import org.team3128.common.utility.math.Rotation2D;
import org.team3128.common.drive.Drive;
import org.team3128.common.hardware.limelight.Limelight;
import org.team3128.common.drive.DriveCommandRunning;
import org.team3128.compbot.autonomous.*;
import org.team3128.compbot.commands.*;
import org.team3128.compbot.subsystems.Constants.VisionConstants;
import org.team3128.compbot.subsystems.*;
import com.kauailabs.navx.frc.AHRS;

public class CmdBallIntake extends CommandGroup {

    public AutoPriority(FalconDrive drive, Shooter shooter, Arm arm, Hopper hopper, AHRS ahrs, Limelight limelight, DriveCommandRunning cmdRunning, double timeoutMs) {       
        addCommands(
            addParallel(
                CmdBallPursuit(ahrs, ballLimelight, driveCmdRunning,  0.472441 * Constants.MechanismConstants.inchesToMeters, Constants.VisionConstants.BALL_PID, 0, 2.5*Length.ft, 0.6666666666666666666666 * Length.ft, Constants.VisionConstants.BLIND_BALL_PID,42 * Angle.DEGREES)
                    //new CmdStreamUpdate(bottomLimelight, topLimelight, useBottom)
            );
        );
    }
}