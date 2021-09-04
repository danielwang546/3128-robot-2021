package org.team3128.grogu.commands;

import edu.wpi.first.wpilibj2.command.*;

import org.team3128.common.utility.units.Length;
import org.team3128.common.utility.units.Angle;
import org.team3128.common.utility.math.Pose2D;
import org.team3128.common.utility.math.Rotation2D;
import org.team3128.common.drive.Drive;
import org.team3128.common.hardware.limelight.Limelight;
import org.team3128.common.drive.DriveCommandRunning;
import org.team3128.grogu.commands.*;
import org.team3128.grogu.subsystems.Constants;
import org.team3128.grogu.subsystems.*;
import com.kauailabs.navx.frc.AHRS;

public class CmdBallIntake extends SequentialCommandGroup {

    public CmdBallIntake(FalconDrive drive, Hopper hopper, AHRS ahrs, Limelight ballLimelight, DriveCommandRunning driveCmdRunning) {       
        addCommands(
            new ParallelCommandGroup(
                new RunCommand(() -> hopper.runIntake()),
                new CmdBallPursuit(ahrs, ballLimelight, driveCmdRunning,  0.472441 * Constants.MechanismConstants.inchesToMeters, Constants.VisionConstants.BALL_PID, 0, 2.5*Length.ft, 0.6666666666666666666666 * Length.ft, Constants.VisionConstants.BLIND_BALL_PID,20 * Angle.DEGREES)
                    //new CmdStreamUpdate(bottomLimelight, topLimelight, useBottom)
            ),
            new RunCommand(() -> hopper.stopIntake())
        );
    }
}