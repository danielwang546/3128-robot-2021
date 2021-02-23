package org.team3128.athos.autonomous.deprecated;

import org.team3128.common.utility.units.Length;

import org.team3128.common.drive.SRXTankDrive;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class CmdDriveForward extends SequentialCommandGroup {
    public CmdDriveForward() {
        SRXTankDrive drive = SRXTankDrive.getInstance();

        // addSequential(drive.new CmdMoveDistance(MoveEndMode.EITHER, 100, 100, true,
        // .75, true, 10000));
        addCommands(drive.new CmdDriveStraight(100 * Length.in, .5, 10000).withTimeout(10));
        // addSequential(drive.new CmdMoveDistance(MoveEndMode.BOTH, -254*Length.cm,
        // -254*Length.cm, true, 1, false, 10000));
        // drive.getLeftMotors().set(ControlMode.PercentOutput, -100);
        // drive.getRightMotors().set(ControlMode.PercentOutput, -100);
    }
}