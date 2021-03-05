package org.team3128.athos.autonomous.deprecated;

import org.team3128.common.utility.enums.Direction;
import org.team3128.common.utility.units.Length;

import org.team3128.common.drive.SRXTankDrive;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class CmdArcTurnTest extends SequentialCommandGroup {
    public CmdArcTurnTest() {
        SRXTankDrive drive = SRXTankDrive.getInstance();

        addCommands(drive.new CmdArcTurn(36 * Length.in, 90, Direction.LEFT, .75, 10000).withTimeout(10));
    }

}