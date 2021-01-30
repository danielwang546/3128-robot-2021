package org.team3128.athos.autonomous.deprecated;

import org.team3128.common.utility.datatypes.PIDConstants;

import org.team3128.common.drive.SRXTankDrive;
import org.team3128.common.hardware.limelight.Limelight;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.team3128.common.hardware.gyroscope.Gyro;

public class CmdDynamicAdjustTest extends SequentialCommandGroup {
    /**
     * 
     * @param gyro
     * @param limelight
     */
    public CmdDynamicAdjustTest(Gyro gyro, Limelight limelight) {
        SRXTankDrive drive = SRXTankDrive.getInstance();
        PIDConstants visionPID = new PIDConstants(0, 0.0005, 0, 0.00009);
        addCommands(drive.new CmdTargetAlignSimple(gyro, limelight, 0.3, visionPID, 10000).withTimeout(10));
    }
}