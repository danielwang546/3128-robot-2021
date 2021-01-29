package org.team3128.athos.autonomous.deprecated;

import org.team3128.common.utility.enums.Direction;
import org.team3128.common.utility.units.Length;
import org.team3128.common.hardware.limelight.Limelight;
import org.team3128.common.hardware.limelight.LimelightData;
import org.team3128.common.utility.RobotMath;

import org.team3128.common.drive.SRXTankDrive;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import org.team3128.common.utility.Log;

public class CmdHeadingThenArc extends SequentialCommandGroup {
    public CmdHeadingThenArc(Limelight limelight) {
        SRXTankDrive drive = SRXTankDrive.getInstance();
        LimelightData data = limelight.getValues(20);
        Direction inPlaceDir, arcDir;
        double x, z;
        x = Math.abs(data.x());
        z = Math.abs(data.z());
        double radius = (Math.pow(x, 2) + Math.pow(z, 2)) / (2 * x);
        double angleRobotToHorizontal = RobotMath.atan(z / x);
        double angleRadiusToHorizontal = RobotMath.acos((radius - x) / radius);

        double angleTargetToTangent = angleRobotToHorizontal - (90 - angleRadiusToHorizontal);

        if (data.x() < 0) {
            inPlaceDir = Direction.RIGHT;
            arcDir = Direction.LEFT;
        } else {
            inPlaceDir = Direction.LEFT;
            arcDir = Direction.RIGHT;
        }

        Log.info("radius", String.valueOf(radius));
        Log.info("inPlace angle", String.valueOf(angleTargetToTangent));
        Log.info("arc angle", String.valueOf(angleRadiusToHorizontal));

        // if we need to turn right but this angle is positive then we need to turn left
        addCommands(drive.new CmdInPlaceTurn(angleTargetToTangent, inPlaceDir, 0.75, 10000).withTimeout(10),
                    drive.new CmdArcTurn(radius * Length.in, (int) angleRadiusToHorizontal, arcDir, .75, 10000).withTimeout(10));
    }
}