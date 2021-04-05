package org.team3128.grogu.commands;

import org.team3128.common.drive.DriveCommandRunning;
import org.team3128.common.drive.DriveSignal;
import org.team3128.common.hardware.limelight.LEDMode;
import org.team3128.common.hardware.limelight.Pipeline;
import org.team3128.common.hardware.limelight.Limelight;
import org.team3128.common.hardware.limelight.LimelightData;
import org.team3128.common.hardware.limelight.LimelightKey;
import org.team3128.common.hardware.limelight.StreamMode;
import org.team3128.common.hardware.gyroscope.Gyro;
import org.team3128.common.narwhaldashboard.NarwhalDashboard;
import org.team3128.common.utility.Log;
import org.team3128.common.utility.RobotMath;
import org.team3128.common.utility.datatypes.PIDConstants;
import org.team3128.common.utility.units.Angle;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command; 
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.team3128.grogu.subsystems.*;

import com.kauailabs.navx.frc.AHRS;

import java.util.Set;
import java.util.HashSet;

import org.team3128.grogu.commands.*;

public class CmdAlignShoot implements Command {
    FalconDrive drive;
    Shooter shooter;
    Hopper hopper;
    boolean gotDistance = false;


    Limelight limelight;

    double decelerationStartDistance, decelerationEndDistance;
    DriveCommandRunning cmdRunning;

    private PIDConstants visionPID;

    private double goalHorizontalOffset;

    private double currentHorizontalOffset;

    private double currentError, previousError;
    private double currentTime, previousTime;

    private double feedbackPower;

    private double leftPower, rightPower;

    private double desiredRPM;
    private double effective_distance;

    private Set<Subsystem> requirements;

    private Command hopperShoot, organize;

    int targetFoundCount;
    int plateauCount;

    int numBallsShot;
    int numBallsToShoot;

    private enum HorizontalOffsetFeedbackDriveState {
        SEARCHING, FEEDBACK; // , BLIND;
    }

    private HorizontalOffsetFeedbackDriveState aimState = HorizontalOffsetFeedbackDriveState.SEARCHING;

    public CmdAlignShoot(Limelight limelight, DriveCommandRunning cmdRunning, double goalHorizontalOffset, int numBallsToShoot) {
        this.shooter = Shooter.getInstance();
        this.hopper = Hopper.getInstance();
        this.drive = FalconDrive.getInstance();

        this.requirements = new HashSet<Subsystem>();
        this.requirements.add(drive);
        this.requirements.add(shooter);
        this.requirements.add(hopper);
        
        this.limelight = limelight;
        this.visionPID = Constants.VisionConstants.VISION_PID;

        this.cmdRunning = cmdRunning;

        this.goalHorizontalOffset = goalHorizontalOffset;

        this.numBallsToShoot = numBallsToShoot;
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return requirements;
    }

    @Override
    public void initialize() {
        limelight.setLEDMode(LEDMode.ON);
        cmdRunning.isRunning = false;
        // TODO: prob not helpful but sets hopper to shooting
        //hopper.setAction(Hopper.ActionState.SHOOTING);
        Log.info("CmdAlignShoot", "initialized limelight, aren't I cool!");
    }

    @Override
    public void execute() {
        //Log.info("CmdAlignShoot", "Running one loop of execute");
        switch (aimState) {
            case SEARCHING:
                NarwhalDashboard.put("align_status", "searching");
                Log.info("CmdAlignShoot", "Searching...");
                if (limelight.hasValidTarget()) {
                    targetFoundCount += 1;
                } else {
                    targetFoundCount = 0;
                }

                if (targetFoundCount > 5) {
                    Log.info("CmdAlignShoot", "Target found.");
                    Log.info("CmdAlignShoot", "Switching to FEEDBACK...");
                    LimelightData initData = limelight.getValues(Constants.VisionConstants.SAMPLE_RATE);

                    SmartDashboard.putNumber("ty", initData.ty());

                    currentHorizontalOffset = limelight.getValue(LimelightKey.HORIZONTAL_OFFSET, Constants.VisionConstants.SAMPLE_RATE);//5);

                    previousTime = RobotController.getFPGATime();
                    previousError = goalHorizontalOffset - currentHorizontalOffset;

                    cmdRunning.isRunning = true;

                    aimState = HorizontalOffsetFeedbackDriveState.FEEDBACK;
                }

                break;

            case FEEDBACK:
                NarwhalDashboard.put("align_status", "feedback");
                cmdRunning.isRunning = false;
                if (!limelight.hasValidTarget()) {
                    Log.info("CmdAlignShoot", "No valid target.");
                    Log.info("CmdAlignShoot", "Returning to SEARCHING...");

                    aimState = HorizontalOffsetFeedbackDriveState.SEARCHING;

                } else {

                    if (!gotDistance) {
                        LimelightData initData = limelight.getValues(Constants.VisionConstants.SAMPLE_RATE);

                        //shooter.setState(Shooter.ShooterState.MID_RANGE);

                        SmartDashboard.putNumber("ty", initData.ty());


                        gotDistance = true;
                    }

                    currentHorizontalOffset = limelight.getValue(LimelightKey.HORIZONTAL_OFFSET, Constants.VisionConstants.SAMPLE_RATE);

                    currentTime = RobotController.getFPGATime();
                    currentError = goalHorizontalOffset - currentHorizontalOffset;

                    /**
                     * PID feedback loop for the left and right powers based on the horizontal
                     * offset errors.
                     */
                    feedbackPower = 0;

                    feedbackPower += visionPID.kP * currentError;
                    feedbackPower += visionPID.kD * (currentError - previousError) / (currentTime - previousTime);

                    leftPower = RobotMath.clamp(-feedbackPower, -1, 1);
                    rightPower = RobotMath.clamp(feedbackPower, -1, 1);

                    SmartDashboard.putNumber("Shooter Power", leftPower);

                    double leftSpeed = leftPower * Constants.DriveConstants.DRIVE_HIGH_SPEED;
                    double rightSpeed = rightPower * Constants.DriveConstants.DRIVE_HIGH_SPEED;
                    
                    drive.setWheelPower(new DriveSignal(leftPower, rightPower));

                    previousTime = currentTime;
                    previousError = currentError;
                }
                if ((Math.abs(currentError) < Constants.VisionConstants.TX_THRESHOLD)) {
                    plateauCount++;
                    if (plateauCount > 5) {
                        shooter.isAligned = true;
                        Log.info("Cmd Align Shoot","SHOOTY TIME!!!");
                    }
                } else {
                    shooter.isAligned = false;
                }
                break;
        }
    }

    @Override
    public boolean isFinished() {
        // if (hopper.getBallCount() == 0|| numBallsShot >= numBallsToShoot) {
        // return true;
        // } else {
        // return false;
        // }
        return shooter.isAligned;
    }

    @Override
    public void end(boolean interrupted) {
        limelight.setLEDMode(LEDMode.OFF);
        drive.stopMovement();

        Log.info("CmdAlignShoot", "Command Finished.");
        if (interrupted)
            Log.info("CmdAlignShoot", "Command interru-");
        //hopper.setAction(Hopper.ActionState.ORGANIZING);
    }
}