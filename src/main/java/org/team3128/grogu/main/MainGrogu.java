package org.team3128.grogu.main;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import org.team3128.common.generics.RobotConstants;

import com.kauailabs.navx.frc.AHRS;


import org.team3128.common.NarwhalRobot;
import org.team3128.common.control.trajectory.Trajectory;
import org.team3128.common.control.trajectory.TrajectoryGenerator;
import org.team3128.common.control.trajectory.constraint.TrajectoryConstraint;
import org.team3128.common.drive.DriveCommandRunning;
import org.team3128.common.hardware.limelight.LEDMode;
import org.team3128.common.hardware.limelight.Limelight;
import org.team3128.common.hardware.gyroscope.NavX;
import org.team3128.common.utility.units.Angle;
import org.team3128.common.utility.units.Length;
import org.team3128.common.vision.CmdHorizontalOffsetFeedbackDrive;
import org.team3128.common.utility.Log;
import org.team3128.common.utility.RobotMath;
import org.team3128.common.utility.datatypes.PIDConstants;
import org.team3128.common.narwhaldashboard.NarwhalDashboard;
import org.team3128.common.listener.ListenerManager;
import org.team3128.common.listener.POVValue;
import org.team3128.common.listener.controllers.ControllerExtreme3D;
import org.team3128.common.listener.controltypes.Button;
import org.team3128.common.listener.controltypes.POV;
import org.team3128.common.hardware.motor.LazyCANSparkMax;
import org.team3128.common.utility.math.Pose2D;
import org.team3128.common.utility.math.Rotation2D;
import org.team3128.common.utility.test_suite.CanDevices;
import org.team3128.common.utility.test_suite.ErrorCatcherUtility;
import org.team3128.grogu.subsystems.*;
import org.team3128.grogu.commands.*;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import java.util.ArrayList;
import java.util.concurrent.*;

import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class MainGrogu extends NarwhalRobot {

    private DriveCommandRunning driveCmdRunning;

    private CmdBallIntake cmdBallIntake;
    private CmdBallPursuit cmdBallPursuit;

    // public StateTracker stateTracker = StateTracker.getInstance();

 
    // RobotTracker robotTracker = RobotTracker.getInstance();

    ExecutorService executor = Executors.newFixedThreadPool(6);
    CommandScheduler scheduler = CommandScheduler.getInstance();
    Thread auto;

    public Joystick joystickRight, joystickLeft;
    public ListenerManager listenerLeft, listenerRight;
    public AHRS ahrs;
    public static PowerDistributionPanel pdp;

    public NetworkTable table;
    public NetworkTable limelightTable;

    public double startTime = 0;

    public int reverse = 1;

    public String trackerCSV = "Time, X, Y, Theta, Xdes, Ydes";

    public ArrayList<Pose2D> waypoints = new ArrayList<Pose2D>();
    public Trajectory trajectory;

    public Limelight shooterLimelight, ballLimelight;
    public boolean inPlace = false;
    public boolean inPlace2 = false;

    public FalconDrive drive = FalconDrive.getInstance();
    public Hopper hopper = Hopper.getInstance();
    public Shooter shooter = Shooter.getInstance();
    public Sidekick sidekick = Sidekick.getInstance();

    public CmdAlignShoot alignCmd;

    public ErrorCatcherUtility errorCatcher;

    @Override
    protected void constructHardware() {

        //shooterLimelight.setLEDMode(LEDMode.OFF);
        //ballLimelight.setLEDMode(LEDMode.OFF);
        
        driveCmdRunning = new DriveCommandRunning();

        ahrs = drive.ahrs;

        //hopper.enable();

        joystickRight = new Joystick(1);
        listenerRight = new ListenerManager(joystickRight);
        addListenerManager(listenerRight);

        joystickLeft = new Joystick(0);
        listenerLeft = new ListenerManager(joystickLeft);
        addListenerManager(listenerLeft);

        // initialization of limelights

        //REVERSED
        ballLimelight = new Limelight("limelight-pog", -26.0, 0, 0, 30);
        shooterLimelight = new Limelight("limelight-sog", Constants.VisionConstants.BOTTOM_LIMELIGHT_ANGLE,
                Constants.VisionConstants.BOTTOM_LIMELIGHT_HEIGHT,
                Constants.VisionConstants.BOTTOM_LIMELIGHT_DISTANCE_FROM_FRONT, 14.5 * Length.in);
        drive.resetGyro();

        hopper.register();

        shooter.enable();
        shooter.setSetpointAndTolerance(0);
        sidekick.enable();
        sidekick.setState(Sidekick.ShooterState.DEFAULT);

        shooter.setState(Shooter.ShooterState.MID_RANGE);
        alignCmd = new CmdAlignShoot(shooterLimelight, driveCmdRunning, 0, 26);

        

        //errorCatcher = new ErrorCatcherUtility(CanChain, limelights, drive);
        /*
        NarwhalDashboard.addButton("ErrorCatcher", (boolean down) -> {
            if(down) {
                errorCatcher.testEverything();
            }
        });

        NarwhalDashboard.addButton("VelocityTester", (boolean down) -> {
            if(down) {
                errorCatcher.velocityTester();
            }
        });
        */
    }

    @Override
    protected void constructAutoPrograms() {
        //cmdBallPursuit = new CmdBallPursuit(ahrs, ballLimelight, driveCmdRunning,  0.472441 * Constants.MechanismConstants.inchesToMeters, Constants.VisionConstants.BALL_PID, 0, 2.5*Length.ft, 0.6666666666666666666666 * Length.ft, Constants.VisionConstants.BLIND_BALL_PID,42 * Angle.DEGREES);
        //scheduler.schedule(cmdBallPursuit);
        
        cmdBallIntake = new CmdBallIntake(drive, hopper, ahrs, ballLimelight, driveCmdRunning);

        NarwhalDashboard.addAuto("Find ball maybe", cmdBallIntake);

        //cmdBallPursuit = new CmdBallPursuit(drive, hopper, ahrs, ballLimelight, driveCmdRunning);

        //NarwhalDashboard.addAuto("pog", cmdBallPursuit);
    }

    @Override
    protected void setupListeners() {
        listenerRight.nameControl(ControllerExtreme3D.TWIST, "MoveTurn");
        listenerRight.nameControl(ControllerExtreme3D.JOYY, "MoveForwards");
        listenerRight.nameControl(ControllerExtreme3D.THROTTLE, "Throttle");
        listenerRight.nameControl( ControllerExtreme3D.TRIGGER, "Intake");

        listenerRight.nameControl(new Button(10), "MoveArmDown");
        listenerRight.nameControl(new Button(8), "MoveArmUp");

        listenerRight.nameControl(new Button(2), "Shoot");

        listenerRight.nameControl(new Button(3), "EmptyHopper");

        listenerRight.nameControl(new Button(7), "SetOverYonder");
        listenerRight.nameControl(new Button(9), "SetMiddling");
        listenerRight.nameControl(new Button(11), "SetIntimate");

        listenerRight.nameControl(new Button(12), "ResetBallCount");

        listenerLeft.nameControl(ControllerExtreme3D.TRIGGER, "REVERSE");

        listenerLeft.nameControl(new Button(12), "SetGreen");
        listenerLeft.nameControl(new Button(11), "SetYellow");
        listenerLeft.nameControl(new Button(9), "SetBlue");
        listenerLeft.nameControl(new Button(7), "SetRed");

        listenerRight.addMultiListener(() -> {
            if (driveCmdRunning.isRunning) {
                double horiz = 0.4  * listenerRight.getAxis("MoveTurn"); //-0.5
                double vert = -1.0 * reverse * listenerRight.getAxis("MoveForwards"); //-1.0
                double throttle = -1.0 * listenerRight.getAxis("Throttle"); // -1.0

                drive.arcadeDrive(horiz, vert, throttle, true);
            }
        }, "MoveTurn", "MoveForwards", "Throttle");

        listenerRight.addButtonDownListener("Intake", () -> {
            hopper.runIntake();
            scheduler.schedule(cmdBallIntake);
            Log.info("Joystick","Button 3 pressed");
        });

        listenerRight.addButtonUpListener("Intake", () -> {
            hopper.stopIntake();
            cmdBallIntake.cancel();
            Log.info("Joystick","Button 3 unpressed");
        });

        listenerRight.addButtonDownListener("Shoot", () -> {
            //sidekick.setState(Sidekick.ShooterState.MID_RANGE);
            sidekick.shoot();
            shooter.shoot();
            scheduler.schedule(alignCmd);
            Log.info("Joystick","Button 4 pressed");
        });

        listenerRight.addButtonUpListener("Shoot", () -> {
            //sidekick.setState(Sidekick.ShooterState.OFF);
            sidekick.counterShoot();
            shooter.counterShoot();
            hopper.unshoot = true;
            alignCmd.cancel();
            //shooter.setSetpoint(0);
            driveCmdRunning.isRunning = true;
            shooter.isAligned = false;
            Log.info("Joystick","Button 4 unpressed");
        });

        listenerRight.addButtonDownListener("EmptyHopper", () -> {
            hopper.runHopperOpp();
            hopper.runIntakeOpp();
        });

        listenerRight.addButtonUpListener("EmptyHopper", () -> {
            hopper.stopHopper();
            hopper.stopIntake();
        });

        listenerRight.addButtonDownListener("MoveArmDown", () -> {
            hopper.moveArmDown();
        });

        listenerRight.addButtonUpListener("MoveArmDown", () -> {
            hopper.stopArm();
        });

        listenerRight.addButtonDownListener("MoveArmUp", () -> {
            hopper.moveArmUp();
        });

        listenerRight.addButtonUpListener("MoveArmUp", () -> {
            hopper.stopArm();
        });

        listenerRight.addButtonDownListener("ResetBallCount", () -> {
            hopper.resetBallCount();
        });

        listenerLeft.addButtonDownListener("REVERSE", () -> {
            reverse *= -1;
        });

        listenerRight.addButtonDownListener("SetOverYonder", () -> {
            shooter.setState(Shooter.ShooterState.LONG_RANGE);
        });
        listenerRight.addButtonDownListener("SetMiddling", () -> {
            shooter.setState(Shooter.ShooterState.MID_RANGE);
        });
        listenerRight.addButtonDownListener("SetIntimate", () -> {
            shooter.setState(Shooter.ShooterState.SHORT_RANGE);
        });

        listenerLeft.addButtonDownListener("SetGreen", () -> {
            shooter.setState(Shooter.ShooterState.GREEN);
        });
        listenerLeft.addButtonDownListener("SetYellow", () -> {
            shooter.setState(Shooter.ShooterState.YELLOW);
            //shooter.setSetpoint(Shooter.ShooterState.YELLOW.shooterRPM);
        });
        listenerLeft.addButtonDownListener("SetBlue", () -> {
            shooter.setState(Shooter.ShooterState.BLUE);
        });
        listenerLeft.addButtonDownListener("SetRed", () -> {
            shooter.setState(Shooter.ShooterState.RED);
        });

    }

    @Override
    protected void teleopPeriodic() {
    }

    double maxLeftSpeed = 0;
    double maxRightSpeed = 0;
    double maxSpeed = 0;
    double minLeftSpeed = 0;
    double minRightSpeed = 0;
    double minSpeed = 0;

    double currentLeftSpeed;
    double currentRightSpeed;
    double currentSpeed;
    double currentDistance;
    

    @Override
    protected void updateDashboard() {
        // SmartDashboard.putString("hopper update count", String.valueOf(//hopper.hopper_update_count));
        NarwhalDashboard.put("time", DriverStation.getInstance().getMatchTime());
        NarwhalDashboard.put("voltage", RobotController.getBatteryVoltage());

        currentLeftSpeed = drive.getLeftSpeed();
        currentRightSpeed = drive.getRightSpeed();

        currentSpeed = drive.getSpeed();
    

        SmartDashboard.putNumber("Left Velocity", currentLeftSpeed);
        SmartDashboard.putNumber("Right Velocity", currentRightSpeed);

    }

    @Override
    protected void teleopInit() {
        shooterLimelight.setLEDMode(LEDMode.OFF);
        Log.info("MainGrogu", "TeleopInit has started. Setting arm state to ArmState.STARTING");
        driveCmdRunning.isRunning = true;
    }

    @Override
    protected void autonomousInit() {
        drive.resetGyro();
        
        cmdBallIntake = new CmdBallIntake(drive, hopper, ahrs, ballLimelight, driveCmdRunning);

        //cmdBallPursuit = new CmdBallPursuit(ahrs, ballLimelight, driveCmdRunning,  0.472441 * Constants.MechanismConstants.inchesToMeters, Constants.VisionConstants.BALL_PID, 0, 2.5*Length.ft, 0.6666666666666666666666 * Length.ft, Constants.VisionConstants.BLIND_BALL_PID,42 * Angle.DEGREES);
        scheduler.schedule(cmdBallIntake);
    }

    @Override
    protected void disabledInit() {
        shooterLimelight.setLEDMode(LEDMode.OFF);
    }

    public static void main(String... args) {
        RobotBase.startRobot(MainGrogu::new);
    }

    @Override
    public void endCompetition() {
        // TODO Auto-generated method stub

    }
}