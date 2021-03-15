package org.team3128.grogu.main;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import org.team3128.common.generics.RobotConstants;

import com.kauailabs.navx.frc.AHRS;


import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;


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

    // public StateTracker stateTracker = StateTracker.getInstance();
    static FalconDrive drive = FalconDrive.getInstance();

 
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

    public String trackerCSV = "Time, X, Y, Theta, Xdes, Ydes";

    public ArrayList<Pose2D> waypoints = new ArrayList<Pose2D>();
    public Trajectory trajectory;

    public Limelight shooterLimelight, ballLimelight;
    public Limelight[] limelights;
    public boolean inPlace = false;
    public boolean inPlace2 = false;

    public Hopper hopper = Hopper.getInstance();

    private boolean teleopKinematics = true;


    static EKF ekf = new EKF(0, 0, Math.PI/2, 0, 0, 10, 10, 0.66,//0.9652,
    0.01, 1e-3, 0.01, 0.01);

    ArrayList<Double> KxList = new ArrayList<Double>();
    ArrayList<Double> KyList = new ArrayList<Double>();
    ArrayList<Double> KthetaList = new ArrayList<Double>();
    ArrayList<Double> KvlList = new ArrayList<Double>();
    ArrayList<Double> KvrList = new ArrayList<Double>();

    private double[] inputArray = new double[4];
    private double[] kinematicArray = new double[6];
    private double[] outputArray;
    private double currentTime, previousTime, printerTime, initTime;

    public static Pose2d ekfPosition;


    @Override
    protected void constructHardware() {

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

        shooterLimelight = new Limelight("limelight-shooter", 26.0, 0, 0, 30);
        ballLimelight = new Limelight("limelight-c", Constants.VisionConstants.BOTTOM_LIMELIGHT_ANGLE,
                Constants.VisionConstants.BOTTOM_LIMELIGHT_HEIGHT,
                Constants.VisionConstants.BOTTOM_LIMELIGHT_DISTANCE_FROM_FRONT, 14.5 * Length.in);
        limelights = new Limelight[2];
        drive.resetGyro();

        hopper.register();
    }

    @Override
    protected void constructAutoPrograms() {
    }

    @Override
    protected void setupListeners() {
        listenerRight.nameControl(ControllerExtreme3D.TWIST, "MoveTurn");
        listenerRight.nameControl(ControllerExtreme3D.JOYY, "MoveForwards");
        listenerRight.nameControl(ControllerExtreme3D.THROTTLE, "Throttle");
        listenerRight.nameControl(new Button(3), "Intake");
        listenerRight.nameControl(new Button(7), "MoveArmDown");
        listenerRight.nameControl(new Button(8), "MoveArmUp");

        listenerRight.addMultiListener(() -> {
            if (driveCmdRunning.isRunning) {
                double horiz = 0.5 * listenerRight.getAxis("MoveTurn"); //0.7
                double vert = -1.0 * listenerRight.getAxis("MoveForwards");
                double throttle = -1.0 * listenerRight.getAxis("Throttle");

                drive.arcadeDrive(horiz, vert, throttle, true);
            }
        }, "MoveTurn", "MoveForwards", "Throttle");

        listenerRight.addButtonDownListener("Intake", () -> {
            hopper.runIntake();
        });

        listenerRight.addButtonUpListener("Intake", () -> {
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

    }

    @Override
    protected void teleopPeriodic() {
        if (teleopKinematics){
            Log.info("gyro", ""+drive.getAngle());
            currentTime=RobotController.getFPGATime()/1000000.0;
            //currentTime = currentTime*1e-06;
            //I'm not sure how to check if new readings are available so right now we are running predict and update every time
            inputArray[0] = drive.getAngle() * Math.PI / 180.0;
            inputArray[1] = drive.getLeftSpeed() * 0.0254;
            inputArray[2] = drive.getRightSpeed() * 0.0254;
            inputArray[3] = currentTime-previousTime;
        // where EKF is run
            outputArray = ekf.runFilter(inputArray);

            
            kinematicArray[0] = KxList.get(KxList.size()-1);
            kinematicArray[1] = KyList.get(KyList.size()-1);
            kinematicArray[2] = drive.getAngle() * Math.PI / 180.0;
            kinematicArray[3] = drive.getLeftSpeed() * 0.0254;
            kinematicArray[4] = drive.getRightSpeed() * 0.0254;
            kinematicArray[5] = currentTime-previousTime;

            outputArray = ekf.testFunction(kinematicArray);

            KxList.add(outputArray[0]);
            KyList.add(outputArray[1]);
            KthetaList.add(outputArray[2]);
            KvlList.add(outputArray[3]);
            KvrList.add(outputArray[4]);
        
            //Log.info("EKF", "X: " + outputArray[0] + " Y: " + outputArray[1] + " THETA" + outputArray[2]);
            ekfPosition=new Pose2d(outputArray[0], outputArray[1], new Rotation2d(outputArray[2]));
            Log.info("EKF", ekfPosition.toString());




            previousTime=currentTime;
            //Log.info("MainAthos", ((Double) robotTracker.getOdometry().getTranslation().getX()).toString());

        }

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
        Log.info("MainCompbot", "TeleopInit has started. Setting arm state to ArmState.STARTING");
        driveCmdRunning.isRunning = true;

        if (teleopKinematics){
            drive.resetGyro();
            //scheduler.resume();

            KxList.add((double) 0);
            KyList.add((double) 0);
            KthetaList.add(Math.PI/2);
            KvlList.add((double) 0);
            KvrList.add((double) 0);

            initTime=RobotController.getFPGATime()/1000000.0;
        }



    }

    @Override
    protected void autonomousInit() {
        drive.resetGyro();
        trackerCSV = "Time, X, Y, Theta, Xdes, Ydes";
        Log.info("MainAthos", "going into autonomousinit");
        //scheduler.resume();
        //drive.setAutoTrajectory(trajectory, false);
        //drive.startTrajectory();
        //scheduler.resume();

        KxList.add((double) 0);
        KyList.add((double) 0);
        KthetaList.add(Math.PI/2);
        KvlList.add((double) 0);
        KvrList.add((double) 0);

        
        PathFinding pathfinder = new PathFinding();
        new PathRunner(pathfinder, drive).schedule();
        Log.info("MainAthos","1");
        
        startTime = Timer.getFPGATimestamp();

        initTime=RobotController.getFPGATime()/1000000.0;

    }


    @Override
    protected void autonomousPeriodic() {
        currentTime=RobotController.getFPGATime()/1000000.0;
        //currentTime = currentTime*1e-06;
        //I'm not sure how to check if new readings are available so right now we are running predict and update every time
        inputArray[0] = drive.getAngle() * Math.PI / 180.0;
        inputArray[1] = drive.getLeftSpeed() * 0.0254;
        inputArray[2] = drive.getRightSpeed() * 0.0254;
        inputArray[3] = currentTime-previousTime;
       // where EKF is run

        kinematicArray[0] = KxList.get(KxList.size()-1);
        kinematicArray[1] = KyList.get(KyList.size()-1);
        kinematicArray[2] = drive.getAngle() * Math.PI / 180.0;
        kinematicArray[3] = drive.getLeftSpeed() * 0.0254;
        kinematicArray[4] = drive.getRightSpeed() * 0.0254;
        kinematicArray[5] = currentTime-previousTime;

        outputArray = ekf.testFunction(kinematicArray);

        KxList.add(outputArray[0]);
        KyList.add(outputArray[1]);
        KthetaList.add(outputArray[2]);
        KvlList.add(outputArray[3]);
        KvrList.add(outputArray[4]);
       
        //Log.info("EKF", "X: " + outputArray[0] + " Y: " + outputArray[1] + " THETA" + outputArray[2]);
        ekfPosition=new Pose2d(outputArray[0], outputArray[1], new Rotation2d(outputArray[2]));
        Log.info("EKF", ekfPosition.toString());




        previousTime=currentTime;
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