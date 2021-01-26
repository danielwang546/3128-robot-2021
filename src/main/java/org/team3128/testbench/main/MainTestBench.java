package org.team3128.testbench.main;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import org.team3128.common.generics.RobotConstants;

import org.team3128.common.NarwhalRobot;
import org.team3128.common.control.trajectory.Trajectory;
import org.team3128.common.control.trajectory.TrajectoryGenerator;
import org.team3128.common.control.trajectory.constraint.TrajectoryConstraint;
import org.team3128.common.drive.DriveCommandRunning;
import org.team3128.common.hardware.limelight.Limelight;
import org.team3128.common.hardware.gyroscope.Gyro;
import org.team3128.common.hardware.gyroscope.NavX;
import org.team3128.common.utility.units.Angle;
import org.team3128.common.utility.units.Length;
import org.team3128.common.vision.CmdHorizontalOffsetFeedbackDrive;
import org.team3128.testbench.subsystems.Constants;
import org.team3128.common.utility.Log;
import org.team3128.common.utility.RobotMath;
import org.team3128.common.utility.datatypes.PIDConstants;
import org.team3128.common.narwhaldashboard.NarwhalDashboard;
import org.team3128.common.listener.ListenerManager;
import org.team3128.common.listener.controllers.ControllerExtreme3D;
import org.team3128.common.listener.controltypes.Button;
import org.team3128.common.hardware.motor.LazyCANSparkMax;
import org.team3128.common.hardware.motor.LazyTalonFX;
import org.team3128.common.utility.math.Pose2D;
import org.team3128.common.utility.math.Rotation2D;
import org.team3128.testbench.subsystems.Shooter;
import org.team3128.common.drive.DriveSignal;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.*;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.wpilibj2.command.CommandScheduler;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import java.util.ArrayList;
import java.util.concurrent.*;


public class MainTestBench extends NarwhalRobot {
    Shooter shooter = Shooter.getInstance();

    ExecutorService executor = Executors.newFixedThreadPool(4);
    CommandScheduler scheduler = CommandScheduler.getInstance();
    Thread auto;

    boolean joystickInput, throttleInput, zeroRPM;

    double shooterRPM;

    public DigitalInput digitalInput;
    public DigitalInput digitalInput2;

    public Joystick joystick;
    public ListenerManager lm;

    public NetworkTable table;
    public NetworkTable limelightTable;

    //public LazyTalonFX testMotor;
    //public LazyTalonFX testMotor2;

    public double startTime = 0;

    public String trackerCSV = "Time, X, Y, Theta, Xdes, Ydes";

    public boolean inPlace = false;
    public boolean inPlace2 = false;

    public int countBalls = 0;
    public int countBalls2 = 0;

    @Override
    protected void constructHardware() {

        //digitalInput = new DigitalInput(0);
        //digitalInput2 = new DigitalInput(1);

        // testMotor = new LazyTalonFX(0);
        // testMotor2 = new LazyTalonFX(1);
        // testMotor2.follow(testMotor);
        // testMotor2.setInverted(true);

        joystick = new Joystick(1);
        lm = new ListenerManager(joystick);
        addListenerManager(lm);

        joystickInput = false;
        throttleInput = false;
        zeroRPM = true;
        shooter.enable();
        shooter.setSetpoint(0);
        shooterRPM = Constants.SHOOTER_TESTING_RPM;
    }

    @Override
    protected void constructAutoPrograms() {
    }

    @Override
    protected void setupListeners() {
        lm.nameControl(ControllerExtreme3D.TWIST, "MoveTurn");
        lm.nameControl(ControllerExtreme3D.JOYY, "MoveForwards");
        lm.nameControl(ControllerExtreme3D.THROTTLE, "Throttle");
        lm.nameControl(ControllerExtreme3D.TRIGGER, "Trigger");
        lm.nameControl(new Button(3), "zeroShooter");
        lm.nameControl(new Button(4), "startShooter");
        lm.nameControl(new Button(10), "increaseRPM");
        lm.nameControl(new Button(9), "decreaseRPM");
        lm.nameControl(new Button(11), "bigDecreaseRPM");
        lm.nameControl(new Button(12), "bigIncreaseRPM");
        lm.nameControl(new Button(7), "toggleThrottleInput");

        // lm.addButtonDownListener("forwardMotor", () -> {
        //     testMotor.set(ControlMode.PercentOutput, 0.5);
        // });
        // lm.addButtonUpListener("forwardMotor", () -> {
        //     testMotor.set(ControlMode.PercentOutput, 0);
        // });

        // lm.addButtonDownListener("reverseMotor", () -> {
        //     testMotor.set(ControlMode.PercentOutput, -0.5);
        // });
        // lm.addButtonUpListener("reverseMotor", () -> {
        //     testMotor.set(ControlMode.PercentOutput, 0);
        // });

        // lm.addMultiListener(() -> {
        //     // drive.arcadeDrive(-0.7 * RobotMath.thresh(lm.getAxis("MoveTurn"), 0.1),
        //     // -1.0 * RobotMath.thresh(lm.getAxis("MoveForwards"), 0.1), -1.0 *
        //     // lm.getAxis("Throttle"), true);

        lm.addMultiListener(() -> {
            if (throttleInput) {
                double throttle = (lm.getAxis("Throttle") + 1) / 2;
                throttle*=Constants.FALCON_MAX_RPM;
                shooter.setSetpoint(throttle);
                Log.info("MainTestBench", "Shooter RPM set to " + throttle + " RPM");
            }
        }, "Throttle");

        lm.addButtonDownListener("startShooter", () -> {
            zeroRPM = false;
            if (!throttleInput)
                shooter.setSetpoint(shooterRPM);
                Log.info("TestBenchMain", "starting shooter");
        });
        
        lm.addButtonUpListener("zeroShooter", () -> {
            zeroRPM = true;
            shooter.setSetpoint(0);
            Log.info("TestBenchMain", "Zeroing shooter");
        });

        // lm.addMultiListener(() -> {
        //     if (joystickInput)
        //         testMotor.set(ControlMode.PercentOutput, -1 * RobotMath.clampPosNeg1(lm.getAxis("MoveForwards")));
        // }, "MoveForwards");

        lm.addButtonDownListener("Trigger", () -> {
            joystickInput = true;
        });

        lm.addButtonUpListener("Trigger", () -> {
            joystickInput = false;
        });

        lm.addButtonDownListener("increaseRPM", () -> {
            if (!throttleInput) {
                shooterRPM+=100;
                // shooter.setSetpoint(shooterRPM);
                Log.info("MainTestBench", "Shooter RPM set to " + shooterRPM + " RPM");
            }
        });

        lm.addButtonUpListener("decreaseRPM", () -> {
            if (!throttleInput) {
                shooterRPM-=100;
                // shooter.setSetpoint(shooterRPM);
                Log.info("MainTestBench", "Shooter RPM set to " + shooterRPM + " RPM");
            }
        });

        lm.addButtonDownListener("bigIncreaseRPM", () -> {
            if (!throttleInput) {
                shooterRPM+=500;
                // shooter.setSetpoint(shooterRPM);
                Log.info("MainTestBench", "Shooter RPM set to " + shooterRPM + " RPM");
            }
        });

        lm.addButtonUpListener("bigDecreaseRPM", () -> {
            if (!throttleInput) {
                shooterRPM-=500;
                // shooter.setSetpoint(shooterRPM);
                Log.info("MainTestBench", "Shooter RPM set to " + shooterRPM + " RPM");
            }
        });

        lm.addButtonDownListener("toggleThrottleInput", () -> {
            throttleInput = !throttleInput;
        });

        // }, "MoveTurn", "MoveForwards", "Throttle");

        // lm.nameControl(ControllerExtreme3D.TRIGGER, "AlignToTarget");
        // lm.addButtonDownListener("AlignToTarget", () -> {
        //     // TODO: Add current implementation of vision alignment
        //     Log.info("MainAthos.java", "[Vision Alignment] Not created yet, would've started");
        // });
        // lm.addButtonUpListener("AlignToTarget", () -> {
        //     Log.info("MainAthos.java", "[Vision Alignment] Not created yet, would've ended");
        // });

        // lm.addButtonDownListener("setSetpoint0", () -> {
        //     //shooter.setSetpoint(0);
        //     Log.info("Button4", "pressed");
        // });

        // lm.addButtonDownListener("setSetpoint1", () -> {
        //     Log.info("Button5", "pressed");
        //     Log.info("Shooter", "Start Voltage: " + String.valueOf(RobotController.getBatteryVoltage()));
        //     //shooter.setSetpoint(250);
        // });



        // lm.addButtonDownListener("EndVoltage", () -> {
        //     Log.info("Shooter", String.valueOf(RobotController.getBatteryVoltage())); 

        // });

    }

    @Override
    protected void teleopPeriodic() {
        // if (inPlace == false && digitalInput.get()) {
        //     countBalls++;
        //     System.out.println("Number of balls: " + countBalls);
        //     inPlace = true;
        // } else if (!digitalInput.get()) {
        //     inPlace = false;
        // }

        // if (inPlace2 == false && digitalInput2.get()) {
        //     countBalls--;
        //     System.out.println("Number of balls: " + countBalls);
        //     inPlace2 = true;
        // } else if (!digitalInput2.get()) {
        //     inPlace2 = false;
        // }
        //double outRPM =  testMotor.getSelectedSensorVelocity() * 10 * 60 / Constants.MechanismConstants.ENCODER_RESOLUTION_PER_ROTATION;
        double outRPM = shooter.getMeasurement();
        if (outRPM != 0 && !zeroRPM)
            Log.info("MainTestBench", "Test motor speed: " + outRPM + " RPM");
    }

    @Override
    protected void updateDashboard() {

    }

    @Override
    protected void teleopInit() {
        //scheduler.resume();
        shooter.enable();
    }

    @Override
    protected void autonomousInit() {

    }

    @Override
    protected void autonomousPeriodic() {
    }

    @Override
    protected void disabledInit() {
        //shooter.setSetpoint(0);
        //scheduler.pause();
    }

    public static void main(String... args) {
        RobotBase.startRobot(MainTestBench::new);
    }

    @Override
    public void endCompetition() {
        // TODO Auto-generated method stub

    }
}
