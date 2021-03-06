package org.team3128.grogu.subsystems;

import org.team3128.common.hardware.motor.LazyCANSparkMax;
import org.team3128.common.control.RateLimiter;
import org.team3128.common.control.AsynchronousPid;
import org.team3128.common.control.motion.RamseteController;
import org.team3128.common.control.trajectory.Trajectory;
import org.team3128.common.control.trajectory.Trajectory.State;
import org.team3128.common.drive.AutoDriveSignal;
import org.team3128.common.drive.DriveSignal;
import org.team3128.common.utility.math.Rotation2D;
import org.team3128.common.utility.NarwhalUtility;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Subsystem;

import org.team3128.common.hardware.motor.LazyTalonFX;
import org.team3128.common.hardware.motor.LazyTalonSRX;
import org.team3128.common.hardware.motor.LazyVictorSPX;
import org.team3128.common.utility.RobotMath;

import edu.wpi.first.wpilibj.Timer;

import com.kauailabs.navx.frc.AHRS;

import org.team3128.common.utility.Log;
import org.team3128.common.drive.Drive;

public class Hopper implements Subsystem {

    private enum HopperState {
        IDLE,INTAKING,SHOOTING;
    }

    public static final Hopper instance = new Hopper();
    private LazyTalonSRX ARM_MOTOR, HOPPER_MOTOR;
    private LazyVictorSPX BRUSH_MOTOR, INTAKE_MOTOR;

    private DigitalInput BOTTOM_SENSOR, TOP_SENSOR;

    private HopperState actionState = HopperState.IDLE;
    
    private int ballCount = 0;

    private boolean wasTriggeredTop = false;
    private boolean wasTriggeredBottom = false;

    //TODO: set this boolean to boolean in shooter class 
    private boolean isShooterReady = false;

    public static Hopper getInstance() {
        
        return instance;
    }

    public Hopper() {
        configMotors();
        configSensors();
    }

    private void configMotors() {
        ARM_MOTOR = new LazyTalonSRX(Constants.IntakeConstants.ARM_MOTOR_ID);
        BRUSH_MOTOR = new LazyVictorSPX(Constants.IntakeConstants.BRUSH_MOTOR_ID);
        INTAKE_MOTOR = new LazyVictorSPX(Constants.IntakeConstants.INTAKE_MOTOR_ID);
        HOPPER_MOTOR = new LazyTalonSRX(Constants.HopperConstants.HOPPER_MOTOR_ID);

        ARM_MOTOR.setNeutralMode(Constants.IntakeConstants.ARM_NEUTRAL_MODE);

    }
    
    private void configSensors() {
       BOTTOM_SENSOR = new DigitalInput(Constants.HopperConstants.BOTTOM_SENSOR_ID); 
       TOP_SENSOR = new DigitalInput(Constants.HopperConstants.TOP_SENSOR_ID);
    }

    @Override
    public void periodic() {
        switch(actionState) {
            case IDLE:
                stopIntake();
                stopArm();
                stopHopper();
                if (BOTTOM_SENSOR.get()) {
                    setState(HopperState.INTAKING);
                    Log.info("Hopper","Ball is close enough to eat");
                } else if (isShooterReady) {
                    setState(HopperState.SHOOTING);
                    Log.info("Hopper","I don't feel so good");
                }   
                break;
            case INTAKING:
                intake();
                break;
            case SHOOTING:
                shoot();
                break;
        }
        wasTriggeredBottom = BOTTOM_SENSOR.get();
        wasTriggeredTop = TOP_SENSOR.get();
        //TODO: update this variable
        isShooterReady = (Sidekick.getInstance().atSetpoint() && Sidekick.getInstance().atSetpoint());
    }

    private void intake() {
       if (ballCount >= 3 || TOP_SENSOR.get()) {
            if (ballCount>3)
                Log.info("Hopper","oopsie, should not be greater than 3");
            Log.info("Hopper Stomach","FULL!!!!");
            setState(HopperState.IDLE);
       } else {
            runHopper();
            if (!BOTTOM_SENSOR.get() && wasTriggeredBottom) {
                setState(HopperState.IDLE);
                ballCount++;
                Log.info("Hopper Stomach", "Has eaten 1 ball");
            }
       }

    }

    private void shoot() {
        runHopper();
        if (wasTriggeredTop && TOP_SENSOR.get()) {
            setState(HopperState.IDLE);
            Log.info("Hopper Stomach","Discarded one ball at a high velocity");
        }
    }

    public void setState(HopperState state) {
        actionState = state;
    }

    public HopperState getState() {
        return actionState;
    }

    public void runHopper() {
        HOPPER_MOTOR.set(ControlMode.PercentOutput, Constants.HopperConstants.HOPPER_MOTOR_POWER);
    }

    public void stopHopper() {
        HOPPER_MOTOR.set(ControlMode.PercentOutput, 0);
    }

    public void runIntake() {
        INTAKE_MOTOR.set(ControlMode.PercentOutput, Constants.IntakeConstants.INTAKE_MOTOR_POWER);
        BRUSH_MOTOR.set(ControlMode.PercentOutput, Constants.IntakeConstants.BRUSH_MOTOR_POWER);
    }

    public void stopIntake() {
        INTAKE_MOTOR.set(ControlMode.PercentOutput, 0);
        BRUSH_MOTOR.set(ControlMode.PercentOutput, 0);
    }

    public void moveArmDown() {
        ARM_MOTOR.set(ControlMode.PercentOutput, -Constants.IntakeConstants.ARM_MOTOR_POWER);
    }

    public void moveArmUp() {
        ARM_MOTOR.set(ControlMode.PercentOutput, Constants.IntakeConstants.ARM_MOTOR_POWER);
    }

    public void stopArm() {
        ARM_MOTOR.set(ControlMode.PercentOutput, 0);
    }
}
