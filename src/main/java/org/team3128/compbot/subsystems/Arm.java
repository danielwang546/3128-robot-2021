package org.team3128.compbot.subsystems;

import org.team3128.common.utility.Log;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.team3128.common.generics.Threaded;
import org.team3128.common.hardware.motor.LazyCANSparkMax;
import org.team3128.common.hardware.motor.LazyTalonFX;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

public class Arm extends PIDSubsystem{
    public static enum ArmState {
        STOWED(0), // arm is all the way down
        INTAKE(0), // intaking balls
        STARTING(49), // within frame perimeter
        STARTING_DOWN(40), // arm is pushed to release the intake
        LOADING_STATION(46),
        LONG_RANGE(52), // far range shooting
        MID_RANGE(48.3),
        SHORT_RANGE(5.5), // short range shooting
        CLIMBING(75), // climbing
        DEBUG(5);

        public double armAngle;

        private ArmState(double angle) {
            
            this.armAngle = angle;
        }
    }

    public static final Arm instance = new Arm();
    public LazyTalonFX ARM_MOTOR_LEADER, ARM_MOTOR_FOLLOWER;
    public DigitalInput LIMIT_SWITCH;
    public double setpoint;
    double current = 0;
    double error = 0;
    public double output = 0;
    double accumulator = 0;
    double prevError = 0;
    public ArmState ARM_STATE;
    boolean isZeroing = false;
    public int plateauCount = 0;

    public static Arm getInstance() {
        
        return instance;
    }

    private Arm() {

        super(new PIDController(Constants.ArmConstants.ARM_PID.kP, Constants.ArmConstants.ARM_PID.kI, Constants.ArmConstants.ARM_PID.kD)); 
        getController().setTolerance(Constants.ArmConstants.ANGLE_THRESHOLD);
        // ask about setDistance
        configMotors();
        configSensors();
        setState(ArmState.STARTING);
    }

    public void useOutput(double output, double setpoint) {
        ARM_MOTOR_LEADER.setVoltage(output + armFeedForward.calculate(setpoint)); 
    }

    private void configSensors() {
        LIMIT_SWITCH = new DigitalInput(Constants.ArmConstants.ARM_LIMIT_SWITCH_ID);
    }

    private void configMotors() {
        ARM_MOTOR_LEADER = new LazyTalonFX(Constants.ArmConstants.ARM_MOTOR_LEADER_ID);
        ARM_MOTOR_FOLLOWER = new LazyTalonFX(Constants.ArmConstants.ARM_MOTOR_FOLLOWER_ID);

        ARM_MOTOR_FOLLOWER.follow(ARM_MOTOR_LEADER);

        ARM_MOTOR_LEADER.setNeutralMode(Constants.ArmConstants.ARM_NEUTRAL_MODE);
        ARM_MOTOR_FOLLOWER.setNeutralMode(Constants.ArmConstants.ARM_NEUTRAL_MODE);

        ARM_MOTOR_LEADER.setSensorPhase(true);
        ARM_MOTOR_LEADER.setSelectedSensorPosition(Constants.ArmConstants.STARTING_POSITION);

    }

    public void setState(ArmState armState) {
        ARM_STATE = armState;
        Arm.setSetpoint(armState.armAngle);
    }

    public double armFeedForward(double desired) {
        return -0.3; // true value = -0.46
    }

    public double getMeasurement() {
        return (((getEncoderPos() / Constants.MechanismConstants.ENCODER_RESOLUTION_PER_ROTATION)
                / Constants.ArmConstants.ARM_GEARING) * 360) % 360; // TODO: account for
                                                                    // possible negative
    }

    public void zero() {
        setState(ArmState.STOWED);
    }

    public double getEncoderPos() {
        return ARM_MOTOR_LEADER.getSelectedSensorPosition(0);
    }

    private double getEncoderVel() {
        return ARM_MOTOR_LEADER.getSelectedSensorVelocity(0);
    }

    public boolean getLimitStatus() {
        return !LIMIT_SWITCH.get();
    }

    public boolean isReady() {
        return (plateauCount > Constants.ArmConstants.PLATEAU_THRESHOLD); 
    }

    @Override
    public void periodic() {         

    }

}