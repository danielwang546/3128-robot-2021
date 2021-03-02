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

public class Intake implements Subsystem {

    public static final Intake instance = new Intake();
    public LazyTalonSRX SERIALIZER_ARM_MOTOR;
    public LazyVictorSPX BRUSH_MOTOR, INTAKE_MOTOR;

    public static Intake getInstance() {
        
        return instance;
    }

    public Intake() {
        configMotors();
    }

    private void configMotors() {
        SERIALIZER_ARM_MOTOR = new LazyTalonSRX(Constants.IntakeConstants.SERIALIZER_ARM_MOTOR_ID);
        BRUSH_MOTOR = new LazyVictorSPX(Constants.IntakeConstants.BRUSH_MOTOR_ID);
        INTAKE_MOTOR = new LazyVictorSPX(Constants.IntakeConstants.INTAKE_MOTOR_ID);

        SERIALIZER_ARM_MOTOR.setNeutralMode(Constants.IntakeConstants.ARM_NEUTRAL_MODE);

    }

    public void intake() {
        INTAKE_MOTOR.set(ControlMode.PercentOutput, -.8);
        BRUSH_MOTOR.set(ControlMode.PercentOutput, 0.4);
    }

    public void stopIntake() {
        INTAKE_MOTOR.set(ControlMode.PercentOutput, 0);
        BRUSH_MOTOR.set(ControlMode.PercentOutput, 0);
    }

    public void moveArmDown() {
        SERIALIZER_ARM_MOTOR.set(ControlMode.PercentOutput, -.2);
    }

    public void moveArmUp() {
        SERIALIZER_ARM_MOTOR.set(ControlMode.PercentOutput, .2);
    }

    public void stopArm() {
        SERIALIZER_ARM_MOTOR.set(ControlMode.PercentOutput, 0);
    }
}
