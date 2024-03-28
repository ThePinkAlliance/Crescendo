// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive.modules;

import javax.swing.text.html.HTMLDocument.RunElement;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.Gains;
import frc.robot.Constants;
import frc.robot.subsystems.drive.SwerveModule;
import org.littletonrobotics.junction.Logger;

/**
 * Swerve module based off of the swerve drive specialties pod. A Falcon 500 is
 * used for both steering and driving.
 */
public class WPI_SwerveModule implements SwerveModule {
    private PIDController steerController;

    private TalonFX driveMotor;
    private TalonFX steerMotor;
    private CANcoder canCoder;
    public static final double WARNINGTEMP = 55.0;
    private double absoluteEncoderOffsetRad;

    public WPI_SwerveModule(int steerId, int driveId, int canCoderId, boolean invertDrive, boolean invertSteer,
            double absoluteEncoderOffsetRad,
            Gains steerGains, String network) {
        this.canCoder = new CANcoder(canCoderId, network);
        this.steerMotor = new TalonFX(steerId, network);
        this.driveMotor = new TalonFX(driveId, network);

        TalonFXConfiguration driveConfig = new TalonFXConfiguration();
        CurrentLimitsConfigs driveCurrentLimit = new CurrentLimitsConfigs();
        OpenLoopRampsConfigs driveOpenloopConfig = new OpenLoopRampsConfigs();

        // This could be increased to 60 probably
        driveCurrentLimit.SupplyCurrentLimit = 45;
        driveCurrentLimit.SupplyCurrentLimitEnable = true;

        driveOpenloopConfig.DutyCycleOpenLoopRampPeriod = 0.5;

        driveConfig.OpenLoopRamps = driveOpenloopConfig;
        driveConfig.CurrentLimits = driveCurrentLimit;

        this.driveMotor.getConfigurator().apply(driveConfig);

        this.absoluteEncoderOffsetRad = absoluteEncoderOffsetRad;

        this.steerController = new PIDController(steerGains.kP, steerGains.kI, steerGains.kD);
        this.steerController.enableContinuousInput(-Math.PI, Math.PI);

        this.driveMotor.setInverted(invertDrive);
        this.steerMotor.setInverted(invertSteer);

        this.steerMotor.setNeutralMode(NeutralModeValue.Brake);
        this.driveMotor.setNeutralMode(NeutralModeValue.Brake);

        CANcoderConfiguration config = new CANcoderConfiguration();

        config.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;

        this.canCoder.getConfigurator().apply(config);

        resetEncoders();
    }

    /**
     * Returns the drive wheel position in meters.
     */
    @Override
    public double getDrivePosition() {
        double position = driveMotor.getRotorPosition().getValueAsDouble();

        position = position * 1.01845;

        return (position * Constants.ModuleConstants.kDriveMotorGearRatio)
                * (Constants.ModuleConstants.kWheelDiameterMeters * Math.PI);
    }

    /*
     * This returns the current position of the steer shaft in radians.
     */
    @Override
    public double getSteerPosition() {
        double absolute_position = canCoder.getAbsolutePosition().getValueAsDouble() * (Math.PI / .5);

        return absolute_position - absoluteEncoderOffsetRad;
    }

    @Override
    public double getRawAbsoluteAngularPosition() {
        double absolute_position = canCoder.getAbsolutePosition().getValueAsDouble() * (Math.PI / .5);

        return absolute_position;
    }

    @Override
    public double getDriveVelocity() {
        double selected_velocity = driveMotor.getVelocity().getValueAsDouble();

        return selected_velocity * Constants.ModuleConstants.kDriveMotorGearRatio;
    }

    @Override
    public double getSteerError() {
        return steerController.getPositionError();
    }

    /**
     * Returns the velocity of the steer motor in rad/sec.
     */
    @Override
    public double getSteerVelocity() {
        return canCoder.getVelocity().getValueAsDouble() * 2 * Math.PI;
    }

    @Override
    public void resetEncoders() {
        driveMotor.setPosition(0);
        steerMotor.setPosition(getAbsoluteEncoderAngle());
    }

    @Override
    public double getAbsoluteEncoderAngle() {
        double angle = canCoder.getAbsolutePosition().getValueAsDouble() * (Math.PI / .5);
        angle -= absoluteEncoderOffsetRad;

        return angle;
    }

    /*
     * Returns the current state of the swerve module using velocity.
     */
    @Override
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getSteerPosition()));
    }

    /*
     * Returns the current state of the swerve module using position.
     */
    @Override
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getSteerPosition()));
    }

    public double getMotorTemp() {
        return driveMotor.getDeviceTemp().getValueAsDouble();
    }

    public boolean isMotorOverheated() {
        boolean result = false;

        if (getMotorTemp() > WARNINGTEMP) {
            result = true;
        }

        return result;
    }

    /**
     * Sets the current module state to the desired one.
     * 
     * @param state desired swerve module state
     */
    @Override
    public void setDesiredState(SwerveModuleState state) {
        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor
                .setVoltage(
                        (state.speedMetersPerSecond / Constants.DriveConstants.kPhysicalMaxSpeedMetersPerSecond) * 12);

        double output = steerController.calculate(getSteerPosition(), state.angle.getRadians());
        Logger.recordOutput("Swerve/" + this.driveMotor.getDeviceID() + "/rpm",
                this.driveMotor.getRotorVelocity().getValueAsDouble());
        steerMotor.set(output);
    }

    @Override
    public void stop() {
        driveMotor.set(0);
        steerMotor.set(0);
    }
}
