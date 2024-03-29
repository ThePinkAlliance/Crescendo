package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

    TalonFX m_topTalon;
    TalonFX m_bottomTalon;
    CANSparkMax m_motor;
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;
    public double velocity_ff;
    public double top_desired_vel, bottom_desired_vel;
    DigitalInput m_noteSwitch;

    public enum ShooterMove {
        LOAD,
        SHOOT
    }

    public Shooter() {
        this.m_topTalon = new TalonFX(42, "rio");
        this.m_bottomTalon = new TalonFX(43, "rio");
        this.m_motor = new CANSparkMax(44, CANSparkLowLevel.MotorType.kBrushless);

        this.m_noteSwitch = new DigitalInput(0);
        this.m_bottomTalon.setInverted(true);

        this.bottom_desired_vel = 0;
        this.top_desired_vel = 0;

        // set slot 0 gains
        var slot0Configs_1 = new Slot0Configs();
        slot0Configs_1.kS = 0.05; // Add 0.05 V output to overcome static friction
        slot0Configs_1.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs_1.kP = 0.15; // An error of 1 rps results in 0.11 V output
        slot0Configs_1.kI = 0; // An error of 1 rps increases output by 0.5 V each second
        slot0Configs_1.kD = 0.01; // An acceleration of 1 rps/s results in 0.01 V output

        var slot0Configs_2 = new Slot0Configs();
        slot0Configs_2.kS = 0.05; // Add 0.05 V output to overcome static friction
        slot0Configs_2.kV = 0.12; // A velocity target of 1 rps results in 0.12 V
        slot0Configs_2.kP = 0.11; // An error of 1 rps results in 0.11 V output
        slot0Configs_2.kI = 0; // An error of 1 rps increases output by 0.5 V each second
        slot0Configs_2.kD = 0.01; // An acceleration of 1 rps/s results in 0.01 V output

        m_topTalon.getConfigurator().apply(slot0Configs_1);
        m_bottomTalon.getConfigurator().apply(slot0Configs_2);

        setupLoaderMotor();
    }

    public void setupLoaderMotor() {
        m_motor.restoreFactoryDefaults();
        m_motor.setIdleMode(IdleMode.kCoast);
    }

    public void setSpeed(double speed) {
        this.m_topTalon.set(speed);
        this.m_bottomTalon.set(speed);
    }

    public void setVelocity(double vel) {
        this.setVelocity(vel, vel);
    }

    public void setVelocity(double top, double bottom) {
        double topRps = top / 60;
        double bottomRps = bottom / 60;

        // create a velocity closed-loop request, voltage output, slot 0 configs
        var request = new VelocityVoltage(0).withSlot(0);

        // set velocity rps, add 0.5 V to overcome gravity
        m_topTalon.setControl(request.withVelocity(topRps).withFeedForward(0.5));
        m_bottomTalon.setControl(request.withVelocity(bottomRps).withFeedForward(0.5));

        this.top_desired_vel = top;
        this.bottom_desired_vel = bottom;
    }

    public boolean isAtLeastRpm(double target) {
        boolean result = false;
        StatusSignal<Double> m_bottomVelocity = m_bottomTalon.getRotorVelocity();
        StatusSignal<Double> m_topVelocity = m_topTalon.getRotorVelocity();
        double m_bottomRPM = m_bottomVelocity.getValueAsDouble() * 60;
        double m_topRPM = m_topVelocity.getValueAsDouble() * 60;
        double minimumRpm = 100;
        double t = Math.abs(target);
        double rpm1 = Math.abs(m_bottomRPM);
        double rpm2 = Math.abs(m_topRPM);
        double error = 0.05;
        t = t - (t * error);

        Logger.recordOutput("Shooter/Top Desired Velocity", rpm2);
        Logger.recordOutput("Shooter/Bottom Desired Velocity", rpm1);

        Logger.recordOutput("Shooter/Top Velocity", this.m_topTalon.getVelocity().getValueAsDouble() * 60);
        Logger.recordOutput("Shooter/Bottom Velocity", this.m_bottomTalon.getVelocity().getValueAsDouble() * 60);

        System.out.println("Values: " + rpm1 + ":" + rpm2 + ":" + t);
        if (rpm1 >= t && rpm2 >= t && t > minimumRpm) {
            result = true;
        }
        return result;

    }

    public boolean noteFound() {
        return !m_noteSwitch.get();
    }

    // Do not call directly, use launch and load instead
    private void move(double rpms) {
        this.m_motor.set(rpms);
    }

    public void launch(double rpms) {
        move(-rpms);
    }

    public void load(double rpms) {
        move(rpms);
    }

    public void stop() {
        move(0);
    }

    public Command loadNoteUntilFound(double desiredVelocity) {
        return new FunctionalCommand(() -> {
        },
                () -> {
                    this.setSpeed(desiredVelocity);
                    this.load(1);
                },
                (interrupted) -> {
                    this.move(0);
                    this.setSpeed(0);
                },
                () -> noteFound(),
                this);
    }

    public Command loadNoteUntilFound2(double desiredVelocity) {

        return new FunctionalCommand(() -> {
        },
                () -> {
                    this.setVelocity(desiredVelocity);
                    this.load(1);
                },
                (interrupted) -> {
                    this.move(0);
                    this.setSpeed(0);
                },
                () -> noteFound(),
                this);

    }

    public Command rampUp2(double desiredVelocity) {

        return new FunctionalCommand(() -> {
        },
                () -> {
                    this.setVelocity(desiredVelocity);
                },
                (interrupted) -> {
                },
                () -> isAtLeastRpm(desiredVelocity),
                this);
    }

    public Command launchNote2() {
        Timer time = new Timer();
        return new FunctionalCommand(() -> {
            time.reset();
            time.start();
        },
                () -> {
                    this.launch(1);
                },
                (interrupted) -> {
                    this.setSpeed(0);
                    this.stop();
                },
                () -> {
                    return time.hasElapsed(1.5);
                },
                this);
    }

    public StatusSignal<Double> getBottomVelocity() {
        return this.m_bottomTalon.getVelocity();
    }

    public StatusSignal<Double> getTopVelocity() {
        return this.m_topTalon.getVelocity();
    }

    public double getBottomDesiredVelocity() {
        return this.bottom_desired_vel;
    }

    public double getTopDesiredVelocity() {
        return this.top_desired_vel;
    }

    public Command stopShooter() {
        return runOnce(() -> {
            move(0);
            setSpeed(0);
        });
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Note Loaded", noteFound());
    }
}
