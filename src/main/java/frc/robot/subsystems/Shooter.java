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

    TalonFX m_greenTalon;
    TalonFX m_greyTalon;
    CANSparkMax m_motor;
    SparkPIDController m_pidController;
    private RelativeEncoder m_encoder;
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;
    public double velocity_ff;
    public double top_desired_vel, bottom_desired_vel;
    DigitalInput m_noteSwitch;

    public enum ShooterMove {
        LOAD,
        SHOOT
    }

    public Shooter() {
        this.m_greenTalon = new TalonFX(42, "rio");
        this.m_greyTalon = new TalonFX(43, "rio");
        this.m_motor = new CANSparkMax(44, CANSparkLowLevel.MotorType.kBrushless);

        this.m_noteSwitch = new DigitalInput(0);
        this.m_greyTalon.setInverted(true);

        this.bottom_desired_vel = 0;
        this.top_desired_vel = 0;

        // this.m_greenTalon.ramp

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

        m_greenTalon.getConfigurator().apply(slot0Configs_1);
        m_greyTalon.getConfigurator().apply(slot0Configs_2);

        SmartDashboard.putNumber("RpmsShooter", 0.0);

        setupLoaderMotor();
    }

    public void setupLoaderMotor() {
        m_motor.restoreFactoryDefaults();
        m_motor.setIdleMode(IdleMode.kCoast);
        m_pidController = m_motor.getPIDController();
        m_encoder = m_motor.getEncoder();
        kP = 6e-5;
        kI = 0;
        kD = 0;
        kIz = 0;
        kFF = 0.000015;
        kMaxOutput = 1;
        kMinOutput = -1;
        maxRPM = 5700;
        // set PID coefficients
        m_pidController.setP(kP);
        m_pidController.setI(kI);
        m_pidController.setD(kD);
        m_pidController.setIZone(kIz);
        m_pidController.setFF(kFF);
        m_pidController.setOutputRange(kMinOutput, kMaxOutput);
    }

    public void setupLoaderDashboardInputs() {

        // display PID coefficients on SmartDashboard
        SmartDashboard.putNumber("Spark P Gain", kP);
        SmartDashboard.putNumber("Spark I Gain", kI);
        SmartDashboard.putNumber("Spark D Gain", kD);
        SmartDashboard.putNumber("Spark I Zone", kIz);
        SmartDashboard.putNumber("Spark Feed Forward", kFF);
        SmartDashboard.putNumber("Spark Max Output", kMaxOutput);
        SmartDashboard.putNumber("Spark Min Output", kMinOutput);
    }

    public void setupDashboardInputs() {
        // add values for PID config
        SmartDashboard.putNumber("P", 0.11); // 0.11
        SmartDashboard.putNumber("I", 0.5); // 0.5
        SmartDashboard.putNumber("D", 0.01); // 0.01

        SmartDashboard.putNumber("grey Target", 0);
        SmartDashboard.putNumber("green Target", 0);

        SmartDashboard.putBoolean("stop", false);

        SmartDashboard.putBoolean("Green isInverted", true);
        SmartDashboard.putBoolean("Grey isInverted", false);

        SmartDashboard.putBoolean("individual", false);
        SmartDashboard.putNumber("RpmsShooter", 0.0);

        setupLoaderDashboardInputs();
    }

    public void setInvertedMotors() {
        boolean greenInvert = SmartDashboard.getBoolean("Green isInverted", false);
        boolean greyInvert = SmartDashboard.getBoolean("Grey isInverted", false);

        m_greenTalon.setInverted(greenInvert);
        m_greyTalon.setInverted(greyInvert);
    }

    public void setMotionMagicRps(double grey, double green, double both) {
        boolean controlIndividual = SmartDashboard.getBoolean("individual", false);

        double greenToRpm = green / 60;
        double greyToRpm = grey / 60;
        double bothToRpm = both / 60;

        // create a velocity closed-loop request, voltage output, slot 0 configs
        var request = new VelocityVoltage(0).withSlot(0);

        if (!controlIndividual) {
            m_greenTalon.setControl(request.withVelocity(bothToRpm).withFeedForward(velocity_ff));
            m_greyTalon.setControl(request.withVelocity(bothToRpm).withFeedForward(velocity_ff));
        } else if (controlIndividual) {
            // set velocity rps, add velocity_ff V to overcome gravity
            m_greenTalon.setControl(request.withVelocity(greenToRpm).withFeedForward(velocity_ff));
            m_greyTalon.setControl(request.withVelocity(greyToRpm).withFeedForward(velocity_ff));
        } else {
            DriverStation.reportWarning("!cannot determine motor control! (shooter.java, 95)", true);
        }
    }

    public void setSpeed(double speed) {
        this.m_greenTalon.set(speed);
        this.m_greyTalon.set(speed);
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
        m_greenTalon.setControl(request.withVelocity(topRps).withFeedForward(0.5));
        m_greyTalon.setControl(request.withVelocity(bottomRps).withFeedForward(0.5));

        this.top_desired_vel = top;
        this.bottom_desired_vel = bottom;
    }

    @Deprecated
    public void adjustPID() {
        double proportional = SmartDashboard.getNumber("P", 0);
        double intergral = SmartDashboard.getNumber("I", 0);
        double derivitive = SmartDashboard.getNumber("D", 0);

        var pidConfigs = new Slot0Configs();
        pidConfigs.kS = 0.5; // 0.5 // Add 0.05 V output to overcome static friction
        pidConfigs.kV = 0.12; // 0.12 // A velocity target of 1 rps results in 0.12 V output
        pidConfigs.kP = proportional;
        pidConfigs.kI = intergral;
        pidConfigs.kD = derivitive;

        m_greenTalon.getConfigurator().apply(pidConfigs);
        m_greyTalon.getConfigurator().apply(pidConfigs);

    }

    @Deprecated
    public void putValues() {

        StatusSignal<Double> m_greyFXTickVelocity = m_greyTalon.getRotorVelocity();
        StatusSignal<Double> m_greenFXTickVelocity = m_greenTalon.getRotorVelocity();

        double m_greyFXToRPM = m_greyFXTickVelocity.getValueAsDouble() * 60;
        double m_greenFXToRPM = m_greenFXTickVelocity.getValueAsDouble() * 60;

        // add values for actual RPM of motors
        SmartDashboard.putNumber("grey real", m_greyFXToRPM);
        SmartDashboard.putNumber("green real", m_greenFXToRPM);

        double rpmAverage = (m_greenFXToRPM + m_greyFXToRPM) / 2;
        SmartDashboard.putNumber("average", rpmAverage);

        // add values for graphing
        SmartDashboard.putNumber("grey real graph", m_greyFXToRPM);
        SmartDashboard.putNumber("green real graph", m_greenFXToRPM);

    }

    public boolean isAtLeastRpm(double target) {
        boolean result = false;
        StatusSignal<Double> m_greyFXTickVelocity = m_greyTalon.getRotorVelocity();
        StatusSignal<Double> m_greenFXTickVelocity = m_greenTalon.getRotorVelocity();
        double m_greyFXToRPM = m_greyFXTickVelocity.getValueAsDouble() * 60;
        double m_greenFXToRPM = m_greenFXTickVelocity.getValueAsDouble() * 60;
        double minimumRpm = 100;
        double t = Math.abs(target);
        double rpm1 = Math.abs(m_greyFXToRPM);
        double rpm2 = Math.abs(m_greenFXToRPM);
        double error = 0.05;
        t = t - (t * error);

        Logger.recordOutput("RPM2", rpm2);
        Logger.recordOutput("RPM1", rpm1);

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

    public Command rampUp(double speed) {
        return runOnce(() -> this.setVelocity(speed));
    }

    public Command launchNote() {
        return runOnce(() -> this.launch(1));
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

    public Command launchNote3() {
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
                    return time.hasElapsed(1);
                },
                this);
    }

    public StatusSignal<Double> getBottomVelocity() {
        return this.m_greyTalon.getVelocity();
    }

    public StatusSignal<Double> getTopVelocity() {
        return this.m_greenTalon.getVelocity();
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
        SmartDashboard.putBoolean("Note Loaded", m_noteSwitch.get());

        Logger.recordOutput("Shooter/Top Velocity", this.m_greenTalon.getVelocity().getValueAsDouble() * 60);
        Logger.recordOutput("Shooter/Bottom Velocity", this.m_greyTalon.getVelocity().getValueAsDouble() * 60);
    }
}
