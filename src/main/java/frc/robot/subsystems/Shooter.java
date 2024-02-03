package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

    TalonFX m_greenTalon;
    TalonFX m_greyTalon;

    public Shooter() {
        this.m_greenTalon = new TalonFX(42, "rio");
        this.m_greyTalon = new TalonFX(43, "rio");

        // set slot 0 gains
        var slot0Configs = new Slot0Configs();
        slot0Configs.kS = 0.05; // Add 0.05 V output to overcome static friction
        slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kP = 0.11; // An error of 1 rps results in 0.11 V output
        slot0Configs.kI = 0.5; // An error of 1 rps increases output by 0.5 V each second
        slot0Configs.kD = 0.01; // An acceleration of 1 rps/s results in 0.01 V output

        m_greenTalon.getConfigurator().apply(slot0Configs);
        m_greyTalon.getConfigurator().apply(slot0Configs);

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
        SmartDashboard.putNumber("both Target", 0);
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
            m_greenTalon.setControl(request.withVelocity(bothToRpm).withFeedForward(0.5));
            m_greyTalon.setControl(request.withVelocity(bothToRpm).withFeedForward(0.5));
        } else if (controlIndividual) {
            // set velocity rps, add 0.5 V to overcome gravity
            m_greenTalon.setControl(request.withVelocity(greenToRpm).withFeedForward(0.5));
            m_greyTalon.setControl(request.withVelocity(greyToRpm).withFeedForward(0.5));
        } else {
            DriverStation.reportWarning("!cannot determine motor control! (shooter.java, 95)", true);
        }
    }

    public void setVelocity(double vel) {
        this.setVelocity(vel, vel);
    }

    public void setVelocity(double top, double bottom) {
        double topRpm = top / 60;
        double bottomRpm = bottom / 60;

        // create a velocity closed-loop request, voltage output, slot 0 configs
        var request = new VelocityVoltage(0).withSlot(0);

        // set velocity rps, add 0.5 V to overcome gravity
        m_greenTalon.setControl(request.withVelocity(topRpm).withFeedForward(0.5));
        m_greyTalon.setControl(request.withVelocity(bottomRpm).withFeedForward(0.5));
    }

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

        double t = Math.abs(target);
        double rpm1 = Math.abs(m_greyFXToRPM);
        double rpm2 = Math.abs(m_greenFXToRPM);
        double error = 0.05;
        t = t - (t * error);
        if (rpm1 >= t && rpm2 >= t) {
            result = true;
        }
        return result;

    }

    @Override
    public void periodic() {

    }
}
