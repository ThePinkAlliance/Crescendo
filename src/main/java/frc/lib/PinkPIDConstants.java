// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib;

/** Add your docs here. */
public class PinkPIDConstants {
    /** P */
    public double kP;

    /** I */
    public double kI;

    /** D */
    public double kD;

    /** Integral range */
    public double iZone;

    /**
     * Create a new PIDConstants object
     *
     * @param kP    P
     * @param kI    I
     * @param kD    D
     * @param iZone Integral range
     */
    public PinkPIDConstants(double kP, double kI, double kD, double iZone) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.iZone = iZone;
    }

    /**
     * Create a new PIDConstants object
     *
     * @param kP P
     * @param kI I
     * @param kD D
     */
    public PinkPIDConstants(double kP, double kI, double kD) {
        this(kP, kI, kD, 1.0);
    }

    /**
     * Create a new PIDConstants object
     *
     * @param kP P
     * @param kD D
     */
    public PinkPIDConstants(double kP, double kD) {
        this(kP, 0, kD);
    }

    /**
     * Create a new PIDConstants object
     *
     * @param kP P
     */
    public PinkPIDConstants(double kP) {
        this(kP, 0, 0);
    }

    public void set_kP(double kP) {
        this.kP = kP;
    }

    public void set_kI(double kI) {
        this.kI = kI;
    }

    public void set_kD(double kD) {
        this.kD = kD;
    }

    public void set_iZone(double iZone) {
        this.iZone = iZone;
    }

    public double getkP() {
        return kP;
    }

    public double getkI() {
        return kI;
    }

    public double getkD() {
        return kD;
    }

    public double getiZone() {
        return iZone;
    }
}
