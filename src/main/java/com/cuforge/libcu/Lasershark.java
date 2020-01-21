package com.cuforge.libcu;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalSource;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;

/**
 * Class supporting the Copperforge Lasershark LiDAR ranging sensor.
 *
 * <p>The Lasershark (Copperforge 12ft Laser Rangefinder) is an advanced LiDAR
 * ranging sensor that combines the STMicroelectronics VL53L1X laser ranging
 * sensor with advanced signal processing to precisely measure distances to
 * objects between 2" and 12' away, with .2" (5mm) resolution. It offers precise
 * ranging up to 12ft in an easy-to-use form factor and interface, and onboard
 * firmware allows students to interact with this complex sensor with only a
 * standard 3-wire 0.1" pitch cable and minimal programming.
 *
 * <p>To determine distance, the sensor transmits a pulse of laser light, and
 * measures the time for it to bounce back, which can then be converted into a
 * distance measurement given the speed of light. This method is far more
 * accurate than calculating the distance based on reflected light intensity or
 * speed of sound, yielding superior performance over ultrasonic and infrared
 * sensors.
 */
public class Lasershark implements Sendable {

    private DutyCycle _pwmInput;

    /**
     * Construct a new Lasershark on a given digital input channel.
     *
     * @param channel The channel to which to attach.
     */
    public Lasershark(int channel) {
        this._pwmInput = new DutyCycle(new DigitalInput(channel));
        SendableRegistry.addLW(this, "Lasershark", _pwmInput.getFPGAIndex() + 1);
    }

    /**
     * Construct a new Lasershark attached to a DigitalSource object.
     *
     * @param source The digital source to which to attach.
     */
    public Lasershark(DigitalSource source) {
        this._pwmInput = new DutyCycle(source);
        SendableRegistry.addLW(this, "Lasershark", _pwmInput.getFPGAIndex() + 1);
    }

    /**
     * Get the distance reported by the LiDAR sensor in feet.
     */
    public double getDistanceFeet() {
        return this._pwmInput.getOutput() * 4000 / 25.4 / 12;
    }

    /**
     * Get the distance reported by the LiDAR sensor in inches.
     */
    public double getDistanceInches() {
        return this._pwmInput.getOutput() * 4000 / 25.4;
    }

    /**
     * Get the distance reported by the LiDAR sensor in centimeters.
     */
    public double getDistanceCentimeters() {
        return this._pwmInput.getOutput() * 4000 / 10.0;
    }

    /**
     * Get the distance reported by the LiDAR sensor in meters.
     */
    public double getDistanceMeters() {
        return this._pwmInput.getOutput() * 4000 / 1000.0;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Lasershark");
        builder.addDoubleProperty("Distance (ft)", this::getDistanceFeet, null);
    }
}
