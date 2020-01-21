#include <frc/DigitalSource.h>
#include <frc/DutyCycle.h>
#include <frc/smartdashboard/Sendable.h>
#include <frc/smartdashboard/SendableHelper.h>
#include <units/units.h>

namespace libcu
{

/**
 * Class supporting the Copperforge Lasershark LiDAR ranging sensor.
 *
 * The Lasershark (Copperforge 12ft Laser Rangefinder) is an advanced LiDAR
 * ranging sensor that combines the STMicroelectronics VL53L1X laser ranging
 * sensor with advanced signal processing to precisely measure distances to
 * objects between 2" and 12' away, with .2" (5mm) resolution. It offers precise
 * ranging up to 12ft in an easy-to-use form factor and interface, and onboard
 * firmware allows students to interact with this complex sensor with only a
 * standard 3-wire 0.1" pitch cable and minimal programming.
 *
 * To determine distance, the sensor transmits a pulse of laser light, and
 * measures the time for it to bounce back, which can then be converted into a
 * distance measurement given the speed of light. This method is far more
 * accurate than calculating the distance based on reflected light intensity or
 * speed of sound, yielding superior performance over ultrasonic and infrared
 * sensors.
 */
class Lasershark : public frc::Sendable,
                   public frc::SendableHelper<Lasershark>
{
public:
    /**
     * Construct a new Lasershark on a given digital input channel.
     *
     * @param channel The channel to which to attach.
     */
    explicit Lasershark(int channel);

    /**
     * Construct a new Lasershark attached to a DigitalSource object.
     *
     * @param source The digital source to which to attach.
     */
    explicit Lasershark(frc::DigitalSource &source);

    /**
     * Construct a new Lasershark attached to a DigitalSource object.
     *
     * @param source The digital source to which to attach.
     */
    explicit Lasershark(frc::DigitalSource *source);

    /**
     * Construct a new Lasershark attached to a DigitalSource object.
     *
     * @param source The digital source to which to attach.
     */
    explicit Lasershark(std::shared_ptr<frc::DigitalSource> source);

    Lasershark(Lasershark&&) = default;
    Lasershark& operator=(Lasershark&&) = default;

    /**
     * Get the distance reported by the LiDAR sensor.
     */
    units::foot_t GetDistance();

protected:
    void InitSendable(frc::SendableBuilder& builder) override;

private:
    frc::DutyCycle pwmInput;
};
} // namespace libcu
