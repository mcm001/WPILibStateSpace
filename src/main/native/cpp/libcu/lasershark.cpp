#include <libcu/lasershark.h>
#include <frc/DigitalInput.h>
#include <frc/smartdashboard/SendableRegistry.h>
#include <frc/smartdashboard/SendableBuilder.h>

namespace libcu
{

Lasershark::Lasershark(int channel)
    : pwmInput{std::make_shared<frc::DigitalInput>(channel)}
{
    frc::SendableRegistry::GetInstance().AddLW(this, "Lasershark", pwmInput.GetFPGAIndex() + 1);
}

Lasershark::Lasershark(frc::DigitalSource &source)
    : pwmInput{source}
{
    frc::SendableRegistry::GetInstance().AddLW(this, "Lasershark", pwmInput.GetFPGAIndex() + 1);
}

Lasershark::Lasershark(frc::DigitalSource *source)
    : pwmInput{source}
{
    frc::SendableRegistry::GetInstance().AddLW(this, "Lasershark", pwmInput.GetFPGAIndex() + 1);
}

Lasershark::Lasershark(std::shared_ptr<frc::DigitalSource> source)
    : pwmInput{source}
{
    frc::SendableRegistry::GetInstance().AddLW(this, "Lasershark", pwmInput.GetFPGAIndex() + 1);
}

units::foot_t Lasershark::GetDistance()
{
    return units::foot_t{pwmInput.GetOutput() * 4000 / 25.4 / 12.0};
}

void Lasershark::InitSendable(frc::SendableBuilder &builder)
{
    builder.SetSmartDashboardType("Lasershark");
    builder.AddDoubleProperty("Distance (ft)",
                              [this] { return this->GetDistance().to<double>(); }, nullptr);
}

} // namespace libcu
