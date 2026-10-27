#include "SCCVoiceUnit.hh"

#include "serialize.hh"

namespace openmsx {

SCCVoiceUnit::SCCVoiceUnit(const std::string& name, const DeviceConfig& config,
                           EmuTime time, SCCVoiceRomBypassReader& romReader)
	: voiceA(name + " SCC Voice A", config, time, romReader)
	, voiceB(name + " SCC Voice B", config, time, romReader)
{
}

void SCCVoiceUnit::reset(EmuTime time)
{
	voiceA.reset(time);
	voiceB.reset(time);
	voiceSelect = 0;
}

SCCVoiceCore* SCCVoiceUnit::voiceAt(uint8_t offset)
{
	if ((offset & 0x0F) == REG_NUM) return nullptr; // 0xEF and 0xFF are free
	if ((offset & 0xF0) == VOICE_A_BASE) return &voiceA;
	if ((offset & 0xF0) == VOICE_B_BASE) return &voiceB;
	return nullptr;
}

const SCCVoiceCore* SCCVoiceUnit::voiceAt(uint8_t offset) const
{
	return const_cast<SCCVoiceUnit*>(this)->voiceAt(offset);
}

std::optional<uint8_t> SCCVoiceUnit::read(SCCVoiceCore& v, uint8_t reg, EmuTime time)
{
	switch (reg) {
	case REG_ROM_FILL_LEVEL:    return v.readRomFillLevel(time);
	case REG_CTRL_STATUS:       return v.readStatus(time);
	case REG_PITCH_L:           return v.getPitchLow();
	case REG_PITCH_H:           return v.getPitchHigh();
	case REG_VOLUME:            return v.getVolume();
	case REG_ROM_START_L:       return v.readRomStartAddrLow(time);
	case REG_ROM_START_H:       return v.readRomStartAddrHigh(time);
	case REG_ROM_END_L:
	case REG_ROM_END_H:
	case REG_ROM_REPEAT_ADDR_L:
	case REG_ROM_REPEAT_ADDR_H:
	case REG_STEP_INDEX:
	case REG_PREDICTOR_L:
	case REG_PREDICTOR_H:       return 0xFF; // write-only
	case REG_DATA_LEVEL:        return v.readLevel(time);
	default:                    return std::nullopt;
	}
}

std::optional<uint8_t> SCCVoiceUnit::peek(const SCCVoiceCore& v, uint8_t reg, EmuTime time)
{
	switch (reg) {
	case REG_ROM_FILL_LEVEL:    return v.peekRomFillLevel(time);
	case REG_CTRL_STATUS:       return v.peekStatus(time);
	case REG_PITCH_L:           return v.getPitchLow();
	case REG_PITCH_H:           return v.getPitchHigh();
	case REG_VOLUME:            return v.getVolume();
	case REG_ROM_START_L:       return v.peekRomStartAddrLow(time);
	case REG_ROM_START_H:       return v.peekRomStartAddrHigh(time);
	case REG_ROM_END_L:
	case REG_ROM_END_H:
	case REG_ROM_REPEAT_ADDR_L:
	case REG_ROM_REPEAT_ADDR_H:
	case REG_STEP_INDEX:
	case REG_PREDICTOR_L:
	case REG_PREDICTOR_H:       return 0xFF; // write-only
	case REG_DATA_LEVEL:        return v.peekLevel(time);
	default:                    return std::nullopt;
	}
}

void SCCVoiceUnit::write(SCCVoiceCore& v, uint8_t reg, uint8_t value, EmuTime time)
{
	switch (reg) {
	case REG_ROM_FILL_LEVEL:    v.writeRomFillLevel(value, time); break;
	case REG_CTRL_STATUS:       v.writeControl(value, time); break;
	case REG_PITCH_L:           v.writePitchLow(value, time); break;
	case REG_PITCH_H:           v.writePitchHigh(value, time); break;
	case REG_VOLUME:            v.writeVolume(value, time); break;
	case REG_ROM_START_L:       v.writeRomStartAddrLow(value, time); break;
	case REG_ROM_START_H:       v.writeRomStartAddrHigh(value, time); break;
	case REG_ROM_END_L:         v.writeRomEndAddrLow(value, time); break;
	case REG_ROM_END_H:         v.writeRomEndAddrHigh(value, time); break;
	case REG_ROM_REPEAT_ADDR_L: v.writeRomRepeatAddrLow(value, time); break;
	case REG_ROM_REPEAT_ADDR_H: v.writeRomRepeatAddrHigh(value, time); break;
	case REG_STEP_INDEX:        v.writeStepIndexOverride(value, time); break;
	case REG_PREDICTOR_L:       v.writePredictorLow(value, time); break;
	case REG_PREDICTOR_H:       v.writePredictorHigh(value, time); break;
	case REG_DATA_LEVEL:        v.writeData(value, time); break;
	default:                    break;
	}
}

std::optional<uint8_t> SCCVoiceUnit::readRegister(uint8_t offset, EmuTime time)
{
	auto* v = voiceAt(offset);
	if (!v) return std::nullopt;
	return read(*v, offset & 0x0F, time);
}

std::optional<uint8_t> SCCVoiceUnit::peekRegister(uint8_t offset, EmuTime time) const
{
	const auto* v = voiceAt(offset);
	if (!v) return std::nullopt;
	return peek(*v, offset & 0x0F, time);
}

bool SCCVoiceUnit::writeRegister(uint8_t offset, uint8_t value, EmuTime time)
{
	auto* v = voiceAt(offset);
	if (!v) return false;
	write(*v, offset & 0x0F, value, time);
	return true;
}

uint8_t SCCVoiceUnit::readPort(uint16_t port, EmuTime time)
{
	switch (port & 0xFF) {
	case IO_DATA_LEVEL:   return selected().readLevel(time);
	case IO_CTRL_STATUS:  return selected().readStatus(time);
	case IO_VOICE_SELECT: return voiceSelect;
	default:              return 0xFF; // STEP_INDEX / PREDICTOR_L / PREDICTOR_H: write-only
	}
}

uint8_t SCCVoiceUnit::peekPort(uint16_t port, EmuTime time) const
{
	switch (port & 0xFF) {
	case IO_DATA_LEVEL:   return selected().peekLevel(time);
	case IO_CTRL_STATUS:  return selected().peekStatus(time);
	case IO_VOICE_SELECT: return voiceSelect;
	default:              return 0xFF;
	}
}

void SCCVoiceUnit::writePort(uint16_t port, uint8_t value, EmuTime time)
{
	switch (port & 0xFF) {
	case IO_DATA_LEVEL:   selected().writeData(value, time); break;
	case IO_CTRL_STATUS:  selected().writeControl(value, time); break;
	case IO_STEP_INDEX:   selected().writeStepIndexOverride(value, time); break;
	case IO_PREDICTOR_L:  selected().writePredictorLow(value, time); break;
	case IO_PREDICTOR_H:  selected().writePredictorHigh(value, time); break;
	case IO_VOICE_SELECT: voiceSelect = value & 0x01; break;
	default: break;
	}
}

template<typename Archive>
void SCCVoiceUnit::serialize(Archive& ar, unsigned /*version*/)
{
	ar.serialize("voiceA",      voiceA,
	             "voiceB",      voiceB,
	             "voiceSelect", voiceSelect);
}
INSTANTIATE_SERIALIZE_METHODS(SCCVoiceUnit);

} // namespace openmsx
