// Konami SCC/SCC+ dual-mode cartridge mapper, extended with the SCCボイス
// sample voices A and B. See the .hh file for how this combines
// RomKonamiSCC's ROM bank-switching with MSXSCCVoice's dual-mode
// enable logic.

#include "RomKonamiSCCPlusVoice.hh"
#include "CacheLine.hh"
#include "MSXMotherBoard.hh"
#include "MSXCliComm.hh"
#include "MSXCPUInterface.hh"
#include "narrow.hh"
#include "serialize.hh"
#include "xrange.hh"

namespace openmsx {

RomKonamiSCCPlusVoice::RomKonamiSCCPlusVoice(const DeviceConfig& config, Rom&& rom_, unsigned ramSize)
	: Rom8kBBlocks(config, std::move(rom_))
	, scc(getName() + " SCC", config, getCurrentTime(), SCC::Mode::Compatible)
	, voice(getName(), config, getCurrentTime(), *this)
{
	if (ramSize) {
		ram.emplace(config, getName() + " RAM", "SCC-Voice sample RAM", ramSize);
		ram->clear();
	}
	if (rom.size() > 2048 * 1024) {
		getMotherBoard().getMSXCliComm().printWarning(
			"The size of this ROM image is larger than 2048kB, "
			"which is not supported on real Konami SCC+ mapper "
			"chips!");
	}
	// Manual I/O port registration -- see .hh class comment for why this
	// can't go through the normal XML-driven registerPorts() mechanism
	// for a ROM cart loaded via -romtype.
	getCPUInterface().register_IO_InOut_range(SCCVoiceUnit::IO_BASE, SCCVoiceUnit::IO_NUM, this);
	powerUp(getCurrentTime());
}

RomKonamiSCCPlusVoice::~RomKonamiSCCPlusVoice()
{
	getCPUInterface().unregister_IO_InOut_range(SCCVoiceUnit::IO_BASE, SCCVoiceUnit::IO_NUM, this);
}

void RomKonamiSCCPlusVoice::powerUp(EmuTime time)
{
	scc.powerUp(time);
	voice.reset(time);
	reset(time);
}

void RomKonamiSCCPlusVoice::bankSwitch(unsigned page, unsigned block)
{
	setRom(page, block);

	// Same mirroring behaviour as RomKonamiSCC.
	if (page == 2 || page == 3) {
		// [0x4000-0x8000), mirrored in [0xC000-0x10000)
		setRom(page + 4, block);
	} else if (page == 4 || page == 5) {
		// [0x8000-0xC000), mirrored in [0x0000-0x4000)
		setRom(page - 4, block);
	} else {
		assert(false);
	}
}

void RomKonamiSCCPlusVoice::reset(EmuTime time)
{
	for (auto i : xrange(2u, 6u)) {
		bankValue[i - 2] = narrow_cast<byte>(i - 2);
		bankSwitch(i, i - 2);
	}

	modeRegister = 0;
	enable = EN_NONE;
	scc.setMode(SCC::Mode::Compatible);
	scc.reset(time);
	voice.reset(time);
}

bool RomKonamiSCCPlusVoice::ramVisible(uint16_t address) const
{
	return ram && (modeRegister & MODE_RAM_WINDOW) && (0x8000 <= address) && (address < 0xC000);
}

bool RomKonamiSCCPlusVoice::ramWritable(uint16_t address) const
{
	if (!ramVisible(address)) return false;
	if (modeRegister & MODE_ALL_WRITE) return true;
	// page 4 also with bit 2, in SCC+ mode only (as on the SCC-I); page 5 only with bit 4
	return (address < 0xA000) &&
	       ((modeRegister & (MODE_PAGE4_WRITE | MODE_SCC_PLUS)) == (MODE_PAGE4_WRITE | MODE_SCC_PLUS));
}

size_t RomKonamiSCCPlusVoice::ramOffset(uint16_t address) const
{
	unsigned page = address >> 13;
	return size_t(bankValue[page - 2] & RAM_BANK_MASK) * 0x2000 + (address & 0x1FFF);
}

bool RomKonamiSCCPlusVoice::isSccVisible(uint16_t address) const
{
	return ((enable == EN_SCC)     && (0x9800 <= address) && (address < 0xA000)) ||
	       ((enable == EN_SCCPLUS) && (0xB800 <= address) && (address < 0xC000));
	// SCC  visible in 0x9800 - 0x9FFF
	// SCC+ visible in 0xB800 - 0xBFFF
}

byte RomKonamiSCCPlusVoice::peekMem(uint16_t address, EmuTime time) const
{
	// modeRegister can not be read!
	if (isSccVisible(address)) {
		auto offset = narrow_cast<uint8_t>(address & 0xFF);
		if (auto value = voice.peekRegister(offset, time)) {
			return *value;
		}
		return scc.peekMem(offset, time);
	} else if (ramVisible(address)) {
		return (*ram)[ramOffset(address)];
	} else {
		return Rom8kBBlocks::peekMem(address, time);
	}
}

byte RomKonamiSCCPlusVoice::readMem(uint16_t address, EmuTime time)
{
	if (isSccVisible(address)) {
		auto offset = narrow_cast<uint8_t>(address & 0xFF);
		if (auto value = voice.readRegister(offset, time)) {
			return *value;
		}
		return scc.readMem(offset, time);
	} else if (ramVisible(address)) {
		return (*ram)[ramOffset(address)];
	} else {
		return Rom8kBBlocks::readMem(address, time);
	}
}

const byte* RomKonamiSCCPlusVoice::getReadCacheLine(uint16_t address) const
{
	if (isSccVisible(address)) {
		// SCC and voice registers must go through readMem() every time
		return nullptr;
	} else if (ramVisible(address)) {
		// RAM view: not cached (the bank and the mode can change it at any write)
		return nullptr;
	} else {
		return Rom8kBBlocks::getReadCacheLine(address);
	}
}

void RomKonamiSCCPlusVoice::writeMem(uint16_t address, byte value, EmuTime time)
{
	if ((address < 0x5000) || (address >= 0xC000)) {
		return;
	}

	// Mode register is mapped upon 0xBFFE and 0xBFFF
	if ((address | 0x0001) == 0xBFFF) {
		setModeRegister(value);
		return;
	}

	// A page that shows writable RAM takes every write, also on the register addresses (as the SCC-I)
	if (ramWritable(address)) {
		(*ram)[ramOffset(address)] = value;
		return;
	}

	if (isSccVisible(address)) {
		auto offset = narrow_cast<uint8_t>(address & 0xFF);
		if (!voice.writeRegister(offset, value, time)) {
			scc.writeMem(offset, value, time);
		}
		return;
	}

	/* Bank-switching trigger addresses:
	 *   page 2 (0x4000-0x5FFF): 0x5000-0x57FF (0x5000 used)
	 *   page 3 (0x6000-0x7FFF): 0x7000-0x77FF (0x7000 used)
	 *   page 4 (0x8000-0x9FFF): 0x9000-0x97FF (0x9000 used)
	 *   page 5 (0xA000-0xBFFF): 0xB000-0xB7FF (0xB000 used)
	 */
	if ((address & 0x1800) == 0x1000) {
		unsigned page = address >> 13;
		bankValue[page - 2] = value;
		// Bit 7 is NOT part of the bank number: the bank is selected by
		// bits 6-0 only (so banks 0-127 are reachable through the plain
		// registers). Bit 7 keeps its separate meaning for page 5 -- it
		// is the SCC+ enable flag, read from the raw value by
		// checkEnable() below. This lets page 5 select ANY bank 0-127
		// while SCC+ stays enabled (write 0x80|bank), which is what
		// makes the whole 0x8000-0xBFFF range usable for sample data.
		// Banks above 127 would need a separate offset register (as on
		// the Yamanooto's OFFR), which this model does not implement.
		bankSwitch(page, value & 0x7F);
		checkEnable();
		// The bank value can switch the SCC(+) register window on or off
		// (page 4: low 6 bits 0x3F, page 5: bit 7), so the CPU must not
		// keep a cached ROM view of it -- same as setModeRegister().
		invalidateDeviceRCache(0x9800, 0x0800);
		invalidateDeviceRCache(0xB800, 0x0800);
		if (ram && (page == 4 || page == 5)) {
			// setRom() filled the CPU read cache with the ROM block; with the
			// RAM window set this page shows RAM, so let getReadCacheLine() decide
			invalidateDeviceRCache(page * 0x2000, 0x2000);
		}
	}
}

// ---------------------------------------------------------------------
// I/O port fast path (0x74-0x79) -- see .hh class comment. No SCC-enable
// check here, unlike the memory-mapped path: I/O ports are a completely
// separate address space, so there's no ambiguity to resolve. The ports
// are decoded by SCCVoiceUnit, onto the same SCCVoiceCore methods the
// memory-mapped registers use -- intentionally no separate logic path to
// keep in sync.
// ---------------------------------------------------------------------

byte RomKonamiSCCPlusVoice::readIO(uint16_t port, EmuTime time)
{
	return voice.readPort(port, time);
}

byte RomKonamiSCCPlusVoice::peekIO(uint16_t port, EmuTime time) const
{
	return voice.peekPort(port, time);
}

void RomKonamiSCCPlusVoice::writeIO(uint16_t port, byte value, EmuTime time)
{
	voice.writePort(port, value, time);
}

byte RomKonamiSCCPlusVoice::peekUnderlyingRom(uint16_t address, EmuTime time) const
{
	// Deliberately does NOT check isSccVisible(): the whole point is to
	// return the raw ROM content as currently banked in, even where the
	// SCC(+) register window would otherwise be visible instead. The base
	// class' peekMem() is exactly that -- it indexes the mapper's current
	// bank pointers with no register decoding. With the RAM window set the
	// fill engine reads the sample RAM instead.
	if (ramVisible(address)) {
		return (*ram)[ramOffset(address)];
	}
	return Rom8kBBlocks::peekMem(address, time);
}

byte* RomKonamiSCCPlusVoice::getWriteCacheLine(uint16_t address)
{
	if ((address < 0x5000) || (address >= 0xC000)) {
		return unmappedWrite.data();
	} else if (address == (0xBFFF & CacheLine::HIGH)) {
		// mode register
		return nullptr;
	} else if (isSccVisible(address)) {
		// write to SCC or a voice
		return nullptr;
	} else if ((address & 0x1800) == (0x1000 & CacheLine::HIGH)) {
		// bank-switch trigger addresses
		return nullptr;
	} else if (ram && (0x8000 <= address)) {
		// the sample window: RAM writes depend on the mode register
		return nullptr;
	} else {
		return unmappedWrite.data();
	}
}

void RomKonamiSCCPlusVoice::setModeRegister(byte value)
{
	modeRegister = value;
	checkEnable();

	if (modeRegister & 0x20) {
		scc.setMode(SCC::Mode::Plus);
	} else {
		scc.setMode(SCC::Mode::Compatible);
	}
	invalidateDeviceRCache(0x9800, 0x0800);
	invalidateDeviceRCache(0xB800, 0x0800);
	if (ram) {
		// RAM_WINDOW and the write enables change what pages 4+5 read and write
		invalidateDeviceRWCache(0x8000, 0x4000);
	}
}

void RomKonamiSCCPlusVoice::checkEnable()
{
	// bankValue holds the RAW written byte. For page 5, bit 7 is the SCC+
	// enable flag; it is deliberately NOT part of the bank number (see the
	// bank-switch write path), so it can be set while selecting any of
	// banks 0-127. For page 4, SCC (Compatible) is enabled while the low
	// 6 bits are all set.
	if ((modeRegister & 0x20) && (bankValue[3] & 0x80)) {
		enable = EN_SCCPLUS;
	} else if (!(modeRegister & 0x20) && ((bankValue[2] & 0x3F) == 0x3F)) {
		enable = EN_SCC;
	} else {
		enable = EN_NONE;
	}
}

template<typename Archive>
void RomKonamiSCCPlusVoice::serialize(Archive& ar, unsigned /*version*/)
{
	ar.template serializeBase<Rom8kBBlocks>(*this);
	ar.serialize("scc",         scc,
	             "voice", voice,
	             "bankValue",   bankValue,
	             "modeRegister",modeRegister);
	if (ram) {
		ar.serialize_blob("ram", std::span{*ram});
	}

	if constexpr (Archive::IS_LOADER) {
		// Rom8kBBlocks' own base serialization already restored the
		// actual ROM page mapping; we only need to re-derive the
		// enable/SCC-mode state from the restored mode register and
		// bank values.
		checkEnable();
		scc.setMode((modeRegister & 0x20) ? SCC::Mode::Plus
		                                   : SCC::Mode::Compatible);
	}
}
INSTANTIATE_SERIALIZE_METHODS(RomKonamiSCCPlusVoice);
REGISTER_MSXDEVICE(RomKonamiSCCPlusVoice, "RomKonamiSCCPlusVoice");

} // namespace openmsx
