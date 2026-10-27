#include "MSXSCCVoice.hh"

#include "File.hh"
#include "FileContext.hh"
#include "FileException.hh"
#include "XMLElement.hh"
#include "CacheLine.hh"

#include "enumerate.hh"
#include "narrow.hh"
#include "ranges.hh"
#include "serialize.hh"

#include <algorithm>
#include <bit>

namespace openmsx {

// Identical mapper-config logic to MSXSCCPlusCart (subtypes: expanded,
// Snatcher, SD-Snatcher, mirrored, Popolon). SCCボイス doesn't add any new
// subtypes; the sample voices are orthogonal to memory mapping.
static auto getMapperConfig(const DeviceConfig& config)
{
	MSXSCCVoice::MapperConfig result;

	std::string_view subtype = config.getChildData("subtype", "expanded");
	if (subtype == "Snatcher") {
		result.numBlocks = 8;
		result.registerMask = 0b0000'1111;
		result.registerOffset = 0;
	} else if (subtype == "SD-Snatcher") {
		result.numBlocks = 8;
		result.registerMask = 0b0000'1111;
		result.registerOffset = 8;
	} else if (subtype == "mirrored") {
		result.numBlocks = 8;
		result.registerMask = 0b0000'0111;
		result.registerOffset = 0;
	} else if (subtype == "Popolon") {
		unsigned ramSize = config.getChildDataAsInt("size", 2048);
		if (!std::has_single_bit(ramSize)) {
			throw MSXException(
				"Popolon type Sound Cartridge must have a power-of-2 RAM size: ",
				ramSize);
		}
		if (ramSize < 128 || ramSize > 2048) {
			throw MSXException(
				"Popolon type Sound Cartridge must have a size between 128kB and 2048kB: ",
				ramSize);
		}
		result.numBlocks = ramSize / 8;
		result.registerMask = narrow<byte>(result.numBlocks - 1);
		result.registerOffset = 0;
	} else {
		result.numBlocks = 16;
		result.registerMask = 0b0000'1111;
		result.registerOffset = 0;
	}
	return result;
}


MSXSCCVoice::MSXSCCVoice(const DeviceConfig& config)
	: MSXDevice(config)
	, mapperConfig(getMapperConfig(config))
	, ram(config, getName() + " RAM", "SCC-Voice RAM", size_t(mapperConfig.numBlocks) * 0x2000)
	, scc(getName(), config, getCurrentTime(), SCC::Mode::Compatible)
	, voice(getName(), config, getCurrentTime(), *this)
	, romBlockDebug(*this, mapper, 0x4000, 0x8000, 13)
{
	if (const auto* fileElem = config.findChild("filename")) {
		const auto& filename = fileElem->getData();
		try {
			File file(config.getFileContext().resolve(filename));
			auto size = std::min(file.getSize(), ram.size());
			file.read(subspan(ram, 0, size));
		} catch (FileException&) {
			throw MSXException("Error reading file: ", filename);
		}
	}
	std::ranges::fill(isRamSegment, true);
	std::ranges::fill(mapper, 0);

	powerUp(getCurrentTime());
}

void MSXSCCVoice::powerUp(EmuTime time)
{
	scc.powerUp(time);
	voice.reset(time);
	ram.clear();
	reset(time);
}

void MSXSCCVoice::reset(EmuTime time)
{
	setModeRegister(0);
	setMapper(0, 0);
	setMapper(1, 1);
	setMapper(2, 2);
	setMapper(3, 3);
	scc.reset(time);
	voice.reset(time);
}

bool MSXSCCVoice::isSccVisible(uint16_t address) const
{
	return ((enable == EN_SCC)     && (0x9800 <= address) && (address < 0xA000)) ||
	       ((enable == EN_SCCPLUS) && (0xB800 <= address) && (address < 0xC000));
	// SCC  visible in 0x9800 - 0x9FFF
	// SCC+ visible in 0xB800 - 0xBFFF
}

byte MSXSCCVoice::readMem(uint16_t addr, EmuTime time)
{
	if (isSccVisible(addr)) {
		auto offset = narrow_cast<uint8_t>(addr & 0xFF);
		if (auto value = voice.readRegister(offset, time)) {
			return *value;
		}
		return scc.readMem(offset, time);
	} else {
		return MSXSCCVoice::peekMem(addr, time);
	}
}

byte MSXSCCVoice::peekMem(uint16_t addr, EmuTime time) const
{
	// modeRegister can not be read!
	if (isSccVisible(addr)) {
		auto offset = narrow_cast<uint8_t>(addr & 0xFF);
		if (auto value = voice.peekRegister(offset, time)) {
			return *value;
		}
		return scc.peekMem(offset, time);
	} else if ((0x4000 <= addr) && (addr < 0xC000)) {
		// SCC(+) enabled/disabled but not requested so memory stuff
		return internalMemoryBank[(addr >> 13) - 2][addr & 0x1FFF];
	} else {
		// outside memory range
		return 0xFF;
	}
}

const byte* MSXSCCVoice::getReadCacheLine(uint16_t start) const
{
	if (isSccVisible(start)) {
		// SCC and voice registers must go through readMem() every time
		return nullptr;
	} else if ((0x4000 <= start) && (start < 0xC000)) {
		return &internalMemoryBank[(start >> 13) - 2][start & 0x1FFF];
	} else {
		return unmappedRead.data();
	}
}

byte MSXSCCVoice::peekUnderlyingRom(uint16_t address, EmuTime /*time*/) const
{
	// Deliberately does NOT check isSccVisible() -- the whole point is
	// to return the raw underlying content even where the SCC(+)
	// register window would otherwise "steal" this address. Same
	// indexing as the normal (non-SCC-window) memory path, just without
	// that gate.
	if ((0x4000 <= address) && (address < 0xC000)) {
		return internalMemoryBank[(address >> 13) - 2][address & 0x1FFF];
	}
	return 0xFF; // outside this device's own memory range
}


void MSXSCCVoice::writeMem(uint16_t address, byte value, EmuTime time)
{
	if ((address < 0x4000) || (0xC000 <= address)) {
		// outside memory range
		return;
	}

	// Mode register is mapped upon 0xBFFE and 0xBFFF
	if ((address | 0x0001) == 0xBFFF) {
		setModeRegister(value);
		return;
	}

	// Write to RAM
	int region = (address >> 13) - 2;
	if (isRamSegment[region]) {
		if (isMapped[region]) {
			internalMemoryBank[region][address & 0x1FFF] = value;
		}
		return;
	}

	/* Write to bank-switching registers
	 * The address to change banks:
	 *   bank 1: 0x5000 - 0x57FF (0x5000 used)
	 *   bank 2: 0x7000 - 0x77FF (0x7000 used)
	 *   bank 3: 0x9000 - 0x97FF (0x9000 used)
	 *   bank 4: 0xB000 - 0xB7FF (0xB000 used)
	 */
	if ((address & 0x1800) == 0x1000) {
		setMapper(region, value);
		return;
	}

	if (!isSccVisible(address)) {
		// SCC(+) not enabled/requested at this address: do nothing
		return;
	}

	auto offset = narrow_cast<uint8_t>(address & 0xFF);
	if (!voice.writeRegister(offset, value, time)) {
		scc.writeMem(offset, value, time);
	}
}

// ---------------------------------------------------------------------
// I/O port fast path (0x74-0x79) -- see class comment in the .hh file.
// No SCC(+) enable check here, unlike the memory-mapped path: I/O ports
// are a completely separate address space from the SCC's memory window,
// so there's no ambiguity to resolve (nothing else could ever claim
// these ports out from under us the way memory contention happens on the
// 0x9800/0xB800 windows). The ports are decoded by SCCVoiceUnit, onto the
// same SCCVoiceCore methods the memory-mapped registers use -- intentionally no
// separate logic path to keep in sync.
// ---------------------------------------------------------------------

byte MSXSCCVoice::readIO(uint16_t port, EmuTime time)
{
	return voice.readPort(port, time);
}

byte MSXSCCVoice::peekIO(uint16_t port, EmuTime time) const
{
	return voice.peekPort(port, time);
}

void MSXSCCVoice::writeIO(uint16_t port, byte value, EmuTime time)
{
	voice.writePort(port, value, time);
}

byte* MSXSCCVoice::getWriteCacheLine(uint16_t start)
{
	if ((0x4000 <= start) && (start < 0xC000)) {
		if (start == (0xBFFF & CacheLine::HIGH)) {
			return nullptr;
		}
		if (int region = (start >> 13) - 2;
		    isRamSegment[region] && isMapped[region]) {
			return &internalMemoryBank[region][start & 0x1FFF];
		}
		return nullptr;
	}
	return unmappedWrite.data();
}


void MSXSCCVoice::setMapper(int region, byte value)
{
	mapper[region] = value;

	value &= mapperConfig.registerMask;
	value -= mapperConfig.registerOffset;
	byte* block = [&] {
		if (value < mapperConfig.numBlocks) {
			isMapped[region] = true;
			return &ram[0x2000 * value];
		} else {
			isMapped[region] = false;
			return unmappedRead.data();
		}
	}();

	checkEnable(); // invalidateDeviceRWCache() done below
	internalMemoryBank[region] = block;
	invalidateDeviceRWCache(0x4000 + region * 0x2000, 0x2000);
}

void MSXSCCVoice::setModeRegister(byte value)
{
	modeRegister = value;
	checkEnable(); // invalidateDeviceRWCache() done below

	if (modeRegister & 0x20) {
		scc.setMode(SCC::Mode::Plus);
	} else {
		scc.setMode(SCC::Mode::Compatible);
	}

	if (modeRegister & 0x10) {
		isRamSegment[0] = true;
		isRamSegment[1] = true;
		isRamSegment[2] = true;
		isRamSegment[3] = true;
	} else {
		isRamSegment[0] = (modeRegister & 0x01) == 0x01;
		isRamSegment[1] = (modeRegister & 0x02) == 0x02;
		isRamSegment[2] = (modeRegister & 0x24) == 0x24; // extra requirement: SCC+ mode
		isRamSegment[3] = false;
	}
	invalidateDeviceRWCache(0x4000, 0x8000);
}

void MSXSCCVoice::checkEnable()
{
	if ((modeRegister & 0x20) && (mapper[3] & 0x80)) {
		enable = EN_SCCPLUS;
	} else if ((!(modeRegister & 0x20)) && ((mapper[2] & 0x3F) == 0x3F)) {
		enable = EN_SCC;
	} else {
		enable = EN_NONE;
	}
}


template<typename Archive>
void MSXSCCVoice::serialize(Archive& ar, unsigned /*version*/)
{
	ar.serialize_blob("ram", std::span{ram});
	ar.serialize("scc",   scc,
	             "voice", voice,
	             "mapper", mapper,
	             "mode",   modeRegister);

	if constexpr (Archive::IS_LOADER) {
		for (auto [i, m] : enumerate(mapper)) {
			setMapper(int(i), m);
		}
		setModeRegister(modeRegister);
	}
}
INSTANTIATE_SERIALIZE_METHODS(MSXSCCVoice);
REGISTER_MSXDEVICE(MSXSCCVoice, "SCCVoice");

} // namespace openmsx
