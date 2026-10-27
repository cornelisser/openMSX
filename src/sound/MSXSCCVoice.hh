// SCCボイス (SCC-Voice): an SCC+/SCC-I cartridge extended with two sample
// voices, A and B (see SCCVoiceUnit.hh for the voices and the complete
// register map). The 5 existing SCC channels, wave RAM, mapper and
// mode-register behaviour are byte-for-byte identical to MSXSCCPlusCart; this
// class only adds the voice registers on top.
//
// The two voices are identical: an MSM5205-compatible ADPCM decoder (also
// RAW8/RAW4) with a 255-byte FIFO, an SCC-style period counter, a ROM fill
// engine with loops, and a click-free loop override.
//
// The SCC+/SCC-I chip only uses register offsets 0x00-0xDF within its
// visible 0x800-byte window (wave RAM, frequency/volume/enable, and the
// deformation register at 0xC0-0xDF); offsets 0xE0-0xFF have no function
// there, in SCC and in SCC+ mode alike. SCCボイス uses that range: voice A at
// 0xE0-0xEE, voice B at 0xF0-0xFE (0xEF and 0xFF stay free). (On the
// ORIGINAL SCC, 0xE0-0xFF is the deformation register; see SCCVoiceUnit.hh.)
// Several offsets are read/write register PAIRS -- a write and a read to
// the same address are independent operations (e.g. DATA/LEVEL, CTRL/STATUS).
//
// I/O port fast path (0x74-0x78 for the voice chosen with 0x79; RomKonamiSCCPlusVoice
// provides it too, see there): when this device lives in a different slot
// than the driver's own sample-data RAM, every memory-mapped register touch
// requires switching that RAM out of the CPU page temporarily, which is a
// genuine problem for a per-interrupt FIFO refill loop that also needs to
// keep reading its own sample buffer. I/O ports are a separate address
// space, entirely independent of slot switching, which sidesteps this.
//
// Exposed this way: the two registers touched every interrupt (DATA/LEVEL,
// CTRL/STATUS), plus the loop-override triplet, which on the manual
// (CPU-fed) path must be rewritten at EVERY loop wrap -- for a short loop as
// often as a normal FIFO refill. PITCH_L/H and VOLUME stay memory-mapped
// only, being written once per note:
//   0x74  write=DATA (push FIFO byte)         read=LEVEL
//   0x75  write=CTRL (START/STOP/etc.)        read=STATUS
//   0x76  write=STEP_INDEX (loop-override staging)
//   0x77  write=PREDICTOR_L (loop-override staging)
//   0x78  write=PREDICTOR_H (loop-override arm/commit; write order:
//         STEP, then PREDICTOR_L, then PREDICTOR_H last)
//   0x79  VOICE_SELECT, read/write: bit 0 = 0 voice A, 1 voice B (reset 0)
// Chosen from a genuinely unclaimed gap in the full MSX I/O port map
// (grauw.nl/resources/msx_io_ports.php) -- no known hardware, standard or
// otherwise, uses 0x74-0x7B.

#ifndef MSXSCCVOICE_HH
#define MSXSCCVOICE_HH

#include "MSXDevice.hh"
#include "SCC.hh"
#include "SCCVoiceCore.hh"
#include "SCCVoiceUnit.hh"
#include "Ram.hh"
#include "RomBlockDebuggable.hh"

#include <array>
#include <cstdint>

namespace openmsx {

class MSXSCCVoice final : public MSXDevice, public SCCVoiceRomBypassReader
{
public:
	struct MapperConfig {
		unsigned numBlocks;  // number of 8kB blocks
		byte registerMask;   // mapper selection bits to ignore
		byte registerOffset; // first mapped block
	};

public:
	explicit MSXSCCVoice(const DeviceConfig& config);

	void powerUp(EmuTime time) override;
	void reset(EmuTime time) override;
	[[nodiscard]] byte readMem(uint16_t address, EmuTime time) override;
	[[nodiscard]] byte peekMem(uint16_t address, EmuTime time) const override;
	void writeMem(uint16_t address, byte value, EmuTime time) override;
	[[nodiscard]] const byte* getReadCacheLine(uint16_t start) const override;
	[[nodiscard]] byte* getWriteCacheLine(uint16_t start) override;

	// SCCVoiceRomBypassReader -- see that interface's doc comment in
	// SCCVoiceCore.hh for why this exists (the fill engine reading sample
	// data that coincides with this device's own SCC(+) register
	// window).
	[[nodiscard]] uint8_t peekUnderlyingRom(uint16_t address, EmuTime time) const override;

	// I/O port fast-path for DATA/LEVEL (0x74) and
	// CTRL/STATUS (0x75) -- see class comment. Simply
	// dispatches to the same SCCVoiceCore methods the memory-mapped path
	// uses; no separate logic, just an alternate entry point that
	// doesn't require switching this device's slot into a CPU page.
	[[nodiscard]] byte readIO(uint16_t port, EmuTime time) override;
	[[nodiscard]] byte peekIO(uint16_t port, EmuTime time) const override;
	void writeIO(uint16_t port, byte value, EmuTime time) override;

	template<typename Archive>
	void serialize(Archive& ar, unsigned version);

private:
	void setMapper(int region, byte value);
	void setModeRegister(byte value);
	void checkEnable();

	// The voice register map (offsets 0xE0-0xFE, I/O ports 0x74-0x79) and its
	// decoding live in SCCVoiceUnit.

	[[nodiscard]] bool isSccVisible(uint16_t address) const;

private:
	const MapperConfig mapperConfig;
	Ram ram;
	SCC scc;
	SCCVoiceUnit voice; // voices A and B and their register map
	RomBlockDebuggable romBlockDebug;
	std::array<byte*, 4> internalMemoryBank; // 4 blocks of 8kB starting at #4000
	enum SCCEnable : uint8_t {EN_NONE, EN_SCC, EN_SCCPLUS} enable;
	byte modeRegister;
	std::array<bool, 4> isRamSegment;
	std::array<bool, 4> isMapped;
	std::array<byte, 4> mapper;
};

} // namespace openmsx

#endif
