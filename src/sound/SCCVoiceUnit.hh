#ifndef SCCVOICEUNIT_HH
#define SCCVOICEUNIT_HH

// The two sample voices of the SCCボイス (SCC-Voice), A and B, together with
// their register map (guide section 22). Shared by MSXSCCVoice (RAM cartridge)
// and RomKonamiSCCPlusVoice (ROM mapper), so the register layout and its
// read/peek/write decoding exist in exactly one place. The wrappers only decide
// WHERE the registers are visible (the SCC/SCC+ window); this class decides WHAT
// each register does.
//
// The two voices are identical: period counter, ROM fill engine with loops,
// click-free loop override. Their registers have the same layout, voice A at
// offset 0xE0 and voice B at 0xF0 (B = A + 0x10); 0xEF and 0xFF are free.
// Offsets are relative to the visible SCC(+) window, i.e. (address & 0xFF). On
// the SCC+ chip (both SCC and SCC+ mode) offsets 0xE0-0xFF have no function;
// the deformation register sits at 0xC0-0xDF. Only on the ORIGINAL SCC is
// 0xE0-0xFF the deformation register, so software written for that chip that
// writes 0x98E0 in SCC (Compatible) mode reaches ROM_FILL_LEVEL here.
// Registers marked "W only" read as 0xFF. The SCC+ mode register at
// 0xBFFE/0xBFFF wins over voice B's DATA (offset 0xFE): the cartridges handle
// it before they ask this class.

#include "SCCVoiceCore.hh"

#include <cstdint>
#include <optional>
#include <string>

namespace openmsx {

class DeviceConfig;

class SCCVoiceUnit
{
public:
	// --- one voice: offsets from its base ------------------------------
	static constexpr uint8_t REG_ROM_FILL_LEVEL    = 0x0; // R/W
	static constexpr uint8_t REG_CTRL_STATUS       = 0x1; // W=CTRL, R=STATUS
	static constexpr uint8_t REG_PITCH_L           = 0x2; // W, R=last value
	static constexpr uint8_t REG_PITCH_H           = 0x3; // W, R=last value
	static constexpr uint8_t REG_VOLUME            = 0x4; // W, R=last value
	static constexpr uint8_t REG_ROM_START_L       = 0x5; // R/W
	static constexpr uint8_t REG_ROM_START_H       = 0x6; // R/W
	static constexpr uint8_t REG_ROM_END_L         = 0x7; // W only
	static constexpr uint8_t REG_ROM_END_H         = 0x8; // W only
	static constexpr uint8_t REG_ROM_REPEAT_ADDR_L = 0x9; // W only
	static constexpr uint8_t REG_ROM_REPEAT_ADDR_H = 0xA; // W only
	static constexpr uint8_t REG_STEP_INDEX        = 0xB; // W only
	static constexpr uint8_t REG_PREDICTOR_L       = 0xC; // W only
	static constexpr uint8_t REG_PREDICTOR_H       = 0xD; // W only
	static constexpr uint8_t REG_DATA_LEVEL        = 0xE; // W=DATA, R=LEVEL
	static constexpr uint8_t REG_NUM               = 0xF; // 0xF is free

	// --- the bases ------------------------------------------------------
	static constexpr uint8_t VOICE_A_BASE = 0xE0;
	static constexpr uint8_t VOICE_B_BASE = 0xF0;

	// --- I/O port fast path --------------------------------------------
	// Absolute port numbers (like openMSX's other readIO/writeIO users).
	// Ports 0x74-0x78 act on the voice selected through VOICE_SELECT.
	static constexpr uint16_t IO_DATA_LEVEL   = 0x74; // W=DATA, R=LEVEL
	static constexpr uint16_t IO_CTRL_STATUS  = 0x75; // W=CTRL, R=STATUS
	static constexpr uint16_t IO_STEP_INDEX   = 0x76; // W only
	static constexpr uint16_t IO_PREDICTOR_L  = 0x77; // W only
	static constexpr uint16_t IO_PREDICTOR_H  = 0x78; // W only
	static constexpr uint16_t IO_VOICE_SELECT = 0x79; // R/W, bit 0: 0 = voice A, 1 = voice B
	static constexpr uint16_t IO_BASE = IO_DATA_LEVEL;
	static constexpr unsigned IO_NUM  = 6;

public:
	/** 'name' is the owning device's name; the two sound devices are called
	  * "<name> SCC Voice A" and "<name> SCC Voice B", so they sit next to
	  * "<name> SCC" in the mixer. The fill engines read through 'romReader'. */
	SCCVoiceUnit(const std::string& name, const DeviceConfig& config,
	             EmuTime time, SCCVoiceRomBypassReader& romReader);

	void reset(EmuTime time);

	/** Memory-mapped registers. The result of read/peek is empty when 'offset'
	  * is not a voice register (the caller then passes the access on to the
	  * SCC); writeRegister() returns false in that case. */
	[[nodiscard]] std::optional<uint8_t> readRegister(uint8_t offset, EmuTime time);
	[[nodiscard]] std::optional<uint8_t> peekRegister(uint8_t offset, EmuTime time) const;
	[[nodiscard]] bool writeRegister(uint8_t offset, uint8_t value, EmuTime time);

	/** I/O port fast path. For IN A,(n) / OUT (n),A real Z80 hardware also
	  * puts register A on the upper address lines; nothing decodes that, so
	  * only the low byte of 'port' is used. */
	[[nodiscard]] uint8_t readPort(uint16_t port, EmuTime time);
	[[nodiscard]] uint8_t peekPort(uint16_t port, EmuTime time) const;
	void writePort(uint16_t port, uint8_t value, EmuTime time);

	template<typename Archive>
	void serialize(Archive& ar, unsigned version);

private:
	/** offset -> its voice, or nullptr when it is no voice register */
	[[nodiscard]] SCCVoiceCore* voiceAt(uint8_t offset);
	[[nodiscard]] const SCCVoiceCore* voiceAt(uint8_t offset) const;
	[[nodiscard]] static std::optional<uint8_t> read(SCCVoiceCore& v, uint8_t reg, EmuTime time);
	[[nodiscard]] static std::optional<uint8_t> peek(const SCCVoiceCore& v, uint8_t reg, EmuTime time);
	static void write(SCCVoiceCore& v, uint8_t reg, uint8_t value, EmuTime time);
	[[nodiscard]] SCCVoiceCore& selected() { return voiceSelect ? voiceB : voiceA; }
	[[nodiscard]] const SCCVoiceCore& selected() const { return voiceSelect ? voiceB : voiceA; }

	SCCVoiceCore voiceA;
	SCCVoiceCore voiceB;
	uint8_t voiceSelect = 0; // VOICE_SELECT bit 0
};

} // namespace openmsx

#endif
