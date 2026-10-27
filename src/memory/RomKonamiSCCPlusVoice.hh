// Konami SCC/SCC+ dual-mode ROM mapper, extended with the SCCボイス sample
// voices. This is the only ROM mapper variant of the SCCVoice: it serves both
// the plain SCC and the SCC+ register layout, selected by the mode register.
//
// Combines two things:
//   - the Rom8kBBlocks-based Konami bank-switching of RomKonamiSCC, with the
//     voice register decode of MSXSCCVoice (same register map: voice A at
//     0xE0-0xEE, voice B at 0xF0-0xFE; see SCCVoiceUnit.hh).
//   - MSXSCCVoice's dual SCC/SCC+ mode-register + enable logic (mode
//     register at 0xBFFE/0xBFFF, EN_SCC/EN_SCCPLUS derived from the
//     mode register bit5 plus the raw bank-select value written to
//     page 5 (0xA000-0xBFFF)'s trigger address), just backed by ROM
//     data instead of MSXSCCVoice's onboard RAM.
//
// Bank selection note: only bits 6-0 of a written bank value select the
// bank (banks 0-127); bit 7 is NOT part of the bank number. On page 5 it
// remains the SCC+ enable flag, so a driver writes 0x80|bank to keep SCC+
// enabled while mapping ANY bank 0-127 there. (This differs from
// MSXSCCPlusCart/MSXSCCVoice and from openMSX's own Yamanooto model, which
// use the full 8-bit value as the bank number; this class deliberately
// ignores bit 7 so that the whole 0x8000-0xBFFF range can hold sample data.
// Banks above 127 would need a separate offset register, not modelled.)
//
// Optional sample RAM (ROM type "SCCPlusVoiceRAM", guide section 23): 64kB of RAM next to the ROM, for
// composers who load a .PAK from disk. Mode register bit 6 (RAM_WINDOW, unused on the SCC-I) switches pages
// 4+5 (0x8000-0xBFFF, the sample window) from ROM to RAM bank (bank value & 7); pages 2+3 stay ROM. Writing
// follows the SCC-I rules for the pages that show RAM: page 4 with mode bit 2 in SCC+ mode or bit 4, page 5
// with bit 4; such a write goes to RAM, also on the bank and SCC register addresses. With bit 6 clear (and on
// the plain "SCCPlusVoice" type, which has no RAM) the mapper is exactly the ROM mapper described above. The
// fill engine reads through the same view, so it plays from RAM while bit 6 is set.
//
// I/O port fast path (0x74-0x79, VOICE_SELECT on 0x79; same ports/semantics as
// MSXSCCVoice -- see MSXSCCVoice.hh for the full rationale and
// SCCVoiceUnit.hh for the register mapping): unlike MSXSCCVoice, a ROM cart loaded via `cart file.rom
// -romtype X` gets its DeviceConfig synthesized in C++ by
// HardwareConfig::createRomConfig(), which only ever emits a hardcoded
// <mem> element -- there's no <io> tag support in that path, so the
// normal XML-driven MSXDevice::registerPorts() mechanism has nothing to
// register. Instead, these ports are registered MANUALLY, directly with
// the CPU interface, in the constructor/destructor (see .cc file) --
// bypassing the XML config path entirely.

#ifndef ROMKONAMISCCPLUSVOICE_HH
#define ROMKONAMISCCPLUSVOICE_HH

#include "RomBlocks.hh"
#include "Ram.hh"
#include "SCC.hh"
#include "SCCVoiceCore.hh"
#include "SCCVoiceUnit.hh"

#include <array>
#include <optional>

namespace openmsx {

class RomKonamiSCCPlusVoice final : public Rom8kBBlocks, public SCCVoiceRomBypassReader
{
public:
	static constexpr unsigned RAM_SIZE = 64 * 1024;    // the optional sample RAM (SCCPlusVoiceRAM)

	/** ramSize: 0 (ROM only) or RAM_SIZE */
	RomKonamiSCCPlusVoice(const DeviceConfig& config, Rom&& rom, unsigned ramSize = 0);
	~RomKonamiSCCPlusVoice() override;

	void powerUp(EmuTime time) override;
	void reset(EmuTime time) override;
	[[nodiscard]] byte peekMem(uint16_t address, EmuTime time) const override;
	[[nodiscard]] byte readMem(uint16_t address, EmuTime time) override;
	[[nodiscard]] const byte* getReadCacheLine(uint16_t address) const override;
	void writeMem(uint16_t address, byte value, EmuTime time) override;
	[[nodiscard]] byte* getWriteCacheLine(uint16_t address) override;

	// SCCVoiceRomBypassReader -- see that interface's doc comment in
	// SCCVoiceCore.hh. Here the underlying storage is the cartridge's own ROM,
	// as currently banked in by the Konami mapper; the fill engine reads
	// it directly, so sample data that happens to sit underneath the
	// SCC(+) register window (0x9800-0x9FFF / 0xB800-0xBFFF) stays
	// readable while SCC is enabled.
	[[nodiscard]] uint8_t peekUnderlyingRom(uint16_t address, EmuTime time) const override;

	// I/O port fast path -- see class comment above. Manually registered/
	// unregistered in the constructor/destructor, not XML-driven.
	[[nodiscard]] byte readIO(uint16_t port, EmuTime time) override;
	[[nodiscard]] byte peekIO(uint16_t port, EmuTime time) const override;
	void writeIO(uint16_t port, byte value, EmuTime time) override;

	template<typename Archive>
	void serialize(Archive& ar, unsigned version);

private:
	void bankSwitch(unsigned page, unsigned block);
	[[nodiscard]] bool ramVisible(uint16_t address) const;
	[[nodiscard]] bool ramWritable(uint16_t address) const;
	[[nodiscard]] size_t ramOffset(uint16_t address) const;
	void setModeRegister(byte value);
	void checkEnable();
	[[nodiscard]] bool isSccVisible(uint16_t address) const;

	// The voice register map (offsets 0xE0-0xFE, I/O ports 0x74-0x79) and its
	// decoding live in SCCVoiceUnit.

private:
	SCC scc;
	SCCVoiceUnit voice; // voices A and B and their register map

	// Raw bank-select values last written for pages 2-5 (0x4000-0xBFFF),
	// indexed as [page - 2] -- needed separately from whatever
	// Rom8kBBlocks does internally with setRom(), since checkEnable()
	// needs the *raw* written byte (including bit7) for page 5, not the
	// masked/wrapped block index Rom8kBBlocks actually maps to.
	std::array<byte, 4> bankValue = {};

	byte modeRegister = 0;
	std::optional<Ram> ram; // the optional sample RAM, banks 0-7 in pages 4+5 while RAM_WINDOW is set

	// mode register bits (0xBFFE/0xBFFF)
	static constexpr byte MODE_PAGE4_WRITE = 0x04; // page 4 RAM writable (with SCC+ mode)
	static constexpr byte MODE_ALL_WRITE   = 0x10; // RAM writable in all pages that show it
	static constexpr byte MODE_SCC_PLUS    = 0x20; // SCC+ register layout
	static constexpr byte MODE_RAM_WINDOW  = 0x40; // pages 4+5 show RAM (SCCVoice, not on the SCC-I)
	static constexpr unsigned RAM_BANK_MASK = 0x07;

	enum SCCEnable : uint8_t {EN_NONE, EN_SCC, EN_SCCPLUS} enable = EN_NONE;
};

} // namespace openmsx

#endif
