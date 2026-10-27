// Emulation of one sample voice of the SCCボイス (SCC-Voice) cartridge, built
// around an OKI MSM5205-compatible 4-bit ADPCM decoder. The cartridge has two
// identical voices, A and B (see SCCVoiceUnit). See the SCCVoice programming
// guide (section 22) for the full functional description.
//
// Clock: a 12-bit period counter on the 3.58 MHz slot clock, exactly like an
// SCC channel. It counts PITCH + 1 slot clocks per sample tick:
//
//   sample rate = 3579545 / (PITCH + 1) Hz
//
// so a 32-sample cycle on a voice sounds at the same pitch as an SCC channel
// with the same period. There are no separate rate modes: a fixed-rate
// one-shot simply gets a fixed period (446 = 8 kHz, also the reset value).
// A new period takes effect at the next reload of the counter. On a FIFO
// underrun the voice stops; CTRL bit 4 (HOLD) chooses what the output does
// then: 0 (reset) drops to silence, so a one-shot ends by itself, 1 holds the
// last value, so a melodic sample that briefly runs dry does not click. An
// explicit STOP always drops the output to silence.
//
// Three DATA-SOURCE modes (CTRL bit5 = RAW8_MODE, bit7 = RAW4_MODE):
//   - ADPCM decode (default): standard OKI 4-bit ADPCM, 2 nibbles/byte,
//     same algorithm as e.g. MSM6258/MSM5205 and generic OKI-ADPCM.
//   - Raw 8-bit signed PCM bypass: the OKI decoder is skipped entirely;
//     each FIFO byte is sign-extended and used directly as a PCM sample
//     (scaled into the same -2048..2047 predictor range). Higher
//     fidelity (~40dB+ SNR) at 2x the FIFO byte cost per unit of audio
//     time versus ADPCM (1 byte/sample instead of 0.5).
//   - Raw 4-bit signed PCM (RAW4_MODE): same nibble sequencing as ADPCM
//     (2 samples per FIFO byte, HIGH nibble first), but the OKI decoder is
//     bypassed: each nibble is a two's-complement value -8..7 (NOT the
//     sign-magnitude nibble of ADPCM), reconstructed at the middle of its
//     quantisation step as v*256+128 (-1920..1920, symmetric, no DC
//     offset). No decoder state, so no loop-wrap override is needed.
//     Intended for single-cycle wavetable-style waveforms; for ordinary
//     (non-periodic) samples ADPCM is far better. If RAW8_MODE and
//     RAW4_MODE are both set, RAW8 wins.
//
// In all modes: a 255-byte FIFO, fed either by the CPU one byte at a time
// (DATA) or by the ROM fill engine (ROM_xxx registers, see below), and a
// hardware auto-stop on FIFO underrun while active. There is no ADSR in
// hardware; looping is done by the fill engine (or by the driver itself on
// the manual path).
//
// The class owns its own Mixer channel and applies a 2nd-order
// Sallen-Key-style low-pass filter (fc ~= 3.5kHz, Q ~= 0.707) to ONLY this
// channel's output, modelling the dedicated analog filter that sits after the
// voice DAC and before the voice/SCC mixer.

#ifndef SCCVOICECORE_HH
#define SCCVOICECORE_HH

#include "SoundDevice.hh"
#include "Schedulable.hh"
#include "BlipBuffer.hh"
#include "DynamicClock.hh"

#include <array>
#include <cstdint>
#include <string>

namespace openmsx {

class DeviceConfig;

/** The interface through which the fill engine reads sample bytes: the
  * cartridge's own ROM (or RAM) as its mapper currently banks it in,
  * bypassing the SCC(+) register window that may be visible at that address
  * (the window "steals" 2KB of the page it is active on, which would
  * otherwise make that sample data unreadable). It is mandatory: the engine
  * belongs to the cartridge and never reads through the CPU's slot
  * selection, which could see another slot. */
class SCCVoiceRomBypassReader
{
public:
	virtual ~SCCVoiceRomBypassReader() = default;
	[[nodiscard]] virtual uint8_t peekUnderlyingRom(uint16_t address, EmuTime time) const = 0;
};

class SCCVoiceCore final : public SoundDevice, public Schedulable
{
public:
	SCCVoiceCore(const std::string& name_, const DeviceConfig& config, EmuTime time,
	             SCCVoiceRomBypassReader& romReader);
	~SCCVoiceCore();

	/** Full hardware reset (power-up / cartridge reset). */
	void reset(EmuTime time);

	// Register-level interface, called by SCCVoiceUnit after it has decoded
	// the register address. All of these are cycle-accurate: they first
	// replay any pending FIFO/decoder ticks up to 'time' before reading or
	// modifying state.

	/** DATA (write-only): push one byte (2 nibbles) into the FIFO. If the
	  * FIFO is already full, the byte is silently dropped (this specific
	  * behaviour is unspecified; dropping is the safest assumption for a
	  * hardware FIFO without handshake). */
	void writeData(uint8_t value, EmuTime time);

	/** CTRL (write-only): bit0=START, bit1=STOP, bits2-3 reserved (ignored),
	  * bit4=HOLD (what the output does on an underrun: 0=silence, 1=hold the
	  * last value), bit5=RAW8_MODE (0=ADPCM decode, 1=raw 8-bit signed PCM bypass -- see
	  * class comment above), bit6=FLUSH (clears the FIFO only -- does not
	  * touch decoder state or ACTIVE; if the voice is actively playing and
	  * the flush empties the FIFO, this triggers an immediate underrun, same
	  * as if the FIFO had drained naturally), bit7=RAW4_MODE (1=raw 4-bit
	  * signed PCM, two samples per byte, high nibble first; RAW8 wins if
	  * both are set). Applied in the order FLUSH, then STOP, then START, so a
	  * single write can flush-and-restart in one go. */
	void writeControl(uint8_t value, EmuTime time);

	/** PITCH_L / PITCH_H (write-only): the 12-bit period (bits 7:0 / bits
	  * 3:0 respectively, upper nibble of the high byte is reserved), same
	  * scale as an SCC channel: rate = 3579545 / (period + 1). Takes effect
	  * at the next reload of the period counter, as on an SCC channel. */
	void writePitchLow(uint8_t value, EmuTime time);
	void writePitchHigh(uint8_t value, EmuTime time);

	/** VOLUME (write-only): bits 0-3 = 4-bit volume, 0=mute,15=full.
	  * Applied to the signal before the mixer, also to the value currently
	  * held on the output (volume cut-off). */
	void writeVolume(uint8_t value, EmuTime time);

	/** Read-back of the last-written PITCH_L / PITCH_H / VOLUME values.
	  * These are pure state queries, so no EmuTime/sync() is needed. */
	[[nodiscard]] uint8_t getPitchLow() const;
	[[nodiscard]] uint8_t getPitchHigh() const;
	[[nodiscard]] uint8_t getVolume() const;

	/** STEP_INDEX (write-only, optional): stages a step_index override
	  * value (clamped 0..48) to be applied together with the next
	  * PREDICTOR_H write. Must be written BEFORE PREDICTOR_H to take effect
	  * -- PREDICTOR_H is what actually arms/commits the combined override
	  * (see below). If not used, step_index is left untouched by the
	  * predictor override. */
	void writeStepIndexOverride(uint8_t value, EmuTime time);

	/** PREDICTOR_L / PREDICTOR_H (write-only): stages a 12-bit signed
	  * predictor override value (bits 7:0 / bits 3:0 respectively). Writing
	  * PREDICTOR_H commits the override and ARMS a mark: the NEXT byte
	  * pushed into the FIFO (by DATA or by the fill engine) carries it. When
	  * the decoder reaches that byte it loads the predictor (and, if
	  * STEP_INDEX was written since the last arm, the step_index) from the
	  * override values, right before decoding the byte's nibbles --
	  * regardless of how much unrelated data is queued ahead of it. The
	  * marks travel with the FIFO entries, so any number of them can be
	  * queued at once.
	  *
	  * Intended usage, for click-free ADPCM loop transitions (the decoder's
	  * predictor/step_index state must match at both sides of a loop seam):
	  * write the last byte of the pre-loop data, then (optionally)
	  * STEP_INDEX, then PREDICTOR_L/H, then IMMEDIATELY write the loop-start
	  * byte via DATA with nothing else in between -- the mark attaches to
	  * the very next byte pushed, so the loop-start byte must be that byte.
	  * The override VALUES are read when the marked byte is decoded: do not
	  * change them while marked bytes may still be queued. The ROM fill
	  * engine marks the first byte of every loop pass by itself. */
	void writePredictorLow(uint8_t value, EmuTime time);
	void writePredictorHigh(uint8_t value, EmuTime time);

	/** LEVEL (read-only): current FIFO fill level, 0..255 (FIFO_CAPACITY). */
	[[nodiscard]] uint8_t readLevel(EmuTime time);
	[[nodiscard]] uint8_t peekLevel(EmuTime time) const;

	/** STATUS (read-only): bit0=ACTIVE, bit1=UNDERRUN, bit2=FIFO_FULL,
	  * bit3=FIFO_EMPTY, bit4=HOLD readback, bit5=RAW8_MODE readback,
	  * bit6=FILL_BUSY (always 0 here: the fill completes inside the
	  * register write), bit7=RAW4_MODE readback. */
	[[nodiscard]] uint8_t readStatus(EmuTime time);
	[[nodiscard]] uint8_t peekStatus(EmuTime time) const;

	// --- Hardware-assisted buffer fill (ROM_xxx registers) -------------
	// An alternative to feeding the FIFO one byte at a time via
	// writeData(): the driver points this engine at a 16-bit address in the
	// cartridge's own ROM as its mapper banks it in (read through the
	// SCCVoiceRomBypassReader) and asks it to fill the FIFO up to a target
	// level in one bounded, synchronous operation.
	//
	// A run is described by three addresses: START (current read position),
	// END (exclusive end of the data) and REPEAT_ADDR (loop start;
	// REPEAT_ADDR == END means no loop). Before fetching each byte the engine
	// compares START with END; when they are equal it loads START from
	// REPEAT_ADDR. If START still equals END after that, there is no loop and
	// the engine stops supplying data; otherwise it continues from the loop
	// start and marks that first byte of the new pass for the click-free
	// override (see tick()). No address is reserved as a "no loop" value, so
	// a loop may start anywhere, page 0 included. Because the comparison is
	// made BEFORE a fetch and not right after the increment, a fill that
	// ends exactly at END leaves the engine waiting there; rewriting START
	// and END at that point (after a bank switch) simply continues the run
	// -- this is what makes stitching a sample across mapper banks possible.
	// The loop always ends at END: the loop region is [REPEAT_ADDR, END).
	// Coexists with, does not replace, the plain writeData()-driven path.

	/** ROM_START_ADDR_L / ROM_START_ADDR_H: the fill engine's current read
	  * address (a plain 16-bit address in the cartridge's window). Write:
	  * sets/re-points the read address without touching END, REPEAT_ADDR or
	  * the decoder state. Read: returns the engine's current position (END
	  * minus this is the number of bytes left). */
	void writeRomStartAddrLow(uint8_t value, EmuTime time);
	void writeRomStartAddrHigh(uint8_t value, EmuTime time);
	[[nodiscard]] uint8_t readRomStartAddrLow(EmuTime time);
	[[nodiscard]] uint8_t readRomStartAddrHigh(EmuTime time);
	[[nodiscard]] uint8_t peekRomStartAddrLow(EmuTime time) const;
	[[nodiscard]] uint8_t peekRomStartAddrHigh(EmuTime time) const;

	/** ROM_END_ADDR_L / ROM_END_ADDR_H (write-only): exclusive end address
	  * of the data (the address just after the last byte). For a sample
	  * stitched across banks the driver writes the end of the mapped window
	  * for every segment except the last one (and REPEAT_ADDR = that same
	  * END, so the segment does not loop). */
	void writeRomEndAddrLow(uint8_t value, EmuTime time);
	void writeRomEndAddrHigh(uint8_t value, EmuTime time);

	/** ROM_REPEAT_ADDR_L / ROM_REPEAT_ADDR_H (write-only): address to resume
	  * at when START reaches END. REPEAT_ADDR == END means no loop (an empty
	  * loop region), so every address, page 0 included, can be a loop start.
	  * Write it on every trigger: a value left over from the previous sample
	  * would make a one-shot loop. */
	void writeRomRepeatAddrLow(uint8_t value, EmuTime time);
	void writeRomRepeatAddrHigh(uint8_t value, EmuTime time);

	/** ROM_FILL_LEVEL: write = fill the FIFO up to this target level (the
	  * engine reads its own current level internally and fills the
	  * difference -- the driver only ever states a target, never a byte
	  * count). Modelled as completing synchronously within this call, so
	  * FILL_BUSY always reads 0 and an immediately-following read of this
	  * register already gives the result.
	  * Read: bytes still needed to reach the most recently requested target;
	  * 0 = the target was reached. Non-zero after a write means the engine
	  * stopped at END (no loop, or a stitched segment that ended) before
	  * reaching the target. */
	void writeRomFillLevel(uint8_t value, EmuTime time);
	[[nodiscard]] uint8_t readRomFillLevel(EmuTime time) const;
	[[nodiscard]] uint8_t peekRomFillLevel(EmuTime time) const;

	template<typename Archive>
	void serialize(Archive& ar, unsigned version);

private:
	// SoundDevice
	void setOutputRate(unsigned hostSampleRate, double speed) override;
	void generateChannels(std::span<float*> bufs, unsigned num) override;
	[[nodiscard]] bool updateBuffer(size_t length, float* buffer, EmuTime time) override;
	[[nodiscard]] float getAmplificationFactorImpl() const override;

	// Schedulable
	void executeUntil(EmuTime time) override;

	// internal helpers
	void sync(EmuTime time);
	void schedule(EmuTime time);
	void tick(EmuTime time);
	[[nodiscard]] int decodeNibble(uint8_t nibble);
	/** Raw 4-bit PCM nibble -> 12-bit sample (two's complement -8..7,
	  * reconstructed at the middle of the step: v*256+128). */
	[[nodiscard]] static int rawNibbleToSample(uint8_t nibble);
	void doStart(EmuTime time);
	void doStop(EmuTime time);
	void doFlush(EmuTime time);
	void forceSilence(EmuTime time);
	void triggerUnderrun(EmuTime time);
	void pushByte(uint8_t value);
	void computeFilterCoefficients(float outputRate);

private:
	// 256-entry ring buffer with one entry always left free (the classic
	// "full when head+1 == tail" ring): capacity 255, the level always fits the
	// 8-bit LEVEL / ROM_FILL_LEVEL registers and an 8-bit pointer pair is
	// enough in hardware (level = head - tail, no separate wrap bit).
	static constexpr unsigned FIFO_SIZE = 256;
	static constexpr unsigned FIFO_CAPACITY = FIFO_SIZE - 1;

	// Output level against one SCC channel (guide section 20.8): a full-scale
	// voice at volume 15 mixes in twice as loud as an SCC channel at volume 15.
	// Two sample voices otherwise drown under five SCC channels and the PSG.
	static constexpr float OUTPUT_GAIN = 2.0f;

	// Period counter (guide section 22.3): the MSX slot clock, and the period
	// after reset (8008 Hz), so a driver that writes no period hears 8 kHz.
	static constexpr unsigned SLOT_CLOCK_HZ = 3579545;
	static constexpr uint16_t RESET_PERIOD = 446;

	// Hardware FIFO. Granularity of consumption depends on raw8Mode:
	// 2 nibbles/byte for ADPCM decode and for raw 4-bit PCM, 1 byte/sample
	// for raw 8-bit PCM bypass.
	std::array<uint8_t, FIFO_SIZE> fifo = {};
	// Loop-override marker per FIFO entry (bit0: apply the predictor
	// override when this byte is decoded, bit1: also the step index).
	std::array<uint8_t, FIFO_SIZE> fifoMark = {};
	bool markNext = false;     // the next byte pushed carries bit0
	bool markNextStep = false; // ... and bit1
	unsigned fifoHead = 0; // next write position
	unsigned fifoTail = 0; // next read position
	unsigned fifoCount = 0; // number of bytes currently queued
	uint8_t currentByte = 0; // byte currently being split into nibbles (ADPCM mode only)
	bool highNibbleNext = true; // ADPCM mode only

	// The slot clock (3.58 MHz). 'countdown' is the number of slot clocks
	// until the next sample tick (1 .. 4096); at that tick it reloads with
	// pitch + 1. Drives both the FIFO/decoder state (for cycle-accurate
	// register reads) and, via BlipBuffer, the generated audio.
	DynamicClock clock;
	unsigned countdown = RESET_PERIOD + 1;
	BlipBuffer blip;
	float lastOutput = 0.0f; // last value pushed into 'blip', for delta calc
	int lastSample = 0;      // last decoded sample BEFORE volume (-2048..2047), so a
	                         // volume write can rescale the output immediately

	// OKI ADPCM decoder state
	int predictor = 0;     // 12-bit signed, -2048..2047
	uint8_t stepIndex = 0; // 0..48

	// Predictor/step_index override, for click-free ADPCM loop
	// transitions (see writePredictorHigh() doc comment). The *values* are
	// single registers, but the *events* travel with the FIFO bytes
	// (fifoMark): any number of loop wraps can be queued in the buffer at
	// once, so a loop shorter than the buffer stays click-free. The values
	// are read when the marked byte is DECODED, so do not change them while
	// marked bytes are still queued.
	uint16_t predictorOverrideStaging = 0; // 12-bit value being built up
	                                        // by PREDICTOR_L/H writes
	int overridePredictorValue = 0;   // sign-extended, ready to apply
	uint8_t overrideStepIndexValue = 0;
	bool overrideStepIndexPending = false; // STEP_INDEX written, not yet armed

	uint16_t pitch = RESET_PERIOD; // 12-bit PITCH register (period)

	// Sallen-Key low-pass filter (2nd order, fc~=3.5kHz, Q~=0.707),
	// applied at the *output* sample rate (i.e. modelling a fixed analog
	// filter after the DAC, independent of the playback rate).
	// Direct-form-I biquad.
	float lpB0 = 1.0f, lpB1 = 0.0f, lpB2 = 0.0f, lpA1 = 0.0f, lpA2 = 0.0f;
	float lpX1 = 0.0f, lpX2 = 0.0f, lpY1 = 0.0f, lpY2 = 0.0f;

	uint8_t volume = 15;    // 4-bit; default full volume
	bool active = false;
	bool underrun = false;
	bool raw8Mode = false;  // false=ADPCM decode, true=raw 8-bit signed PCM bypass
	bool raw4Mode = false;  // true=raw 4-bit signed PCM (ignored if raw8Mode)
	bool hold = false;      // CTRL bit 4: on an underrun hold the last value (true) or go silent

	// --- Hardware-assisted buffer fill (ROM_xxx registers) state -------
	SCCVoiceRomBypassReader& romReader; // the cartridge's own ROM, see the class comment

	uint16_t romStartAddr = 0;    // current read position (R/W)
	uint16_t romEndAddr = 0;      // exclusive end of the data
	uint16_t romRepeatAddr = 0;   // loop start; == romEndAddr -> no loop
	uint8_t romFillRemaining = 0; // bytes still needed from the most
	                               // recent ROM_FILL_LEVEL request
};

} // namespace openmsx

#endif
