#include "SCCVoiceCore.hh"

#include "DeviceConfig.hh"
#include "MSXMotherBoard.hh"
#include "narrow.hh"
#include "serialize.hh"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace openmsx {

// Dummy value passed to the SoundDevice base constructor; the real
// (host) output rate is only known once setOutputRate() is called, see
// DACSound16S for the same pattern.
static constexpr unsigned DUMMY_INPUT_RATE = 44100;

// Standard OKI ADPCM step-size and index-adjust tables (identical to the
// ones used by the reference Z80 encoder; these are
// the fixed OKI MSM5205/MSM6258-family hardware tables, not copyrightable
// creative content).
static constexpr std::array<int, 49> okiStepTable = {
	   16,   17,   19,   21,   23,   25,   28,   31,   34,   37,
	   41,   45,   50,   55,   60,   66,   73,   80,   88,   97,
	  107,  118,  130,  143,  157,  173,  190,  209,  230,  253,
	  279,  307,  337,  371,  408,  449,  494,  544,  598,  658,
	  724,  796,  876,  963, 1060, 1166, 1282, 1411, 1552,
};
static constexpr std::array<int, 8> okiIndexAdjustTable = {
	-1, -1, -1, -1, 2, 4, 6, 8,
};

// The period counter (guide section 22.3) ticks a sample every PITCH + 1 slot
// clocks, exactly like the wavetable pointer of an SCC channel:
//
//   sample rate = 3579545 / (period + 1)  Hz
//
// (the well-known SCC note-frequency formula divides this by 32 again: one
// wavetable cycle is 32 steps), so a 32-sample cycle on a voice sounds at
// the same pitch as an SCC channel with the same period. A cycle of N
// samples sounds 32/N times that, a fixed transposition the toolchain
// corrects with the sample's NOTE_OFFSET.


SCCVoiceCore::SCCVoiceCore(const std::string& name_, const DeviceConfig& config, EmuTime time,
                           SCCVoiceRomBypassReader& romReader_)
	: SoundDevice(config.getMotherBoard().getMSXMixer(), name_,
	              "SCCVoice sample voice", 1, DUMMY_INPUT_RATE, false)
	, Schedulable(config.getScheduler())
	, clock(time)
	, romReader(romReader_)
{
	clock.setFreq(SLOT_CLOCK_HZ);
	reset(time);
	registerSound(config);
}

SCCVoiceCore::~SCCVoiceCore()
{
	unregisterSound();
}

void SCCVoiceCore::reset(EmuTime time)
{
	removeSyncPoint();

	fifoHead = fifoTail = fifoCount = 0;
	currentByte = 0;
	highNibbleNext = true;

	predictor = 0;
	stepIndex = 0;
	markNext = markNextStep = false;
	overrideStepIndexPending = false;
	predictorOverrideStaging = 0;

	active = false;
	underrun = false;
	volume = 15;
	raw8Mode = false;  // ADPCM decode after reset
	raw4Mode = false;
	hold = false;      // silence on an underrun after reset
	pitch = RESET_PERIOD; // 8 kHz after reset
	countdown = pitch + 1u;

	// Hardware-assisted fill engine (ROM_xxx registers): clear to a
	// known, inert state. START == END == REPEAT_ADDR means "nothing to
	// supply, no loop" -- a fill requested before the driver has
	// configured a run does nothing, rather than reading stale data from
	// a previous run.
	romStartAddr = 0;
	romEndAddr = 0;
	romRepeatAddr = 0;
	romFillRemaining = 0;

	clock.reset(time);
	lastOutput = 0.0f;
	lastSample = 0;
}


// ---- register interface -----------------------------------------------

void SCCVoiceCore::writeData(uint8_t value, EmuTime time)
{
	// IMPORTANT ORDER: sync() first (pushes this device's own pending
	// tick()-generated deltas into blip, up to 'time'), THEN
	// updateStream() (has the mixer pull that freshly-available content
	// out to the actual host audio output). Doing it the other way
	// around pulls stale/old content, since updateStream()'s
	// generateChannels()->blip.readSamples() can only return whatever
	// has already been pushed into blip -- reversing this order was an
	// earlier mistake that produced the exact same one-interrupt delay
	// via a different mechanism.
	sync(time);
	if (fifoCount < FIFO_CAPACITY) {
		pushByte(value);
	}
	// else: FIFO full, byte is dropped. Not specified by the spec; this
	// mirrors typical hardware-FIFO-without-handshake behaviour.

	if (active) schedule(time);
	updateStream(time);
}

void SCCVoiceCore::writeControl(uint8_t value, EmuTime time)
{
	sync(time);

	// Bits 2-3 (the former RATE) are reserved and ignored: the rate is always
	// the period counter's (guide section 22.1). Bit 4 (the former
	// PITCH_MODE) is HOLD: what the output does on an underrun -- the old
	// Pitch Mode held, the old Legacy Mode went silent, so old drivers keep
	// their behaviour. raw8Mode/raw4Mode change how FIFO bytes are
	// interpreted (see class comment).
	hold = (value & 0x10) != 0;
	raw8Mode = (value & 0x20) != 0;
	raw4Mode = (value & 0x80) != 0; // RAW8 takes priority if both are set

	// FLUSH is applied before STOP/START, so a single write can
	// flush-and-restart in one go (FLUSH clears the FIFO; a subsequent
	// START in the same byte then finds a clean, empty FIFO to play
	// from once refilled).
	if (value & 0x40) { // FLUSH
		doFlush(time);
	}

	// If both STOP and START are written in the same byte, START wins
	// (i.e. this acts as a combined "restart"): we apply STOP first,
	// then let START re-activate. This choice is unspecified.
	if (value & 0x02) { // STOP
		doStop(time);
	}
	if (value & 0x01) { // START
		doStart(time);
	}

	if (active) {
		schedule(time);
	} else {
		removeSyncPoint();
	}
	updateStream(time); // see writeData() for why this must come after
	                    // sync(), not before
}

void SCCVoiceCore::writePitchLow(uint8_t value, EmuTime time)
{
	sync(time);
	// The counter picks the new period up at its next reload, as an SCC
	// channel does, so the pending sync point (the current countdown) stays
	// valid. The register is not double-buffered: a brief intermediate
	// value between the LOW and HIGH write is expected hardware behaviour.
	pitch = narrow_cast<uint16_t>((pitch & 0x0F00) | value);
	updateStream(time); // see writeData() for why this must come after sync()
}

void SCCVoiceCore::writePitchHigh(uint8_t value, EmuTime time)
{
	sync(time);
	pitch = narrow_cast<uint16_t>((pitch & 0x00FF) | (unsigned(value & 0x0F) << 8));
	updateStream(time); // see writeData() for why this must come after sync()
}

void SCCVoiceCore::writeVolume(uint8_t value, EmuTime time)
{
	sync(time); // keep state ordering consistent, even though volume
	            // itself doesn't affect FIFO/decoder timing
	volume = value & 0x0F;
	// Volume cut-off: apply the new volume to the sample that is currently
	// held on the output RIGHT NOW, instead of waiting for the next decoded
	// sample. Writing 0 therefore silences the voice immediately, also when
	// the output is being held (after an underrun) or the sample
	// rate is low. (forceSilence() has already zeroed lastSample for a
	// stopped voice, so this is a no-op there.)
	{
		float out = float(lastSample) * (float(volume) / 15.0f);
		BlipBuffer::TimeIndex t;
		getHostSampleClock().getTicksTill(time, t);
		blip.addDelta(t, out - lastOutput);
		lastOutput = out;
	}
	updateStream(time); // see writeData() -- especially important here,
	                    // since a volume change directly affects audible
	                    // output; without this, a volume change wouldn't
	                    // take audible effect until something else
	                    // happened to flush the mixer
}

uint8_t SCCVoiceCore::getPitchLow() const
{
	return narrow_cast<uint8_t>(pitch & 0xFF);
}

uint8_t SCCVoiceCore::getPitchHigh() const
{
	return narrow_cast<uint8_t>((pitch >> 8) & 0x0F);
}

uint8_t SCCVoiceCore::getVolume() const
{
	return volume;
}

void SCCVoiceCore::writeStepIndexOverride(uint8_t value, EmuTime time)
{
	sync(time);
	overrideStepIndexValue = narrow_cast<uint8_t>(std::clamp(int(value), 0, 48));
	overrideStepIndexPending = true;
	// Does NOT arm/capture a FIFO position by itself -- must be written
	// before PREDICTOR_H, which performs the actual arm/commit
	// (see its doc comment). If PREDICTOR_H is never subsequently
	// written, this staged value is simply never applied.
	updateStream(time); // see writeData() for why this must come after sync()
}

void SCCVoiceCore::writePredictorLow(uint8_t value, EmuTime time)
{
	sync(time);
	predictorOverrideStaging = narrow_cast<uint16_t>((predictorOverrideStaging & 0x0F00) | value);
	updateStream(time); // see writeData() for why this must come after sync()
}

void SCCVoiceCore::writePredictorHigh(uint8_t value, EmuTime time)
{
	sync(time);
	predictorOverrideStaging = narrow_cast<uint16_t>(
		(predictorOverrideStaging & 0x00FF) | (unsigned(value & 0x0F) << 8));

	// Sign-extend the 12-bit staged value to a normal signed int.
	int signed12 = int(predictorOverrideStaging);
	if (signed12 & 0x800) signed12 -= 0x1000;
	overridePredictorValue = signed12;

	// Arm: the NEXT byte pushed into the FIFO (by DATA or by the fill
	// engine) carries a mark. When the decoder reaches that byte it loads
	// predictor (and, if STEP_INDEX was written since the last arm,
	// step index) from the override values, so this byte's deltas build on
	// the new baseline. Marks are per FIFO entry: several can be queued at
	// once. Write the triplet immediately before pushing the loop-start
	// byte, and do not rewrite it while marked bytes are still queued.
	markNext = true;
	markNextStep = overrideStepIndexPending;
	overrideStepIndexPending = false;
	updateStream(time); // see writeData() for why this must come after sync()
}

uint8_t SCCVoiceCore::readLevel(EmuTime time)
{
	sync(time);
	return narrow_cast<uint8_t>(fifoCount);
}

uint8_t SCCVoiceCore::peekLevel(EmuTime time) const
{
	const_cast<SCCVoiceCore*>(this)->sync(time);
	return narrow_cast<uint8_t>(fifoCount);
}

uint8_t SCCVoiceCore::readStatus(EmuTime time)
{
	sync(time);
	uint8_t result = 0;
	if (active)             result |= 0x01;
	if (underrun)           result |= 0x02;
	if (fifoCount == FIFO_CAPACITY) result |= 0x04;
	if (fifoCount == 0)         result |= 0x08;
	if (hold)               result |= 0x10;
	if (raw8Mode)           result |= 0x20;
	if (raw4Mode)           result |= 0x80;
	return result;
}

uint8_t SCCVoiceCore::peekStatus(EmuTime time) const
{
	const_cast<SCCVoiceCore*>(this)->sync(time);
	uint8_t result = 0;
	if (active)             result |= 0x01;
	if (underrun)           result |= 0x02;
	if (fifoCount == FIFO_CAPACITY) result |= 0x04;
	if (fifoCount == 0)         result |= 0x08;
	if (hold)               result |= 0x10;
	if (raw8Mode)           result |= 0x20;
	if (raw4Mode)           result |= 0x80;
	return result;
}


// ---- hardware-assisted buffer fill (ROM_xxx registers) ------------------

void SCCVoiceCore::writeRomStartAddrLow(uint8_t value, EmuTime time)
{
	sync(time);
	romStartAddr = narrow_cast<uint16_t>((romStartAddr & 0xFF00) | value);
	updateStream(time);
}

void SCCVoiceCore::writeRomStartAddrHigh(uint8_t value, EmuTime time)
{
	sync(time);
	romStartAddr = narrow_cast<uint16_t>((romStartAddr & 0x00FF) | (unsigned(value) << 8));
	updateStream(time);
}

uint8_t SCCVoiceCore::readRomStartAddrLow(EmuTime time)
{
	sync(time);
	return narrow_cast<uint8_t>(romStartAddr & 0xFF);
}

uint8_t SCCVoiceCore::readRomStartAddrHigh(EmuTime time)
{
	sync(time);
	return narrow_cast<uint8_t>((romStartAddr >> 8) & 0xFF);
}

uint8_t SCCVoiceCore::peekRomStartAddrLow(EmuTime time) const
{
	const_cast<SCCVoiceCore*>(this)->sync(time);
	return narrow_cast<uint8_t>(romStartAddr & 0xFF);
}

uint8_t SCCVoiceCore::peekRomStartAddrHigh(EmuTime time) const
{
	const_cast<SCCVoiceCore*>(this)->sync(time);
	return narrow_cast<uint8_t>((romStartAddr >> 8) & 0xFF);
}

void SCCVoiceCore::writeRomEndAddrLow(uint8_t value, EmuTime time)
{
	sync(time);
	romEndAddr = narrow_cast<uint16_t>((romEndAddr & 0xFF00) | value);
	updateStream(time);
}

void SCCVoiceCore::writeRomEndAddrHigh(uint8_t value, EmuTime time)
{
	sync(time);
	romEndAddr = narrow_cast<uint16_t>((romEndAddr & 0x00FF) | (unsigned(value) << 8));
	updateStream(time);
}

void SCCVoiceCore::writeRomRepeatAddrLow(uint8_t value, EmuTime time)
{
	sync(time);
	romRepeatAddr = narrow_cast<uint16_t>((romRepeatAddr & 0xFF00) | value);
	updateStream(time);
}

void SCCVoiceCore::writeRomRepeatAddrHigh(uint8_t value, EmuTime time)
{
	sync(time);
	romRepeatAddr = narrow_cast<uint16_t>((romRepeatAddr & 0x00FF) | (unsigned(value) << 8));
	updateStream(time);
}

void SCCVoiceCore::writeRomFillLevel(uint8_t value, EmuTime time)
{
	// Catch up any pending decode ticks FIRST, so fifoCount correctly
	// reflects what's genuinely already queued before we compute how
	// much more this request actually needs.
	sync(time);

	if (!active) {
		romFillRemaining = 0; // nothing to do if not playing
		updateStream(time);
		return;
	}

	int needed = int(value) - int(fifoCount);

	while ((needed > 0) && (fifoCount < FIFO_CAPACITY)) {
		// The end check is made BEFORE fetching a byte, not right after
		// the address increment. A fill that stops exactly at END
		// therefore leaves the engine waiting there, so a driver that
		// stitches a sample across banks can switch the bank and rewrite
		// START and END before the next fill, without the engine having
		// wrapped or stopped in the meantime.
		if (romStartAddr == romEndAddr) {
			// Load the loop start. REPEAT_ADDR == END is the "no loop"
			// encoding (an empty loop region): START then still equals END,
			// so the engine stops -- and stays stopped, since every later
			// fill request sees START == END again and repeats this same
			// harmless reload. No address value is reserved for "no loop",
			// so a loop may start anywhere, page 0 included. In hardware this
			// reuses the START == END comparator: a "just reloaded" flag
			// that is still set at the next compare means stop, and the same
			// flag marks the first fetched byte of a new pass. No mark is
			// armed here when stopping, so nothing leaks onto a later byte.
			romStartAddr = romRepeatAddr;
			if (romStartAddr == romEndAddr) {
				break; // no loop: nothing more to supply
			}

			// Click-free wrap: mark the FIRST byte of the next pass so that
			// the decoder reloads predictor and step index from the staged
			// override values (the one-time STEP_INDEX/PREDICTOR_L/H
			// writes at trigger time) when it reaches that byte. The mark
			// travels with the byte through the FIFO, so every wrap carries
			// its own mark -- also several wraps in one request, i.e. a loop
			// shorter than the buffer. The raw PCM modes (RAW8 and RAW4) have
			// no decoder state to restore, so this is skipped there. The
			// byte is pushed right below (the FIFO is known not to be full).
			if (!raw8Mode && !raw4Mode) {
				markNext = true;
				markNextStep = true;
			}
		}

		// The engine reads the cartridge's own ROM (or RAM) as its mapper
		// banks it in at this 16-bit address, also where the SCC(+)
		// register window would otherwise be visible (see
		// SCCVoiceRomBypassReader). It knows nothing of banks itself: the
		// driver maps the right bank before it requests a fill, and it never
		// sees the CPU's slot selection.
		pushByte(romReader.peekUnderlyingRom(romStartAddr, time));

		++romStartAddr;
		--needed;
	}

	romFillRemaining = (needed > 0) ? narrow_cast<uint8_t>(needed) : uint8_t(0);

	if (active) schedule(time);
	updateStream(time);
}

uint8_t SCCVoiceCore::readRomFillLevel(EmuTime /*time*/) const
{
	return romFillRemaining;
}

uint8_t SCCVoiceCore::peekRomFillLevel(EmuTime /*time*/) const
{
	return romFillRemaining;
}


// ---- start/stop/flush ---------------------------------------------------

void SCCVoiceCore::doStart(EmuTime time)
{
	// "initialiseert de playback state; reset de ADPCM-decoder state;
	//  wist UNDERRUN; zet ACTIVE; start de voice" (spec section 11)
	predictor = 0;
	stepIndex = 0;
	highNibbleNext = true;
	underrun = false;
	active = true;
	// A fresh decoder start shouldn't carry over a stale armed override
	// from a previous sample/session.
	markNext = markNextStep = false;
	overrideStepIndexPending = false;
	// Re-align the counter to 'time' so the first decoded sample lands
	// exactly one period after START, regardless of the previous phase.
	clock.reset(time);
	countdown = pitch + 1u;
}

void SCCVoiceCore::doStop(EmuTime time)
{
	// "Een expliciete STOP blijft beschikbaar als safety/debug-feature."
	// We intentionally do NOT clear the FIFO or reset the decoder state:
	// only playback halts. A following START will reset decoder state.
	//
	// Unlike underrun (which holds the last value to avoid an audible click
	// from an *unexpected* stop), an explicit STOP drops to silence: it's a
	// deliberate, controlled action by software, so there's no
	// surprise-avoidance rationale for holding the last value here.
	active = false;
	forceSilence(time);
}

void SCCVoiceCore::doFlush(EmuTime time)
{
	// Clears only the FIFO -- does not touch decoder state (predictor,
	// stepIndex) or the ACTIVE flag itself. A separate, minimal-cost
	// operation from START/STOP/RESET (added because manual register-
	// level testing repeatedly needed a way to clear stale FIFO content
	// without a full device reset).
	fifoHead = fifoTail = fifoCount = 0;
	currentByte = 0;
	highNibbleNext = true;
	// The FIFO is empty again: a mark that was armed but not yet
	// attached to a byte (and a staged step index) must not leak onto
	// the first byte of the next sample -- clear them.
	markNext = markNextStep = false;
	overrideStepIndexPending = false;

	if (active) {
		// Flushing an actively-playing voice's FIFO to empty has the
		// same consequence as the FIFO draining dry naturally: an
		// immediate underrun, triggered synchronously here rather than
		// waiting for a future scheduled tick that would otherwise
		// never come (schedule() has nothing left to count down from).
		triggerUnderrun(time);
	}
}


// ---- emu-time-accurate FIFO/decoder simulation --------------------------

void SCCVoiceCore::sync(EmuTime time)
{
	if (active) {
		// 'clock' is the slot clock; 'countdown' counts the slot clocks to the
		// next sample tick and reloads with pitch + 1 at every tick, so a
		// period written in between applies from the next sample on (as on
		// an SCC channel). Skips straight from one sample tick to the next.
		unsigned ticks = clock.getTicksTill(time);
		while (active && ticks >= countdown) {
			clock += countdown;
			ticks -= countdown;
			countdown = pitch + 1u;
			tick(clock.getTime());
		}
		if (active) {
			clock += ticks;
			countdown -= ticks;
		}
	}
	clock.advance(time);
}

void SCCVoiceCore::schedule(EmuTime /*time*/)
{
	removeSyncPoint();
	if (!active || fifoCount == 0) return;

	// Schedule exactly the NEXT sample tick -- not a lookahead to when the
	// FIFO would fully deplete. Scheduling one tick at a time keeps the
	// decoded deltas in 'blip' up to date at every moment, so whenever the
	// mixer pulls samples (its own regular sync point, or any device's
	// updateStream()) they are there, independent of external register
	// polling. tick() detects and handles running fully dry on its own
	// (triggerUnderrun()), so no separate depletion-prediction is needed.
	// sync() has brought 'clock' up to the caller's time, so the next tick
	// lies 'countdown' slot clocks after its last tick.
	setSyncPoint(clock.getFastAdd(countdown));
}

void SCCVoiceCore::executeUntil(EmuTime time)
{
	sync(time);
	if (active) {
		// Defensive: normally sync() above already consumed exactly the
		// scheduled single tick and, if that was the last available
		// byte, hit the underrun condition (clearing 'active'). If
		// still active, reschedule the next single tick to keep the
		// chain going.
		schedule(time);
	}
	// No updateStream() here: the tick's delta is already in 'blip' and
	// the mixer reads it at its own regular sync point (like DACSound16S).
	// Calling updateStream() makes the whole mixer render a tiny buffer
	// for every decoded nibble -- with voice A on a high note plus voice B
	// that is ~30000 mixer passes a second, which slows the emulation below
	// real time and makes the sound stutter.
}

void SCCVoiceCore::pushByte(uint8_t value)
{
	// The loop-override marks armed by PREDICTOR_H / the fill engine's wrap
	// attach to exactly this byte.
	fifo[fifoHead] = value;
	fifoMark[fifoHead] = narrow_cast<uint8_t>((markNext ? 1 : 0) | (markNextStep ? 2 : 0));
	markNext = markNextStep = false;
	fifoHead = (fifoHead + 1) % FIFO_SIZE;
	++fifoCount;
}

void SCCVoiceCore::forceSilence(EmuTime time)
{
	// Pushes an immediate delta bringing the output to 0 -- used by an
	// explicit STOP, and by an underrun without HOLD.
	BlipBuffer::TimeIndex t;
	getHostSampleClock().getTicksTill(time, t);
	blip.addDelta(t, 0.0f - lastOutput);
	lastOutput = 0.0f;
	lastSample = 0;
}

void SCCVoiceCore::triggerUnderrun(EmuTime time)
{
	// The FIFO ran dry while active: hardware auto-stop. Without HOLD the
	// output drops to silence, so a one-shot ends by itself; with HOLD it
	// keeps the last decoded value, so a melodic sample that briefly runs dry
	// does not click.
	active = false;
	underrun = true;
	if (!hold) {
		forceSilence(time);
	}
}

void SCCVoiceCore::tick(EmuTime time)
{
	int sample;

	if (raw8Mode) {
		// Raw 8-bit signed PCM bypass: 1 FIFO byte -> 1 output sample,
		// no OKI decode. Sign-extend and scale into the same
		// -2048..2047 range the ADPCM predictor uses, so everything
		// downstream (volume, filter, BlipBuffer) is unaffected by
		// which mode produced the sample.
		if (fifoCount == 0) {
			triggerUnderrun(time);
			return;
		}
		uint8_t rawByte = fifo[fifoTail];
		fifoTail = (fifoTail + 1) % FIFO_SIZE;
		--fifoCount;
		sample = int(int8_t(rawByte)) * 16; // -128..127 -> -2048..2032
		predictor = sample; // keep 'predictor' as "current PCM value"
		                     // regardless of which mode produced it
	} else {
		// Standard 4-bit OKI ADPCM decode, 2 nibbles/byte.
		if (highNibbleNext) {
			if (fifoCount == 0) {
				// FIFO underrun while active: hardware auto-stop.
				triggerUnderrun(time);
				return;
			}
			if (uint8_t mark = fifoMark[fifoTail]; mark & 1) {
				// The FIFO consumer has reached a byte that carries a mark
				// (set by PREDICTOR_H or by the fill engine's loop
				// wrap) -- apply the override
				// now, before decoding this byte's nibbles, so this
				// byte's deltas build on the new baseline (see
				// writePredictorHigh() doc comment for the full
				// click-free-loop usage pattern).
				predictor = overridePredictorValue;
				if (mark & 2) {
					stepIndex = overrideStepIndexValue;
				}
			}
			currentByte = fifo[fifoTail];
		}

		uint8_t nibble = highNibbleNext ? uint8_t(currentByte >> 4)
		                                 : uint8_t(currentByte & 0x0F);
		if (!highNibbleNext) {
			// finished this byte, pop it
			fifoTail = (fifoTail + 1) % FIFO_SIZE;
			--fifoCount;
		}
		highNibbleNext = !highNibbleNext;

		if (raw4Mode) {
			// Raw 4-bit PCM: same nibble sequencing as ADPCM, decoder
			// bypassed. Two's complement -8..7, reconstructed at the
			// middle of the quantisation step: v*256 + 128 gives the
			// symmetric range -1920..1920 with no DC offset. (A loop-wrap
			// override applied by the Z80 path is harmless here: it only
			// sets predictor/stepIndex, and 'predictor' is overwritten
			// with the new sample right below.)
			sample = rawNibbleToSample(nibble);
			predictor = sample; // keep 'predictor' as "current PCM value"
		} else {
			sample = decodeNibble(nibble);
		}
	}

	lastSample = sample;
	float out = float(sample) * (float(volume) / 15.0f);

	BlipBuffer::TimeIndex t;
	getHostSampleClock().getTicksTill(time, t);
	blip.addDelta(t, out - lastOutput);
	lastOutput = out;
}

int SCCVoiceCore::rawNibbleToSample(uint8_t nibble)
{
	int v = (nibble & 8) ? int(nibble) - 16 : int(nibble); // two's complement
	return v * 256 + 128; // -1920 .. +1920
}

int SCCVoiceCore::decodeNibble(uint8_t nibble)
{
	int step = okiStepTable[stepIndex];
	int diff = step >> 3;
	if (nibble & 4) diff += step;
	if (nibble & 2) diff += step >> 1;
	if (nibble & 1) diff += step >> 2;

	if (nibble & 8) {
		predictor -= diff;
	} else {
		predictor += diff;
	}
	predictor = std::clamp(predictor, -2048, 2047);

	int idx = int(stepIndex) + okiIndexAdjustTable[nibble & 7];
	stepIndex = narrow_cast<uint8_t>(std::clamp(idx, 0, 48));

	return predictor;
}


// ---- SoundDevice ----------------------------------------------------

void SCCVoiceCore::setOutputRate(unsigned hostSampleRate, double /*speed*/)
{
	setInputRate(hostSampleRate);
	computeFilterCoefficients(float(hostSampleRate));
}

void SCCVoiceCore::computeFilterCoefficients(float outputRate)
{
	// 2nd-order Sallen-Key-equivalent low-pass, fc ~= 3.5kHz, Q ~= 0.707
	// (RBJ audio-EQ-cookbook biquad formulas). This models the *fixed*
	// analog filter placed after the real MSM5205 DAC, so it is computed
	// against the actual host output rate, not the (variable) ADPCM
	// playback rate -- exactly like the real analog filter doesn't care
	// what rate the DAC is being updated at.
	constexpr float fc = 3500.0f;
	constexpr float Q = 0.70710678f; // 1/sqrt(2)

	float w0 = 2.0f * std::numbers::pi_v<float> * fc / outputRate;
	float cosw0 = std::cos(w0);
	float sinw0 = std::sin(w0);
	float alpha = sinw0 / (2.0f * Q);

	float a0 = 1.0f + alpha;
	lpB0 = ((1.0f - cosw0) / 2.0f) / a0;
	lpB1 = (1.0f - cosw0) / a0;
	lpB2 = lpB0;
	lpA1 = (-2.0f * cosw0) / a0;
	lpA2 = (1.0f - alpha) / a0;
}

void SCCVoiceCore::generateChannels(std::span<float*> bufs, unsigned num)
{
	assert(bufs.size() == 1);
	if (!blip.readSamples<1>(bufs[0], num)) {
		bufs[0] = nullptr;
		return;
	}

	// Apply the (ADPCM-path-only) Sallen-Key low-pass filter.
	float x1 = lpX1, x2 = lpX2, y1 = lpY1, y2 = lpY2;
	for (unsigned i = 0; i < num; ++i) {
		float x0 = bufs[0][i];
		float y0 = lpB0 * x0 + lpB1 * x1 + lpB2 * x2
		         - lpA1 * y1 - lpA2 * y2;
		x2 = x1; x1 = x0;
		y2 = y1; y1 = y0;
		bufs[0][i] = y0;
	}
	lpX1 = x1; lpX2 = x2;
	lpY1 = y1; lpY2 = y2;
}

bool SCCVoiceCore::updateBuffer(size_t length, float* buffer, EmuTime /*time*/)
{
	return mixChannels(buffer, length);
}

float SCCVoiceCore::getAmplificationFactorImpl() const
{
	// predictor range is -2048..2047 (12-bit signed), mixed at OUTPUT_GAIN
	return OUTPUT_GAIN / 2048.0f;
}


template<typename Archive>
void SCCVoiceCore::serialize(Archive& ar, unsigned /*version*/)
{
	ar.template serializeBase<Schedulable>(*this);
	ar.serialize("clock",         clock,
	             "fifo",          fifo,
	             "fifoHead",      fifoHead,
	             "fifoTail",      fifoTail,
	             "fifoCount",     fifoCount,
	             "currentByte",   currentByte,
	             "highNibbleNext",highNibbleNext,
	             "predictor",     predictor,
	             "stepIndex",     stepIndex,
	             "volume",        volume,
	             "active",        active,
	             "underrun",      underrun,
	             "pitch",         pitch,
	             "countdown",     countdown,
	             "raw8Mode",      raw8Mode,
	             "predictorOverrideStaging", predictorOverrideStaging,
	             "overridePredictorValue",   overridePredictorValue,
	             "overrideStepIndexValue",   overrideStepIndexValue,
	             "fifoMark",                 fifoMark,
	             "markNext",                 markNext,
	             "markNextStep",             markNextStep,
	             "lastSample",               lastSample,
	             "overrideStepIndexPending", overrideStepIndexPending,
	             "romStartAddr",    romStartAddr,
	             "romEndAddr",      romEndAddr,
	             "romRepeatAddr",   romRepeatAddr,
	             "romFillRemaining",romFillRemaining,
	             "raw4Mode",        raw4Mode,
	             "hold",            hold);

	if constexpr (Archive::IS_LOADER) {
		// Reschedule the next sample tick. The saved clock need not have been
		// synced up to the save time, so count from the current time: at
		// worst the first tick after loading moves by less than one period.
		clock.setFreq(SLOT_CLOCK_HZ);
		removeSyncPoint();
		if (active && fifoCount != 0) {
			setSyncPoint(clock.add(getCurrentTime(), countdown));
		}
	}
}
INSTANTIATE_SERIALIZE_METHODS(SCCVoiceCore);

} // namespace openmsx
