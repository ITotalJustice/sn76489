// this code is all based on the fantastic docs linked below
// SOURCE: https://www.smspower.org/uploads/Development/richard.txt
// SOURCE: https://www.smspower.org/uploads/Development/psg-20030421.txt

#include "sn76489.h"
#include "blargg/blip_wrap.h"
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

enum { PSG_CLOCK_DIVIDER = 16 }; // the apu runs x16 slower than cpu!

enum ChannelType {
    ChannelType_TONE0 = 0,
    ChannelType_TONE1 = 1,
    ChannelType_TONE2 = 2,
    ChannelType_NOISE = 3,
};

enum LatchType {
    LatchType_TONE = 0,
    LatchType_NOISE = 0,
    LatchType_VOL = 1,
};

enum {
    WHITE_NOISE = 1,
    PERIDOIC_NOISE = 0,
};

struct Sn76489Tone
{
    uint16_t tone; // 10-bits
};

struct Sn76489Noise
{
    uint16_t lfsr; // can be either 16-bit or 15-bit...
    uint8_t mode; // 1-bits
    uint8_t shift_rate; // 2-bits
    uint8_t flip_flop;
    uint8_t _padding[1];
};

struct Sn76489Channel
{
    // timestamp since last sync
    unsigned timestamp;
    // cycles since last sync
    unsigned clock;
    // previous left/right sample output, used by blip_buf
    int amp[2];
    // cycles until next clock
    int frequency_timer;
    // 4-bit output volume, reversed
    uint8_t volume;
    // +volume / -volume
    bool polarity;
    // GG has stereo switches for each channel
    bool enabled[2];
};

struct Sn76489
{
    /* for savestates, back up everything here. */
    struct Sn76489Channel channels[4];
    struct Sn76489Tone tone[3];
    struct Sn76489Noise noise;
    // which of the 4 channels are latched.
    uint8_t latched_channel;
    // vol or tone (or mode + shift instead of tone for noise).
    uint8_t latched_type;
    uint8_t _padding[2];
    /* end. */

    blip_wrap_t* blip;
    float channel_volume[4];
    unsigned lfsr_tapped_bit;
    unsigned lfsr_feed_bit;
};

// https://www.smspower.org/Development/SN76489#Volumeattenuation
static const int16_t VOLUME_TABLE[16] = {
    16383, 13014, 10337, 8211, 6522, 5181, 4115, 3284, 2596, 2062, 1638, 1301, 1033, 821, 652, 0
};

static const uint16_t NOISE_CHANNEL_FREQ[3] = {
    16*16, 32*16, 64*16
};

#define psg_min(x, y) (x) < (y) ? (x) : (y)
#define psg_max(x, y) (x) > (y) ? (x) : (y)
#define psg_clamp(a, x, y) psg_max(psg_min(a, y), x)
#define psg_array_size(a) (sizeof(a) / sizeof(a[0]))

static inline bool parity(unsigned value)
{
    #if defined(__has_builtin) && __has_builtin(__builtin_parity)
        return !__builtin_parity(value);
    #else
        // SOURCE: https://graphics.stanford.edu/~seander/bithacks.html#ParityParallel
        value ^= value >> 8; // 16-bit
        value ^= value >> 4; // 8-bit
        value &= 0xF;
        return !((0x6996 >> value) & 0x1);
    #endif
}

static inline unsigned channel_get_freq(const Sn76489* psg, int num)
{
    if (num == ChannelType_NOISE)
    {
        if (psg->noise.shift_rate == 3)
        {
            return psg->tone[2].tone * PSG_CLOCK_DIVIDER;
        }
        return NOISE_CHANNEL_FREQ[psg->noise.shift_rate];
    }
    else
    {
        return psg->tone[num].tone * PSG_CLOCK_DIVIDER;
    }
}

static inline void add_delta(Sn76489* psg, struct Sn76489Channel* c, unsigned clock_time, int sample, unsigned lr)
{
    const int delta = sample - c->amp[lr];
    if (delta) // same as (sample != amp)
    {
        blip_wrap_add_delta(psg->blip, clock_time, delta, lr);
        c->amp[lr] += delta; // same as (amp = sample)
    }
}

static inline void add_delta_fast(Sn76489* psg, struct Sn76489Channel* c, unsigned clock_time, int sample, unsigned lr)
{
    const int delta = sample - c->amp[lr];
    if (delta) // same as (sample != amp)
    {
        blip_wrap_add_delta_fast(psg->blip, clock_time, delta, lr);
        c->amp[lr] += delta; // same as (amp = sample)
    }
}

static void channel_sync_psg(Sn76489* psg, unsigned num, unsigned time)
{
    struct Sn76489Channel* c = &psg->channels[num];

    // get starting point
    const unsigned base_clock = c->clock;
    // i am not 100% sure how this works, but trust me, it works
    unsigned from = base_clock + c->frequency_timer;
    // get new timestamp
    const unsigned new_timestamp = time;
    // calculate how many cycles have elapsed since last sync
    const int until = new_timestamp - c->timestamp;
    // advance forward
    c->clock += until;
    // save new timestamp
    c->timestamp = new_timestamp;

    // already clocked on this cycle, or bad timestamp
    if (until <= 0)
    {
        return;
    }

    // clip clock range
    if (c->frequency_timer > until)
    {
        from = base_clock + until;
    }

    const int sample = blip_apply_volume_to_sample(psg->blip, VOLUME_TABLE[c->volume], psg->channel_volume[num]);
    // const int sample = VOLUME_TABLE[c->volume];
    const int sample_left = sample * c->enabled[0];
    const int sample_right = sample * c->enabled[1];

    /*
        from the docs:

        Sample playback makes use of a feature of the psg's tone generators:
        when the half-wavelength (tone value) is set to 1, they output a DC offset
        value corresponding to the volume level (i.e. the wave does not flip-flop).
        By rapidly manipulating the volume, a crude form of PCM is obtained.

        This effect is used by the sega intro in Tail's Adventure and
        sonic tripple trouble.
    */
    const unsigned freq = channel_get_freq(psg, num);
    if (freq <= PSG_CLOCK_DIVIDER)
    {
        const int left = +sample_left;
        const int right = +sample_right;
        add_delta_fast(psg, c, from, left, 0);
        add_delta_fast(psg, c, from, right, 1);
        return;
    }

    int left = (c->polarity ? +sample_left : -sample_left);
    int right = (c->polarity ? +sample_right : -sample_right);
    add_delta(psg, c, from, left, 0);
    add_delta(psg, c, from, right, 1);

    c->frequency_timer -= until;

    if (num == ChannelType_NOISE)
    {
        const unsigned tapped_bits = psg->lfsr_tapped_bit;
        const unsigned feedback_bit = psg->lfsr_feed_bit;

        while (c->frequency_timer <= 0)
        {
            psg->noise.flip_flop ^= 1;
            if (psg->noise.flip_flop)
            {
                psg->noise.lfsr = (psg->noise.lfsr >> 1) |
                    ((psg->noise.mode == WHITE_NOISE
                    ? parity(psg->noise.lfsr & tapped_bits)
                    : psg->noise.lfsr & 1) << feedback_bit);

                const unsigned new_polarity = psg->noise.lfsr & 0x1;
                if (c->polarity != new_polarity)
                {
                    c->polarity = new_polarity;
                    left = -left;
                    right = -right;
                    add_delta_fast(psg, c, from, left, 0);
                    add_delta_fast(psg, c, from, right, 1);
                }
            }

            c->frequency_timer += freq;
            from += freq;
        }
    }
    else // tone
    {
        while (c->frequency_timer <= 0)
        {
            c->polarity ^= 1;
            add_delta(psg, c, from, left, 0);
            add_delta(psg, c, from, right, 1);
            c->frequency_timer += freq;
            from += freq;
        }
    }
}

static void channel_sync_psg_all(Sn76489* psg, unsigned time)
{
    channel_sync_psg(psg, ChannelType_TONE0, time);
    channel_sync_psg(psg, ChannelType_TONE1, time);
    channel_sync_psg(psg, ChannelType_TONE2, time);
    channel_sync_psg(psg, ChannelType_NOISE, time);
}

static void noise_channel_reset(Sn76489* psg, unsigned data)
{
    psg->noise.lfsr = 1 << psg->lfsr_feed_bit;
    psg->noise.flip_flop = true; // todo: confirm flipflop is reset
    psg->noise.shift_rate = data & 0x3;
    psg->noise.mode = (data >> 2) & 0x1;
}

static void latch_reg_write(Sn76489* psg, unsigned value, unsigned time)
{
    const unsigned data = value & 0xF;
    psg->latched_channel = (value >> 5) & 0x3;
    psg->latched_type = (value >> 4) & 0x1;

    // sync the newly latched channel as the data is about to be changed
    channel_sync_psg(psg, psg->latched_channel, time);

    if (psg->latched_type == LatchType_VOL)
    {
        psg->channels[psg->latched_channel].volume = data & 0xF;
    }
    else
    {
        switch (psg->latched_channel)
        {
            case 0: case 1: case 2:
                psg->tone[psg->latched_channel].tone &= 0x3F0;
                psg->tone[psg->latched_channel].tone |= data;
                break;

            case 3:
                noise_channel_reset(psg, data);
                break;
        }
    }
}

static void data_reg_write(Sn76489* psg, unsigned value, unsigned time)
{
    const unsigned data = value & 0x3F;
    channel_sync_psg(psg, psg->latched_channel, time);

    if (psg->latched_type == LatchType_VOL)
    {
        psg->channels[psg->latched_channel].volume = data & 0xF;
    }
    else
    {
        switch (psg->latched_channel)
        {
            case 0: case 1: case 2:
                psg->tone[psg->latched_channel].tone &= 0xF;
                psg->tone[psg->latched_channel].tone |= data << 4;
                break;

            case 3:
                noise_channel_reset(psg, data);
                break;
        }
    }
}

Sn76489* psg_init(double clock_rate, double sample_rate)
{
    Sn76489* psg = calloc(1, sizeof(*psg));
    if (!psg)
    {
        goto fail;
    }

    for (unsigned i = 0; i < psg_array_size(psg->channel_volume); i++)
    {
        psg->channel_volume[i] = 1.0;
    }

    if (!(psg->blip = blip_wrap_new(sample_rate))) {
        goto fail;
    }

    if (blip_wrap_set_rates(psg->blip, clock_rate, sample_rate)) {
        goto fail;
    }

    psg_set_master_volume(psg, 0.25);

    return psg;

fail:
    psg_quit(psg);
    return NULL;
}

void psg_quit(Sn76489* psg)
{
    if (psg)
    {
        if (psg->blip)
        {
            blip_wrap_delete(psg->blip);
            psg->blip = NULL;
        }
        free(psg);
    }
}

void psg_reset(Sn76489* psg, unsigned lfsr_tapped_bit, unsigned lfsr_feed_bit)
{
    psg_clear_samples(psg);
    memset(&psg->channels, 0, sizeof(psg->channels));
    memset(&psg->tone, 0, sizeof(psg->tone));
    memset(&psg->noise, 0, sizeof(psg->noise));
    for (unsigned i = 0; i < psg_array_size(psg->channels); i++)
    {
        psg->channels[i].polarity = 1;
        psg->channels[i].volume = 0xF;
        psg->channels[i].enabled[0] = true;
        psg->channels[i].enabled[1] = true;
    }
    psg->lfsr_tapped_bit = lfsr_tapped_bit;
    psg->lfsr_feed_bit = lfsr_feed_bit;
    noise_channel_reset(psg, 0);
    psg->latched_channel = 0;
}

void psg_write_io(Sn76489* psg, unsigned value, unsigned time)
{
    // if MSB is set, then this is a latched write, else its a normal data write
    if (value & 0x80)
    {
        latch_reg_write(psg, value, time);
    }
    else
    {
        data_reg_write(psg, value, time);
    }
}

void psg_gg_write_io(Sn76489* psg, unsigned value, unsigned time)
{
    channel_sync_psg_all(psg, time);
    // left side of channels
    psg->channels[0].enabled[0] = value & 0x10;
    psg->channels[1].enabled[0] = value & 0x20;
    psg->channels[2].enabled[0] = value & 0x40;
    psg->channels[3].enabled[0] = value & 0x80;
    // right side of channels
    psg->channels[0].enabled[1] = value & 0x01;
    psg->channels[1].enabled[1] = value & 0x02;
    psg->channels[2].enabled[1] = value & 0x04;
    psg->channels[3].enabled[1] = value & 0x08;
}

void psg_set_channel_volume(Sn76489* psg, unsigned channel_num, float volume)
{
    psg->channel_volume[channel_num] = psg_clamp(volume, 0.0F, 1.0F);
}

void psg_set_master_volume(Sn76489* psg, float volume)
{
    blip_wrap_set_volume(psg->blip, psg_clamp(volume, 0.0F, 1.0F));
}

void psg_set_bass(Sn76489* psg, int frequency)
{
    blip_wrap_set_bass(psg->blip, frequency);
}

void psg_set_treble(Sn76489* psg, double treble_db)
{
    blip_wrap_set_treble(psg->blip, treble_db);
}

void psg_update_timestamp(Sn76489* psg, int time)
{
    for (unsigned i = 0; i < psg_array_size(psg->channels); i++)
    {
        psg->channels[i].timestamp += time;
    }
}

int psg_clocks_needed(const Sn76489* psg, int sample_count)
{
    return blip_wrap_clocks_needed(psg->blip, sample_count);
}

int psg_samples_avaliable(const Sn76489* psg)
{
    return blip_wrap_samples_avail(psg->blip);
}

void psg_end_frame(Sn76489* psg, unsigned time)
{
    // catchup all the channels to the same point.
    channel_sync_psg_all(psg, time);

    // clocks of all channels will be the same as they're synced above.
    const unsigned clock_duration = psg->channels[0].clock;

    // reset clocks.
    for (unsigned i = 0; i < psg_array_size(psg->channels); i++)
    {
        assert(clock_duration == psg->channels[i].clock);
        psg->channels[i].clock = 0;
    }

    // make all samples up to this clock point available.
    blip_wrap_end_frame(psg->blip, clock_duration);
}

int psg_read_samples(Sn76489* psg, short out[], int count)
{
    return blip_wrap_read_samples(psg->blip, out, count);
}

void psg_clear_samples(Sn76489* psg)
{
    blip_wrap_clear(psg->blip);
}

#if (defined(__cplusplus) && __cplusplus < 201103L) || (!defined(static_assert))
  #if defined(__STDC_VERSION__) && __STDC_VERSION__ >= 201112L
    #define static_assert _Static_assert
  #else
    #define static_assert(expr, msg) typedef char static_assertion[(expr) ? 1 : -1]
  #endif
#endif

static_assert(offsetof(Sn76489, channels) == 0, "bad channels offset, save states broken!");
static_assert(offsetof(Sn76489, tone) == 96, "bad tone offset, save states broken!");
static_assert(offsetof(Sn76489, noise) == 102, "bad noise offset, save states broken!");
static_assert(offsetof(Sn76489, latched_channel) == 108, "bad latched_channel offset, save states broken!");
static_assert(offsetof(Sn76489, latched_type) == 109, "bad latched_type offset, save states broken!");
static_assert(offsetof(Sn76489, _padding) == 110, "bad _padding offset, save states broken!");
static_assert(offsetof(Sn76489, blip) == 112, "bad blip offset, save states broken!");

unsigned psg_state_size(void)
{
    return offsetof(Sn76489, blip);
}

int psg_save_state(const Sn76489* psg, void* data, unsigned size)
{
    if (!data || size < psg_state_size())
    {
        return 1;
    }

    return !memcpy(data, psg, psg_state_size());
}

int psg_load_state(Sn76489* psg, const void* data, unsigned size)
{
    if (!data || size < psg_state_size())
    {
        return 1;
    }

    return !memcpy(psg, data, psg_state_size());
}
