#include "blip_wrap.h"
#include "blargg/blip_buf.h"
#include <stdint.h>
#include <stdlib.h>

struct blip_wrap_t
{
    blip_t* buf[2];
    float volume;
};

blip_wrap_t* blip_wrap_new(double sample_rate)
{
    blip_wrap_t* b = malloc(sizeof(*b));
    if (b) {
        b->volume = 0;
        b->buf[0] = blip_new(sample_rate / 10);
        b->buf[1] = blip_new(sample_rate / 10);

        if (!b->buf[0] || !b->buf[1]) {
            if (b->buf[0]) {
                free(b->buf[0]); b->buf[0] = NULL;
            }
            if (b->buf[1]) {
                free(b->buf[1]); b->buf[1] = NULL;
            }
            free(b); b = NULL;
        }
    }
    return b;
}

void blip_wrap_delete(blip_wrap_t* b)
{
    if (b)
    {
        if (b->buf[0])
        {
            blip_delete(b->buf[0]);
        }
        if (b->buf[1])
        {
            blip_delete(b->buf[1]);
        }
        free(b);
    }
}

int blip_wrap_set_rates(blip_wrap_t* b, double clock_rate, double sample_rate)
{
    blip_set_rates(b->buf[0], clock_rate, sample_rate);
    blip_set_rates(b->buf[1], clock_rate, sample_rate);
    return 0;
}

void blip_wrap_clear(blip_wrap_t* b)
{
    blip_clear(b->buf[0]);
    blip_clear(b->buf[1]);
}

void blip_wrap_add_delta(blip_wrap_t* b, unsigned clock_time, int delta, int lr)
{
    blip_add_delta(b->buf[lr], clock_time, delta);
}

void blip_wrap_add_delta_fast(blip_wrap_t* b, unsigned clock_time, int delta, int lr)
{
    blip_add_delta_fast(b->buf[lr], clock_time, delta);
}

int blip_wrap_clocks_needed(const blip_wrap_t* b, int sample_count)
{
    return blip_clocks_needed(b->buf[0], sample_count / 2);
}

void blip_wrap_end_frame(blip_wrap_t* b, unsigned clock_duration)
{
    blip_end_frame(b->buf[0], clock_duration);
    blip_end_frame(b->buf[1], clock_duration);
}

int blip_wrap_samples_avail(const blip_wrap_t* b)
{
    return blip_samples_avail(b->buf[0]) * 2;
}

int blip_wrap_read_samples(blip_wrap_t* b, short out[], int count)
{
    blip_read_samples(b->buf[0], out + 0, count / 2, 1);
    return blip_read_samples(b->buf[1], out + 1, count / 2, 1) * 2;
}

int blip_apply_volume_to_sample(blip_wrap_t* b, int sample, float volume)
{
#ifdef __NDS__
    return sample;
#else
    return sample * b->volume * volume;
#endif
}

void blip_wrap_set_volume(blip_wrap_t* b, float volume)
{
    b->volume = volume;
}

// only available when using blip_buffer.cpp
void blip_wrap_set_bass(blip_wrap_t* b, int frequency)
{
}

void blip_wrap_set_treble(blip_wrap_t* b, double treble_db)
{
}

unsigned blip_wrap_state_size(const blip_wrap_t* b)
{
    return blip_state_size(b->buf[0]) * 2;
}

int blip_wrap_save_state(const blip_wrap_t* b, void* buf, unsigned size)
{
    if (size < blip_wrap_state_size(b))
    {
        return 1;
    }

    const unsigned state_size = blip_state_size(b->buf[0]);
    if (blip_save_state(b->buf[0], buf, state_size) < 0)
    {
        return 1;
    }
    if (blip_save_state(b->buf[1], (uint8_t*)buf + state_size, state_size) < 0)
    {
        return 1;
    }

    return 0;
}

int blip_wrap_load_state(blip_wrap_t* b, const void* buf, unsigned size)
{
    if (size < blip_wrap_state_size(b))
    {
        return -1;
    }

    const unsigned state_size = blip_state_size(b->buf[0]);
    if (blip_load_state(b->buf[0], buf, state_size) < 0)
    {
        return -1;
    }
    if (blip_load_state(b->buf[1], (const uint8_t*)buf + state_size, state_size) < 0)
    {
        return -1;
    }

    return 0;
}
