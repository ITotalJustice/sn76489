#ifndef SN76489_H
#define SN76489_H

#ifdef __cplusplus
extern "C" {
#endif

enum Sn76489LfsrTappedBit {
    Sn76489LfsrTappedBit_SMS = 0x0009, // sms, gg
    Sn76489LfsrTappedBit_GG = 0x0009, // sms, gg
    Sn76489LfsrTappedBit_SG = 14, // sg100
};

enum Sn76489LfsrFeedBit {
    Sn76489LfsrFeedBit_SMS = 15, // sms, gg
    Sn76489LfsrFeedBit_GG = 15, // sms, gg
    Sn76489LfsrFeedBit_SG = 14, // sg100
};

typedef struct Sn76489 Sn76489;

/* ------------------------- */
/* ------Initialise Api----- */
/* ------------------------- */
/* ensure you call this at start-up .*/
Sn76489* psg_init(double clock_rate, double sample_rate);
/* call to free allocated memory by blip buf. */
void psg_quit(Sn76489*);
/* clock_rate should be the cpu speed of the system. */
void psg_reset(Sn76489*, unsigned lfsr_tapped_bit, unsigned lfsr_feed_bit);

/* ------------------------- */
/* ------Input Api------ */
/* ------------------------- */
/* writes to an io register. */
void psg_write_io(Sn76489*, unsigned value, unsigned time);
/* used for GG to enable / disable channels. */
void psg_gg_write_io(Sn76489*, unsigned value, unsigned time);

/* ------------------------- */
/* ------Configure Api------ */
/* ------------------------- */
/* channel volume, max range: 0.0 - 1.0. */
void psg_set_channel_volume(Sn76489*, unsigned channel_num, float volume);
/* master volume, max range: 0.0 - 1.0. */
void psg_set_master_volume(Sn76489*, float volume);
/* only available with Blip_Buffer. */
void psg_set_bass(Sn76489*, int frequency);
/* only available with Blip_Buffer. */
void psg_set_treble(Sn76489*, double treble_db);
/* updates timestamp, useful if the time overflows. */
void psg_update_timestamp(Sn76489*, int time);

/* ------------------------- */
/* ------Advanced Api------- */
/* ------------------------- */
/* returns value of io register, without unused bits applied / masked, */
/* regardless if the apu is enabled or not. */
/* useful for gui io viewer. */
unsigned psg_read_io_raw(const Sn76489*, unsigned addr);
unsigned psg_agb_read_io_raw(const Sn76489*, unsigned addr);

/* ------------------------- */
/* ------Sample Output------ */
/* ------------------------- */
/* returns how many cycles are needed until sample_count == psg_samples_avaliable() */
int psg_clocks_needed(const Sn76489*, int sample_count);
/* returns how many samples are available, call psg_end_frame() first. */
int psg_samples_avaliable(const Sn76489*);
/* call this when you want to read out samples. */
void psg_end_frame(Sn76489*, unsigned time);
/* read stereo samples, returns the amount read. */
int psg_read_samples(Sn76489*, short out[], int count);
/* removes all samples. */
void psg_clear_samples(Sn76489*);

#ifdef __cplusplus
}
#endif

#endif /* SN76489_H */
