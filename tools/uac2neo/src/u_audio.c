// SPDX-License-Identifier: GPL-2.0+
/*
 * u_audio.c -- interface to USB gadget "ALSA sound card" utilities
 *
 * Copyright (C) 2016
 * Author: Ruslan Bilovol <ruslan.bilovol@gmail.com>
 *
 * Sound card implementation was cut-and-pasted with changes
 * from f_uac2.c and has:
 *    Copyright (C) 2011
 *    Yadwinder Singh (yadi.brar01@gmail.com)
 *    Jaswinder Singh (jaswinder.singh@linaro.org)
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/control.h>
#include <sound/tlv.h>
#include <linux/usb/audio.h>

#include "u_audio.h"

#define BUFF_SIZE_MAX	(PAGE_SIZE * 512)
#define PRD_SIZE_MAX	(PAGE_SIZE * 4)
#define MIN_PERIODS	4

enum {
	UAC_FBACK_CTRL,
	UAC_P_PITCH_CTRL,
	UAC_MUTE_CTRL,
	UAC_VOLUME_CTRL,
	UAC_RATE_CTRL,
};

#define CLK_PPM_GROUP_SIZE	20

/* Runtime data params for one stream */
struct uac_rtd_params {
	struct snd_uac_chip *uac; /* parent chip */
	bool ep_enabled; /* if the ep is enabled */

	struct snd_pcm_substream *ss;

	/* Ring buffer */
	ssize_t hw_ptr;

	void *rbuf;

	unsigned int pitch;	/* Stream pitch ratio to 1000000 */
	long fb_integral;	/* PI controller integrator state */
	long fb_err_smooth;	/* EMA-filtered error for P-term */
	unsigned long fb_freeze;/* Freeze PI for N feedback cycles */
	unsigned long fb_freeze_grace;/* Grace period: suppress emergency unfreeze */
	unsigned int max_psize;	/* MaxPacketSize of endpoint */

	struct usb_request **reqs;

	struct usb_request *req_fback; /* Feedback endpoint request */
	bool fb_ep_enabled; /* if the ep is enabled */

  /* Volume/Mute controls and their state */
  int fu_id; /* Feature Unit ID */
  struct snd_ctl_elem_id snd_kctl_volume_id;
  struct snd_ctl_elem_id snd_kctl_mute_id;
  s16 volume_min, volume_max, volume_res;
  s16 volume;
  int mute;

	struct snd_ctl_elem_id snd_kctl_rate_id; /* read-only current rate */
	int srate; /* selected samplerate */
	int active; /* playback/capture running */

  spinlock_t lock; /* lock for control transfers */

};

struct snd_uac_chip {
	struct g_audio *audio_dev;

	struct uac_rtd_params p_prm;
	struct uac_rtd_params c_prm;

	struct snd_card *card;
	struct snd_pcm *pcm;

	/* Sysfs attributes for format change notification */
	struct device *dev;
	struct kobject *kobj;
	int current_rate;
	int current_format;
	int current_channels;

	/* pre-calculated values for playback iso completion */
	unsigned long long p_residue_mil;
	unsigned int p_interval;
	unsigned int p_framesize;
};

static const struct snd_pcm_hardware uac_pcm_hardware = {
	.info = SNDRV_PCM_INFO_INTERLEAVED | SNDRV_PCM_INFO_BLOCK_TRANSFER
		 | SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_MMAP_VALID
		 | SNDRV_PCM_INFO_PAUSE | SNDRV_PCM_INFO_RESUME,
	.rates = SNDRV_PCM_RATE_CONTINUOUS,
	.periods_max = BUFF_SIZE_MAX / PRD_SIZE_MAX,
	.buffer_bytes_max = BUFF_SIZE_MAX,
	.period_bytes_max = PRD_SIZE_MAX,
	.periods_min = MIN_PERIODS,
};

static struct class *audio_class;
static void u_audio_set_fback_frequency(enum usb_device_speed speed,
					struct usb_ep *out_ep,
					unsigned long long freq,
					unsigned int pitch,
					void *buf)
{
	u32 ff = 0;
	const struct usb_endpoint_descriptor *ep_desc;

	/*
	 * Because the pitch base is 1000000, the final divider here
	 * will be 1000 * 1000000 = 1953125 << 9
	 *
	 * Instead of dealing with big numbers lets fold this 9 left shift
	 */

	if (speed == USB_SPEED_FULL) {
		/*
		 * Full-speed feedback endpoints report frequency
		 * in samples/frame
		 * Format is encoded in Q10.10 left-justified in the 24 bits,
		 * so that it has a Q10.14 format.
		 *
		 * ff = (freq << 14) / 1000
		 */
		freq <<= 5;
	} else {
		/*
		 * High-speed feedback endpoints report frequency
		 * in samples/microframe.
		 * Format is encoded in Q12.13 fitted into four bytes so that
		 * the binary point is located between the second and the third
		 * byte fromat (that is Q16.16)
		 *
		 * ff = (freq << 16) / 8000
		 *
		 * Win10 and OSX UAC2 drivers require number of samples per packet
		 * in order to honor the feedback value.
		 * Linux snd-usb-audio detects the applied bit-shift automatically.
		 */
		ep_desc = out_ep->desc;
		freq <<= 4 + (ep_desc->bInterval - 1);
	}

	ff = DIV_ROUND_CLOSEST_ULL((freq * pitch), 1953125);

	*(__le32 *)buf = cpu_to_le32(ff);
}

static void u_audio_iso_complete(struct usb_ep *ep, struct usb_request *req)
{
	unsigned int pending;
	unsigned int hw_ptr;
	int status = req->status;
	struct snd_pcm_substream *substream;
	struct snd_pcm_runtime *runtime;
	struct uac_rtd_params *prm = req->context;
	struct snd_uac_chip *uac = prm->uac;
	unsigned int frames, p_pktsize;
	unsigned long long pitched_rate_mil, p_pktsize_residue_mil,
			residue_frames_mil, div_result;

	/* i/f shutting down */
	if (!prm->ep_enabled) {
		usb_ep_free_request(ep, req);
		return;
	}

	if (req->status == -ESHUTDOWN)
		return;

	/*
	 * We can't really do much about bad xfers.
	 * Afterall, the ISOCH xfers could fail legitimately.
	 */
	if (status)
		pr_debug("%s: iso_complete status(%d) %d/%d\n",
			__func__, status, req->actual, req->length);

	substream = prm->ss;

	/* Do nothing if ALSA isn't active */
	if (!substream)
		goto exit;

	snd_pcm_stream_lock(substream);

	runtime = substream->runtime;
	if (!runtime || !snd_pcm_running(substream)) {
		snd_pcm_stream_unlock(substream);
		goto exit;
	}

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		/*
		 * For each IN packet, take the quotient of the current data
		 * rate and the endpoint's interval as the base packet size.
		 * If there is a residue from this division, add it to the
		 * residue accumulator.
		 */
		unsigned long long p_interval_mil = uac->p_interval * 1000000ULL;

		pitched_rate_mil = (unsigned long long) prm->srate * prm->pitch;
		div_result = pitched_rate_mil;
		do_div(div_result, uac->p_interval);
		do_div(div_result, 1000000);
		frames = (unsigned int) div_result;

		pr_debug("p_srate %d, pitch %d, interval_mil %llu, frames %d\n",
				prm->srate, prm->pitch, p_interval_mil, frames);

		p_pktsize = min_t(unsigned int,
					uac->p_framesize * frames,
					ep->maxpacket);

		if (p_pktsize < ep->maxpacket) {
			residue_frames_mil = pitched_rate_mil - frames * p_interval_mil;
			p_pktsize_residue_mil = uac->p_framesize * residue_frames_mil;
		} else
			p_pktsize_residue_mil = 0;

		req->length = p_pktsize;
		uac->p_residue_mil += p_pktsize_residue_mil;

		/*
		 * Whenever there are more bytes in the accumulator p_residue_mil than we
		 * need to add one more sample frame, increase this packet's
		 * size and decrease the accumulator.
		 */
		div_result = uac->p_residue_mil;
		do_div(div_result, uac->p_interval);
		do_div(div_result, 1000000);
		if ((unsigned int) div_result >= uac->p_framesize) {
			req->length += uac->p_framesize;
			uac->p_residue_mil -= uac->p_framesize * p_interval_mil;
			pr_debug("increased req length to %d\n", req->length);
		}
		pr_debug("remains uac->p_residue_mil %llu\n", uac->p_residue_mil);

		req->actual = req->length;
	}

	hw_ptr = prm->hw_ptr;

	/* Pack USB load in ALSA ring buffer */
	pending = runtime->dma_bytes - hw_ptr;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		if (unlikely(pending < req->actual)) {
			memcpy(req->buf, runtime->dma_area + hw_ptr, pending);
			memcpy(req->buf + pending, runtime->dma_area,
			       req->actual - pending);
		} else {
			memcpy(req->buf, runtime->dma_area + hw_ptr,
			       req->actual);
		}
	} else {
		unsigned int actual = req->actual;

		/*
		 * Frame alignment guard: DSD is destroyed by even one
		 * misaligned byte — the entire subsequent stream gets
		 * L/R channels interleaved wrong, producing broadband
		 * noise.  Truncate to a whole number of frames.
		 */
		if (runtime->frame_bits) {
			unsigned int frame_bytes = runtime->frame_bits >> 3;

			if (unlikely(actual % frame_bytes)) {
				dev_warn_ratelimited(uac->card->dev,
					"ISO OUT: partial frame %u bytes (frame=%u), truncating\n",
					actual, frame_bytes);
				actual -= actual % frame_bytes;
			}
		}
		if (unlikely(!actual))
			goto unlock;

		if (unlikely(pending < actual)) {
			memcpy(runtime->dma_area + hw_ptr, req->buf, pending);
			memcpy(runtime->dma_area, req->buf + pending,
			       actual - pending);
		} else {
			memcpy(runtime->dma_area + hw_ptr, req->buf,
			       actual);
		}

		req->actual = actual;
	}

	/* update hw_ptr after data is copied to memory */
	prm->hw_ptr = (hw_ptr + req->actual) % runtime->dma_bytes;
	hw_ptr = prm->hw_ptr;
unlock:
	snd_pcm_stream_unlock(substream);

	if ((hw_ptr % snd_pcm_lib_period_bytes(substream)) < req->actual)
		snd_pcm_period_elapsed(substream);

exit:
	if (usb_ep_queue(ep, req, GFP_ATOMIC))
		dev_err(uac->card->dev, "%d Error!\n", __LINE__);
}

static void u_audio_iso_fback_complete(struct usb_ep *ep,
				       struct usb_request *req)
{
	struct uac_rtd_params *prm = req->context;
	struct snd_uac_chip *uac = prm->uac;
	struct g_audio *audio_dev = uac->audio_dev;
	int status = req->status;

	/* i/f shutting down */
	if (!prm->fb_ep_enabled) {
		kfree(req->buf);
		usb_ep_free_request(ep, req);
		return;
	}

	if (req->status == -ESHUTDOWN)
		return;

	if (status)
		pr_debug("%s: iso_complete status(%d) %d/%d\n",
			__func__, status, req->actual, req->length);

	/*
	 * Adaptive feedback: PI controller for hi-end audio.
	 *
	 * EMA filter (alpha=1/256): smooths sawtooth from single-threaded router.
	 * P-term (Kp=1/32): strong damping, resists buffer changes.
	 * I-term (Ki=1/262144): very slow integration, no overshoot.
	 *
	 * Clamping anti-windup prevents integrator saturation.
	 * fb_freeze: when >0, PI is frozen (pitch from sysfs held as-is).
	 * Decremented each cycle; unfreezes when reaching 0.
	 */
	if (prm->fb_freeze > 0) {
		struct snd_pcm_substream *ss = prm->ss;

		prm->fb_freeze--;
		if (prm->fb_freeze_grace > 0)
			prm->fb_freeze_grace--;
		/*
		 * Emergency unfreeze: if capture buffer drifts beyond
		 * safe limits (12.5%–87.5%), the saved pitch doesn't
		 * match this USB host — let PI correct immediately.
		 * Skip during grace period (first 2s) while buffer fills.
		 */
		if (prm->fb_freeze_grace == 0 &&
		    ss && ss->runtime && snd_pcm_running(ss)) {
			struct snd_pcm_runtime *rt = ss->runtime;
			snd_pcm_uframes_t hw, ap, fl;

			hw = bytes_to_frames(rt, prm->hw_ptr);
			ap = rt->control->appl_ptr % rt->buffer_size;
			fl = (hw >= ap) ? hw - ap
					: hw + rt->buffer_size - ap;
			if (fl < rt->buffer_size / 64 ||
			    fl > rt->buffer_size * 63 / 64) {
				prm->fb_freeze = 0;
				goto run_pi;
			}
		}
		goto skip_pi;
	}
run_pi:
	{
		struct snd_pcm_substream *ss = prm->ss;

		if (ss && ss->runtime && snd_pcm_running(ss)) {
			struct snd_pcm_runtime *runtime = ss->runtime;
			snd_pcm_uframes_t hw_frames, appl_frames;
			snd_pcm_sframes_t fill, target, error;
			long adjust, p_term;

			hw_frames = bytes_to_frames(runtime, prm->hw_ptr);
			appl_frames = runtime->control->appl_ptr %
				      runtime->buffer_size;
			fill = hw_frames - appl_frames;
			if (fill < 0)
				fill += runtime->buffer_size;

			/* Target = small fraction of buffer.
			 * The userspace router reads back-to-back,
			 * so the natural steady-state fill is only
			 * ~0-512 frames (one period).
			 * buffer_size/128 ≈ 512, matching the natural level. */
			target = runtime->buffer_size / 128;
			error = fill - target;

			/* EMA filter: alpha=1/256, time constant ~32ms at HS.
			 * Increased from 1/64 to smooth out sawtooth oscillations
			 * from single-threaded router architecture. */
			prm->fb_err_smooth +=
				(error - prm->fb_err_smooth + 128) / 256;

			/* I-term accumulator (full precision) */
			prm->fb_integral += prm->fb_err_smooth;

			/* Rate-scaled PI gains.
			 * Kp = 1/(32*rr), Ki = 1/(262144*rr).
			 *
			 * Ki was reduced 32x (from 8192 to 262144 denom)
			 * to fix oscillation: loop gain at the system's
			 * natural frequency was 15x too high, causing
			 * sustained ±500 PPM swings. With 262144 the
			 * gain margin is ~6 dB — stable convergence. */
			{
			long rr = (long)prm->srate / 44100;
			if (rr < 1) rr = 1;

			p_term = (2 * prm->fb_err_smooth) / rr;
			adjust = p_term +
				 prm->fb_integral / (262144L * rr);

			/* Back-calculation anti-windup:
			 * When adjust is clamped, recalculate fb_integral
			 * to match the clamped output. This prevents windup
			 * regardless of whether P-term or I-term caused saturation. */
			if (adjust > 500) {
				adjust = 500;
				prm->fb_integral = (500L - p_term) * 262144L * rr;
			} else if (adjust < -500) {
				adjust = -500;
				prm->fb_integral = (-500L - p_term) * 262144L * rr;
			}
			}

			prm->pitch = (unsigned int)(1000000L - adjust);
		}
	}
skip_pi:
	u_audio_set_fback_frequency(audio_dev->gadget->speed, audio_dev->out_ep,
				    prm->srate, prm->pitch,
				    req->buf);

	if (usb_ep_queue(ep, req, GFP_ATOMIC))
		dev_err(uac->card->dev, "%d Error!\n", __LINE__);
}

static int uac_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct snd_uac_chip *uac = snd_pcm_substream_chip(substream);
	struct uac_rtd_params *prm;
	struct g_audio *audio_dev;
	struct uac_params *params;
	unsigned int max_psize;
	int err = 0;

	audio_dev = uac->audio_dev;
	params = &audio_dev->params;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		prm = &uac->p_prm;
	else
		prm = &uac->c_prm;

	/* Reset */
	prm->hw_ptr = 0;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
		prm->ss = substream;
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
		prm->ss = NULL;
		break;
	default:
		err = -EINVAL;
	}

	/* Clear buffer after Play stops */
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK && !prm->ss) {
		max_psize = ALIGN(prm->max_psize, cache_line_size());
		memset(prm->rbuf, 0, max_psize * params->req_number);
	}

	return err;
}

static snd_pcm_uframes_t uac_pcm_pointer(struct snd_pcm_substream *substream)
{
	struct snd_uac_chip *uac = snd_pcm_substream_chip(substream);
	struct uac_rtd_params *prm;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		prm = &uac->p_prm;
	else
		prm = &uac->c_prm;

	return bytes_to_frames(substream->runtime, prm->hw_ptr);
}

static u64 uac_ssize_to_fmt(int ssize)
{
	u64 ret;

	switch (ssize) {
	case 3:
		ret = SNDRV_PCM_FMTBIT_S24_3LE;
		break;
	case 4:
		ret = SNDRV_PCM_FMTBIT_S32_LE;
		break;
	default:
		ret = SNDRV_PCM_FMTBIT_S16_LE;
		break;
	}

	return ret;
}

static int uac_pcm_open(struct snd_pcm_substream *substream)
{
	struct snd_uac_chip *uac = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct g_audio *audio_dev;
	struct uac_params *params;
	struct uac_rtd_params *prm;
	int p_ssize, c_ssize;
	int p_chmask, c_chmask;
	int old_rate, old_format, old_channels;

	audio_dev = uac->audio_dev;
	params = &audio_dev->params;
	p_ssize = params->p_ssize;
	c_ssize = params->c_ssize;
	p_chmask = params->p_chmask;
	c_chmask = params->c_chmask;
	uac->p_residue_mil = 0;

	runtime->hw = uac_pcm_hardware;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		runtime->hw.formats = uac_ssize_to_fmt(p_ssize);
		runtime->hw.channels_min = num_channels(p_chmask);
		prm = &uac->p_prm;
	} else {
		runtime->hw.formats = uac_ssize_to_fmt(c_ssize);
		runtime->hw.channels_min = num_channels(c_chmask);
		prm = &uac->c_prm;
	}

	runtime->hw.period_bytes_min = 2 * prm->max_psize
					/ runtime->hw.periods_min;
	runtime->hw.rate_min = prm->srate;
	runtime->hw.rate_max = runtime->hw.rate_min;
	runtime->hw.channels_max = runtime->hw.channels_min;

	snd_pcm_hw_constraint_integer(runtime, SNDRV_PCM_HW_PARAM_PERIODS);

	/* Initialize static sysfs attributes (format and channels are fixed in UAC2 config) */
	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		uac->current_format = c_ssize;
		uac->current_channels = num_channels(c_chmask);
	} else {
		uac->current_format = p_ssize;
		uac->current_channels = num_channels(p_chmask);
	}

	return 0;
}

/* ALSA cries without these function pointers */
static int uac_pcm_null(struct snd_pcm_substream *substream)
{
	return 0;
}

static const struct snd_pcm_ops uac_pcm_ops = {
	.open = uac_pcm_open,
	.close = uac_pcm_null,
	.trigger = uac_pcm_trigger,
	.pointer = uac_pcm_pointer,
	.prepare = uac_pcm_null,
};

static inline void free_ep(struct uac_rtd_params *prm, struct usb_ep *ep)
{
	struct snd_uac_chip *uac = prm->uac;
	struct g_audio *audio_dev;
	struct uac_params *params;
	int i;

	if (!prm->ep_enabled)
		return;

	audio_dev = uac->audio_dev;
	params = &audio_dev->params;

	for (i = 0; i < params->req_number; i++) {
		if (prm->reqs[i]) {
			if (usb_ep_dequeue(ep, prm->reqs[i]))
				usb_ep_free_request(ep, prm->reqs[i]);
			/*
			 * If usb_ep_dequeue() cannot successfully dequeue the
			 * request, the request will be freed by the completion
			 * callback.
			 */

			prm->reqs[i] = NULL;
		}
	}

	prm->ep_enabled = false;

	if (usb_ep_disable(ep))
		dev_err(uac->card->dev, "%s:%d Error!\n", __func__, __LINE__);
}

static inline void free_ep_fback(struct uac_rtd_params *prm, struct usb_ep *ep)
{
	struct snd_uac_chip *uac = prm->uac;

	if (!prm->fb_ep_enabled)
		return;

	if (prm->req_fback) {
		if (usb_ep_dequeue(ep, prm->req_fback)) {
			kfree(prm->req_fback->buf);
			usb_ep_free_request(ep, prm->req_fback);
		}
		prm->req_fback = NULL;
	}

	prm->fb_ep_enabled = false;

	if (usb_ep_disable(ep))
		dev_err(uac->card->dev, "%s:%d Error!\n", __func__, __LINE__);
}

static void set_active(struct uac_rtd_params *prm, bool active)
{
	// notifying through the Rate ctrl
	unsigned long flags;

	spin_lock_irqsave(&prm->lock, flags);
	if (prm->active != active) {
		prm->active = active;
		snd_ctl_notify(prm->uac->card, SNDRV_CTL_EVENT_MASK_VALUE,
				&prm->snd_kctl_rate_id);
	}
	spin_unlock_irqrestore(&prm->lock, flags);
}

int u_audio_set_capture_srate(struct g_audio *audio_dev, int srate)
{
	struct uac_params *params = &audio_dev->params;
	struct snd_uac_chip *uac = audio_dev->uac;
	struct uac_rtd_params *prm;
	int i;
	unsigned long flags;
	int old_rate;
	int actual_rate;  /* Rate to expose in sysfs (native DSD or PCM) */

	dev_info(&audio_dev->gadget->dev, "HOST requested rate: %d Hz\n", srate);

	/* DSD mode detection: convert PCM-equivalent rates to native DSD
	 * ONLY when Alt Setting 2 (DSD mode) is active.
	 * Alt Setting 1 (PCM) passes rates unchanged.
	 */
	actual_rate = srate;

	if (audio_dev->as_out_alt == 2 &&
	    (srate == 88200 || srate == 176400 || srate == 352800 || srate == 705600)) {
		/* Alt Setting 2 (native DSD): LRCK rate × 32 = DSD bit rate.
		 * c_srates lists only LRCK rates — no need to verify bit rate there. */
		actual_rate = srate * 32;
		dev_info(&audio_dev->gadget->dev,
			"DSD mode (Alt 2): PCM-equivalent %d Hz → Native DSD %d Hz\n",
			srate, actual_rate);
	}

	prm = &uac->c_prm;
	for (i = 0; i < UAC_MAX_RATES; i++) {
		if (params->c_srates[i] == srate) {
			spin_lock_irqsave(&prm->lock, flags);
			prm->srate = srate;
			audio_dev->usb_state[SET_SAMPLE_RATE_OUT] = true;
			schedule_work(&audio_dev->work);
			spin_unlock_irqrestore(&prm->lock, flags);

			/* Update sysfs rate attribute with native rate (DSD or PCM) */
			old_rate = uac->current_rate;
			uac->current_rate = actual_rate;

			if (uac->kobj && old_rate != actual_rate) {
				sysfs_notify(uac->kobj, NULL, "rate");
				kobject_uevent(uac->kobj, KOBJ_CHANGE);
			}

			dev_info(&audio_dev->gadget->dev, "Rate %d Hz ACCEPTED, sysfs shows %d Hz\n",
				srate, actual_rate);
			return 0;
		}
		if (params->c_srates[i] == 0)
			break;
	}

	dev_warn(&audio_dev->gadget->dev, "Rate %d Hz NOT FOUND in c_srates list, rejecting!\n", srate);
	return -EINVAL;
}
EXPORT_SYMBOL_GPL(u_audio_set_capture_srate);

/* Re-check DSD rate after Alt Setting 2 is activated.
 * HOST sends SET_CUR rate before SET_INTERFACE Alt=2, so the initial
 * rate conversion (LRCK→DSD bit rate) is skipped because as_out_alt
 * is still 0/1 at that point. Call this after setting as_out_alt=2. */
void u_audio_recheck_dsd_rate(struct g_audio *audio_dev)
{
	struct snd_uac_chip *uac = audio_dev->uac;
	struct uac_rtd_params *prm = &uac->c_prm;
	int srate = prm->srate;
	int actual_rate = srate;
	int old_rate;

	if (srate == 88200 || srate == 176400 ||
	    srate == 352800 || srate == 705600) {
		actual_rate = srate * 32;
		dev_info(&audio_dev->gadget->dev,
			"DSD recheck (Alt 2 active): %d Hz → %d Hz\n",
			srate, actual_rate);
	}

	old_rate = uac->current_rate;
	if (old_rate != actual_rate) {
		uac->current_rate = actual_rate;
		if (uac->kobj) {
			sysfs_notify(uac->kobj, NULL, "rate");
			kobject_uevent(uac->kobj, KOBJ_CHANGE);
		}
	}
}
EXPORT_SYMBOL_GPL(u_audio_recheck_dsd_rate);

int u_audio_get_capture_srate(struct g_audio *audio_dev, u32 *val)
{
	struct snd_uac_chip *uac = audio_dev->uac;
	struct uac_rtd_params *prm;
	unsigned long flags;

	prm = &uac->c_prm;
	spin_lock_irqsave(&prm->lock, flags);
	*val = prm->srate;
	spin_unlock_irqrestore(&prm->lock, flags);
	return 0;
}
EXPORT_SYMBOL_GPL(u_audio_get_capture_srate);

int u_audio_set_playback_srate(struct g_audio *audio_dev, int srate)
{
	struct uac_params *params = &audio_dev->params;
	struct snd_uac_chip *uac = audio_dev->uac;
	struct uac_rtd_params *prm;
	int i;
	unsigned long flags;
	int old_rate;

	dev_dbg(&audio_dev->gadget->dev, "%s: srate %d\n", __func__, srate);
	prm = &uac->p_prm;
	for (i = 0; i < UAC_MAX_RATES; i++) {
		if (params->p_srates[i] == srate) {
			spin_lock_irqsave(&prm->lock, flags);
			prm->srate = srate;
			audio_dev->usb_state[SET_SAMPLE_RATE_IN] = true;
			schedule_work(&audio_dev->work);
			spin_unlock_irqrestore(&prm->lock, flags);

			/* Update sysfs rate attribute */
			old_rate = uac->current_rate;
			uac->current_rate = srate;

			if (uac->kobj && old_rate != srate) {
				sysfs_notify(uac->kobj, NULL, "rate");
				kobject_uevent(uac->kobj, KOBJ_CHANGE);
			}

			return 0;
		}
		if (params->p_srates[i] == 0)
			break;
	}

	return -EINVAL;
}
EXPORT_SYMBOL_GPL(u_audio_set_playback_srate);

int u_audio_get_playback_srate(struct g_audio *audio_dev, u32 *val)
{
	struct snd_uac_chip *uac = audio_dev->uac;
	struct uac_rtd_params *prm;
	unsigned long flags;

	prm = &uac->p_prm;
	spin_lock_irqsave(&prm->lock, flags);
	*val = prm->srate;
	spin_unlock_irqrestore(&prm->lock, flags);
	return 0;
}
EXPORT_SYMBOL_GPL(u_audio_get_playback_srate);

int u_audio_start_capture(struct g_audio *audio_dev)
{
	struct snd_uac_chip *uac = audio_dev->uac;
	struct usb_gadget *gadget = audio_dev->gadget;
	struct device *dev = &gadget->dev;
	struct usb_request *req, *req_fback;
	struct usb_ep *ep, *ep_fback;
	struct uac_rtd_params *prm;
	struct uac_params *params = &audio_dev->params;
	int req_len, i, ret;
	unsigned int maxpacket;

	/*
	 * For better compatibility on some PC Hosts which
	 * failed to send SetInterface(AltSet=0) to stop
	 * capture last time. It needs to stop capture
	 * prior to start capture next time.
	 */
	if (audio_dev->stream_state[STATE_OUT])
		u_audio_stop_capture(audio_dev);

	audio_dev->usb_state[SET_INTERFACE_OUT] = true;
	audio_dev->stream_state[STATE_OUT] = true;
	schedule_work(&audio_dev->work);

	prm = &uac->c_prm;
	dev_dbg(dev, "start capture with rate %d\n", prm->srate);
	ep = audio_dev->out_ep;
	ret = config_ep_by_speed(gadget, &audio_dev->func, ep);
	if (ret < 0) {
		dev_err(dev, "config_ep_by_speed for out_ep failed (%d)\n", ret);
		return ret;
	}

	req_len = ep->maxpacket;
	maxpacket = ALIGN((unsigned int)ep->maxpacket, cache_line_size());

	prm->ep_enabled = true;
	ret = usb_ep_enable(ep);
	if (ret < 0) {
		dev_err(dev, "usb_ep_enable failed for out_ep (%d)\n", ret);
		return ret;
	}

	for (i = 0; i < params->req_number; i++) {
		if (!prm->reqs[i]) {
			req = usb_ep_alloc_request(ep, GFP_ATOMIC);
			if (req == NULL)
				return -ENOMEM;

			prm->reqs[i] = req;

			req->zero = 0;
			req->context = prm;
			req->length = req_len;
			req->complete = u_audio_iso_complete;
			req->buf = prm->rbuf + i * maxpacket;
		}

		if (usb_ep_queue(ep, prm->reqs[i], GFP_ATOMIC))
			dev_err(dev, "%s:%d Error!\n", __func__, __LINE__);
	}

	set_active(&uac->c_prm, true);

	/* Update sysfs rate (format and channels are static - set in uac_pcm_open) */
	/* DSD mode: convert PCM-equivalent LRCK rate to native DSD bit rate.
	 * c_srates lists only LRCK rates — no c_srates check needed. */
	{
		int actual_rate = prm->srate;
		if (audio_dev->as_out_alt == 2 &&
		    (prm->srate == 88200 || prm->srate == 176400 ||
		     prm->srate == 352800 || prm->srate == 705600))
			actual_rate = prm->srate * 32;
		uac->current_rate = actual_rate;
	}

	if (uac->kobj) {
		sysfs_notify(uac->kobj, NULL, "rate");
		kobject_uevent(uac->kobj, KOBJ_CHANGE);
	}

	ep_fback = audio_dev->in_ep_fback;
	if (!ep_fback)
		return 0;

	/* Setup feedback endpoint */
	ret = config_ep_by_speed(gadget, &audio_dev->func, ep_fback);
	if (ret < 0) {
		dev_err(dev, "config_ep_by_speed in_ep_fback failed (%d)\n", ret);
		return ret; // TODO: Clean up out_ep
	}

	prm->fb_ep_enabled = true;
	ret = usb_ep_enable(ep_fback);
	if (ret < 0) {
		dev_err(dev, "usb_ep_enable failed for in_ep_fback (%d)\n", ret);
		return ret; // TODO: Clean up out_ep
	}
	req_len = ep_fback->maxpacket;

	req_fback = usb_ep_alloc_request(ep_fback, GFP_ATOMIC);
	if (req_fback == NULL)
		return -ENOMEM;

	prm->req_fback = req_fback;
	req_fback->zero = 0;
	req_fback->context = prm;
	req_fback->length = req_len;
	req_fback->complete = u_audio_iso_fback_complete;

	req_fback->buf = kzalloc(req_len, GFP_ATOMIC);
	if (!req_fback->buf)
		return -ENOMEM;

	/*
	 * Configure the feedback endpoint's reported frequency.
	 * Use seed from sysfs if available, otherwise nominal.
	 */
	prm->pitch = 1000000;
	prm->fb_integral = 0;
	prm->fb_err_smooth = 0;
	prm->fb_freeze = 0;
	prm->fb_freeze_grace = 0;
	u_audio_set_fback_frequency(audio_dev->gadget->speed, ep,
				    prm->srate, prm->pitch,
				    req_fback->buf);

	if (usb_ep_queue(ep_fback, req_fback, GFP_ATOMIC))
		dev_err(dev, "%s:%d Error!\n", __func__, __LINE__);

	return 0;
}
EXPORT_SYMBOL_GPL(u_audio_start_capture);

void u_audio_stop_capture(struct g_audio *audio_dev)
{
	struct snd_uac_chip *uac = audio_dev->uac;

	set_active(&uac->c_prm, false);
	if (audio_dev->in_ep_fback)
		free_ep_fback(&uac->c_prm, audio_dev->in_ep_fback);
	free_ep(&uac->c_prm, audio_dev->out_ep);

	audio_dev->usb_state[SET_INTERFACE_OUT] = true;
	audio_dev->stream_state[STATE_OUT] = false;
	schedule_work(&audio_dev->work);
}
EXPORT_SYMBOL_GPL(u_audio_stop_capture);

int u_audio_start_playback(struct g_audio *audio_dev)
{
	struct snd_uac_chip *uac = audio_dev->uac;
	struct usb_gadget *gadget = audio_dev->gadget;
	struct device *dev = &gadget->dev;
	struct usb_request *req;
	struct usb_ep *ep;
	struct uac_rtd_params *prm;
	struct uac_params *params = &audio_dev->params;
	unsigned int factor;
	const struct usb_endpoint_descriptor *ep_desc;
	int req_len, i, ret;
	unsigned int p_pktsize, maxpacket;

	/*
	 * For better compatibility on some PC Hosts which
	 * failed to send SetInterface(AltSet=0) to stop
	 * playback last time. It needs to stop playback
	 * prior to start playback next time.
	 */
	if (audio_dev->stream_state[STATE_IN])
		u_audio_stop_playback(audio_dev);

	audio_dev->usb_state[SET_INTERFACE_IN] = true;
	audio_dev->stream_state[STATE_IN] = true;
	schedule_work(&audio_dev->work);

	prm = &uac->p_prm;
	dev_dbg(dev, "start playback with rate %d\n", prm->srate);
	ep = audio_dev->in_ep;
	ret = config_ep_by_speed(gadget, &audio_dev->func, ep);
	if (ret < 0) {
		dev_err(dev, "config_ep_by_speed for in_ep failed (%d)\n", ret);
		return ret;
	}

	ep_desc = ep->desc;
	maxpacket = ALIGN((unsigned int)ep->maxpacket, cache_line_size());
	prm->pitch = 1000000;
	prm->fb_integral = 0;
	prm->fb_err_smooth = 0;

	/* pre-calculate the playback endpoint's interval */
	if (gadget->speed == USB_SPEED_FULL)
		factor = 1000;
	else
		factor = 8000;

	/* pre-compute some values for iso_complete() */
	uac->p_framesize = params->p_ssize *
			    num_channels(params->p_chmask);
	uac->p_interval = factor / (1 << (ep_desc->bInterval - 1));
	p_pktsize = min_t(unsigned int,
				uac->p_framesize *
					(prm->srate / uac->p_interval),
				ep->maxpacket);

	req_len = p_pktsize;
	uac->p_residue_mil = 0;

	prm->ep_enabled = true;
	ret = usb_ep_enable(ep);
	if (ret < 0) {
		dev_err(dev, "usb_ep_enable failed for in_ep (%d)\n", ret);
		return ret;
	}

	for (i = 0; i < params->req_number; i++) {
		if (!prm->reqs[i]) {
			req = usb_ep_alloc_request(ep, GFP_ATOMIC);
			if (req == NULL)
				return -ENOMEM;

			prm->reqs[i] = req;

			req->zero = 0;
			req->context = prm;
			req->length = req_len;
			req->complete = u_audio_iso_complete;
			req->buf = prm->rbuf + i * maxpacket;
		}

		if (usb_ep_queue(ep, prm->reqs[i], GFP_ATOMIC))
			dev_err(dev, "%s:%d Error!\n", __func__, __LINE__);
	}

	set_active(&uac->p_prm, true);

	/* Update sysfs rate. DSD mode: LRCK × 32 = DSD bit rate (no c_srates check). */
	{
		int actual_rate = prm->srate;
		if (audio_dev->as_out_alt == 2 &&
		    (prm->srate == 88200 || prm->srate == 176400 ||
		     prm->srate == 352800 || prm->srate == 705600))
			actual_rate = prm->srate * 32;
		uac->current_rate = actual_rate;
	}

	if (uac->kobj) {
		sysfs_notify(uac->kobj, NULL, "rate");
		kobject_uevent(uac->kobj, KOBJ_CHANGE);
	}

	return 0;
}
EXPORT_SYMBOL_GPL(u_audio_start_playback);

void u_audio_stop_playback(struct g_audio *audio_dev)
{
	struct snd_uac_chip *uac = audio_dev->uac;

	set_active(&uac->p_prm, false);
	free_ep(&uac->p_prm, audio_dev->in_ep);

	audio_dev->usb_state[SET_INTERFACE_IN] = true;
	audio_dev->stream_state[STATE_IN] = false;
	schedule_work(&audio_dev->work);
}
EXPORT_SYMBOL_GPL(u_audio_stop_playback);

void u_audio_suspend(struct g_audio *audio_dev)
{
	struct snd_uac_chip *uac = audio_dev->uac;

	set_active(&uac->p_prm, false);
	set_active(&uac->c_prm, false);
}
EXPORT_SYMBOL_GPL(u_audio_suspend);

int u_audio_get_volume(struct g_audio *audio_dev, int playback, s16 *val)
{
	struct snd_uac_chip *uac = audio_dev->uac;
	struct uac_rtd_params *prm;
	unsigned long flags;

	if (playback)
		prm = &uac->p_prm;
	else
		prm = &uac->c_prm;

	spin_lock_irqsave(&prm->lock, flags);
	*val = prm->volume;
	spin_unlock_irqrestore(&prm->lock, flags);

	return 0;
}
EXPORT_SYMBOL_GPL(u_audio_get_volume);

int u_audio_set_volume(struct g_audio *audio_dev, int playback, s16 val)
{
	struct snd_uac_chip *uac = audio_dev->uac;
	struct uac_rtd_params *prm;
	unsigned long flags;
	int change = 0;

	if (playback)
		prm = &uac->p_prm;
	else
		prm = &uac->c_prm;

	spin_lock_irqsave(&prm->lock, flags);
	val = clamp(val, prm->volume_min, prm->volume_max);
	if (prm->volume != val) {
		prm->volume = val;
		change = 1;
	}
	spin_unlock_irqrestore(&prm->lock, flags);

	if (change) {
		if (playback)
			audio_dev->usb_state[SET_VOLUME_IN] = true;
		else
			audio_dev->usb_state[SET_VOLUME_OUT] = true;
		schedule_work(&audio_dev->work);

		snd_ctl_notify(uac->card, SNDRV_CTL_EVENT_MASK_VALUE,
				&prm->snd_kctl_volume_id);
	}

	return 0;
}
EXPORT_SYMBOL_GPL(u_audio_set_volume);

int u_audio_get_mute(struct g_audio *audio_dev, int playback, int *val)
{
	struct snd_uac_chip *uac = audio_dev->uac;
	struct uac_rtd_params *prm;
	unsigned long flags;

	if (playback)
		prm = &uac->p_prm;
	else
		prm = &uac->c_prm;

	spin_lock_irqsave(&prm->lock, flags);
	*val = prm->mute;
	spin_unlock_irqrestore(&prm->lock, flags);

	return 0;
}
EXPORT_SYMBOL_GPL(u_audio_get_mute);

int u_audio_set_mute(struct g_audio *audio_dev, int playback, int val)
{
	struct snd_uac_chip *uac = audio_dev->uac;
	struct uac_rtd_params *prm;
	unsigned long flags;
	int change = 0;
	int mute;

	if (playback)
		prm = &uac->p_prm;
	else
		prm = &uac->c_prm;

	mute = val ? 1 : 0;

	spin_lock_irqsave(&prm->lock, flags);
	if (prm->mute != mute) {
		prm->mute = mute;
		change = 1;
	}
	spin_unlock_irqrestore(&prm->lock, flags);

	if (change) {
		if (playback)
			audio_dev->usb_state[SET_MUTE_IN] = true;
		else
			audio_dev->usb_state[SET_MUTE_OUT] = true;
		schedule_work(&audio_dev->work);

		snd_ctl_notify(uac->card, SNDRV_CTL_EVENT_MASK_VALUE,
			       &prm->snd_kctl_mute_id);
	}

	return 0;
}
EXPORT_SYMBOL_GPL(u_audio_set_mute);


static int u_audio_pitch_info(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_info *uinfo)
{
	struct uac_rtd_params *prm = snd_kcontrol_chip(kcontrol);
	struct snd_uac_chip *uac = prm->uac;
	struct g_audio *audio_dev = uac->audio_dev;
	struct uac_params *params = &audio_dev->params;
	unsigned int pitch_min, pitch_max;

	pitch_min = (1000 - FBACK_SLOW_MAX) * 1000;
	pitch_max = (1000 + params->fb_max) * 1000;

	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = pitch_min;
	uinfo->value.integer.max = pitch_max;
	uinfo->value.integer.step = 1;
	return 0;
}

static int u_audio_pitch_get(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	struct uac_rtd_params *prm = snd_kcontrol_chip(kcontrol);

	ucontrol->value.integer.value[0] = prm->pitch;

	return 0;
}

static int u_audio_pitch_put(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	struct uac_rtd_params *prm = snd_kcontrol_chip(kcontrol);
	struct snd_uac_chip *uac = prm->uac;
	struct g_audio *audio_dev = uac->audio_dev;
	struct uac_params *params = &audio_dev->params;
	unsigned int val;
	unsigned int pitch_min, pitch_max;
	int change = 0;

	pitch_min = (1000 - FBACK_SLOW_MAX) * 1000;
	pitch_max = (1000 + params->fb_max) * 1000;

	val = ucontrol->value.integer.value[0];

	if (val < pitch_min)
		val = pitch_min;
	if (val > pitch_max)
		val = pitch_max;

	if (prm->pitch != val) {
		prm->pitch = val;
		change = 1;
	}

	return change;
}

static int u_audio_mute_info(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_BOOLEAN;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 1;
	uinfo->value.integer.step = 1;

	return 0;
}

static int u_audio_mute_get(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	struct uac_rtd_params *prm = snd_kcontrol_chip(kcontrol);
	unsigned long flags;

	spin_lock_irqsave(&prm->lock, flags);
	ucontrol->value.integer.value[0] = !prm->mute;
	spin_unlock_irqrestore(&prm->lock, flags);

	return 0;
}

static int u_audio_mute_put(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	struct uac_rtd_params *prm = snd_kcontrol_chip(kcontrol);
	struct snd_uac_chip *uac = prm->uac;
	struct g_audio *audio_dev = uac->audio_dev;
	unsigned int val;
	unsigned long flags;
	int change = 0;

	val = !ucontrol->value.integer.value[0];

	spin_lock_irqsave(&prm->lock, flags);
	if (val != prm->mute) {
		prm->mute = val;
		change = 1;
	}
	spin_unlock_irqrestore(&prm->lock, flags);

	if (change && audio_dev->notify)
		audio_dev->notify(audio_dev, prm->fu_id, UAC_FU_MUTE);

	return change;
}

/*
 * TLV callback for mixer volume controls
 */
static int u_audio_volume_tlv(struct snd_kcontrol *kcontrol, int op_flag,
			 unsigned int size, unsigned int __user *_tlv)
{
	struct uac_rtd_params *prm = snd_kcontrol_chip(kcontrol);
	DECLARE_TLV_DB_MINMAX(scale, 0, 0);

	if (size < sizeof(scale))
		return -ENOMEM;

	/* UAC volume resolution is 1/256 dB, TLV is 1/100 dB */
	scale[2] = (prm->volume_min * 100) / 256;
	scale[3] = (prm->volume_max * 100) / 256;
	if (copy_to_user(_tlv, scale, sizeof(scale)))
		return -EFAULT;

	return 0;
}

static int u_audio_volume_info(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_info *uinfo)
{
	struct uac_rtd_params *prm = snd_kcontrol_chip(kcontrol);

	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max =
		(prm->volume_max - prm->volume_min + prm->volume_res - 1)
		/ prm->volume_res;
	uinfo->value.integer.step = 1;

	return 0;
}

static int u_audio_volume_get(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	struct uac_rtd_params *prm = snd_kcontrol_chip(kcontrol);
	unsigned long flags;

	spin_lock_irqsave(&prm->lock, flags);
	ucontrol->value.integer.value[0] =
			(prm->volume - prm->volume_min) / prm->volume_res;
	spin_unlock_irqrestore(&prm->lock, flags);

	return 0;
}

static int u_audio_volume_put(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	struct uac_rtd_params *prm = snd_kcontrol_chip(kcontrol);
	struct snd_uac_chip *uac = prm->uac;
	struct g_audio *audio_dev = uac->audio_dev;
	unsigned int val;
	s16 volume;
	unsigned long flags;
	int change = 0;

	val = ucontrol->value.integer.value[0];

	spin_lock_irqsave(&prm->lock, flags);
	volume = (val * prm->volume_res) + prm->volume_min;
	volume = clamp(volume, prm->volume_min, prm->volume_max);
	if (volume != prm->volume) {
		prm->volume = volume;
		change = 1;
	}
	spin_unlock_irqrestore(&prm->lock, flags);

	if (change && audio_dev->notify)
		audio_dev->notify(audio_dev, prm->fu_id, UAC_FU_VOLUME);

	return change;
}

static int get_max_srate(const int *srates)
{
	int i, max_srate = 0;

	for (i = 0; i < UAC_MAX_RATES; i++) {
		if (srates[i] == 0)
			break;
		if (srates[i] > max_srate)
			max_srate = srates[i];
	}
	return max_srate;
}

static int get_min_srate(const int *srates)
{
	int i, min_srate = INT_MAX;

	for (i = 0; i < UAC_MAX_RATES; i++) {
		if (srates[i] == 0)
			break;
		if (srates[i] < min_srate)
			min_srate = srates[i];
	}
	return min_srate;
}

static int u_audio_rate_info(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_info *uinfo)
{
	const int *srates;
	struct uac_rtd_params *prm = snd_kcontrol_chip(kcontrol);
	struct snd_uac_chip *uac = prm->uac;
	struct g_audio *audio_dev = uac->audio_dev;
	struct uac_params *params = &audio_dev->params;

	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;

	if (prm == &uac->c_prm)
		srates = params->c_srates;
	else
		srates = params->p_srates;
	uinfo->value.integer.min = get_min_srate(srates);
	uinfo->value.integer.max = get_max_srate(srates);
	return 0;
}

static int u_audio_rate_get(struct snd_kcontrol *kcontrol,
						 struct snd_ctl_elem_value *ucontrol)
{
	struct uac_rtd_params *prm = snd_kcontrol_chip(kcontrol);
	unsigned long flags;

	spin_lock_irqsave(&prm->lock, flags);
	if (prm->active)
		ucontrol->value.integer.value[0] = prm->srate;
	else
		/* not active: reporting zero rate */
		ucontrol->value.integer.value[0] = 0;
	spin_unlock_irqrestore(&prm->lock, flags);
	return 0;
}

static struct snd_kcontrol_new u_audio_controls[]  = {
  [UAC_FBACK_CTRL] {
    .iface =        SNDRV_CTL_ELEM_IFACE_PCM,
    .name =         "Capture Pitch 1000000",
    .info =         u_audio_pitch_info,
    .get =          u_audio_pitch_get,
    .put =          u_audio_pitch_put,
  },
	[UAC_P_PITCH_CTRL] {
		.iface =        SNDRV_CTL_ELEM_IFACE_PCM,
		.name =         "Playback Pitch 1000000",
		.info =         u_audio_pitch_info,
		.get =          u_audio_pitch_get,
		.put =          u_audio_pitch_put,
	},
  [UAC_MUTE_CTRL] {
		.iface =	SNDRV_CTL_ELEM_IFACE_MIXER,
		.name =		"", /* will be filled later */
		.info =		u_audio_mute_info,
		.get =		u_audio_mute_get,
		.put =		u_audio_mute_put,
	},
	[UAC_VOLUME_CTRL] {
		.iface =	SNDRV_CTL_ELEM_IFACE_MIXER,
		.name =		"", /* will be filled later */
		.info =		u_audio_volume_info,
		.get =		u_audio_volume_get,
		.put =		u_audio_volume_put,
	},
	[UAC_RATE_CTRL] {
		.iface =	SNDRV_CTL_ELEM_IFACE_PCM,
		.name =		"", /* will be filled later */
		.access =	SNDRV_CTL_ELEM_ACCESS_READ | SNDRV_CTL_ELEM_ACCESS_VOLATILE,
		.info =		u_audio_rate_info,
		.get =		u_audio_rate_get,
	},
};

static void g_audio_work(struct work_struct *data)
{
	struct g_audio *audio = container_of(data, struct g_audio, work);
	struct usb_gadget *gadget = audio->gadget;
	struct snd_uac_chip *uac = audio->uac;
	struct uac_rtd_params *prm;
	struct device *dev = &gadget->dev;
	char *uac_event[4]  = { NULL, NULL, NULL, NULL };
	char str[19];
	int i;

	for (i = 0; i < SET_USB_STATE_MAX; i++) {
		if (!audio->usb_state[i])
			continue;

		switch (i) {
		case SET_INTERFACE_OUT:
			uac_event[0] = "USB_STATE=SET_INTERFACE";
			uac_event[1] = "STREAM_DIRECTION=OUT";
			uac_event[2] = audio->stream_state[STATE_OUT] ?
				       "STREAM_STATE=ON" : "STREAM_STATE=OFF";
			break;
		case SET_INTERFACE_IN:
			uac_event[0] = "USB_STATE=SET_INTERFACE";
			uac_event[1] = "STREAM_DIRECTION=IN";
			uac_event[2] = audio->stream_state[STATE_IN] ?
				       "STREAM_STATE=ON" : "STREAM_STATE=OFF";
			break;
		case SET_SAMPLE_RATE_OUT:
			uac_event[0] = "USB_STATE=SET_SAMPLE_RATE";
			uac_event[1] = "STREAM_DIRECTION=OUT";
			prm = &uac->c_prm;
			snprintf(str, sizeof(str), "SAMPLE_RATE=%d",
				 prm->srate);
			uac_event[2] = str;
			break;
		case SET_SAMPLE_RATE_IN:
			uac_event[0] = "USB_STATE=SET_SAMPLE_RATE";
			uac_event[1] = "STREAM_DIRECTION=IN";
			prm = &uac->p_prm;
			snprintf(str, sizeof(str), "SAMPLE_RATE=%d",
				 prm->srate);
			uac_event[2] = str;
			break;
		case SET_MUTE_OUT:
			uac_event[0] = "USB_STATE=SET_MUTE";
			uac_event[1] = "STREAM_DIRECTION=OUT";
			prm = &uac->c_prm;
			snprintf(str, sizeof(str), "MUTE=%d", prm->mute);
			uac_event[2] = str;
			break;
		case SET_MUTE_IN:
			uac_event[0] = "USB_STATE=SET_MUTE";
			uac_event[1] = "STREAM_DIRECTION=IN";
			prm = &uac->p_prm;
			snprintf(str, sizeof(str), "MUTE=%d", prm->mute);
			uac_event[2] = str;
			break;
		case SET_VOLUME_OUT:
			uac_event[0] = "USB_STATE=SET_VOLUME";
			uac_event[1] = "STREAM_DIRECTION=OUT";
			prm = &uac->c_prm;
			snprintf(str, sizeof(str), "VOLUME=0x%hx", prm->volume);
			uac_event[2] = str;
			break;
		case SET_VOLUME_IN:
			uac_event[0] = "USB_STATE=SET_VOLUME";
			uac_event[1] = "STREAM_DIRECTION=IN";
			prm = &uac->p_prm;
			snprintf(str, sizeof(str), "VOLUME=0x%hx", prm->volume);
			uac_event[2] = str;
			break;
		case SET_AUDIO_CLK:
			uac_event[0] = "USB_STATE=SET_AUDIO_CLK";
			snprintf(str, sizeof(str), "PPM=%d", audio->params.ppm);
			uac_event[1] = str;
			break;
		default:
			break;
		}

		audio->usb_state[i] = false;
		kobject_uevent_env(&audio->device->kobj, KOBJ_CHANGE,
				   uac_event);
		dev_dbg(dev, "%s: sent uac uevent %s %s %s\n", __func__,
			uac_event[0], uac_event[1], uac_event[2]);
	}
}

static void ppm_calculate_work(struct work_struct *data)
{
	struct g_audio *g_audio = container_of(data, struct g_audio,
					       ppm_work.work);
	struct usb_gadget *gadget = g_audio->gadget;
	uint32_t frame_number, fn_msec, clk_msec;
	struct frame_number_data *fn = g_audio->fn;
	uint64_t time_now, time_msec_tmp;
	int32_t ppm;
	static int32_t ppms[CLK_PPM_GROUP_SIZE];
	static int32_t ppm_sum;
	int32_t cnt = fn->second % CLK_PPM_GROUP_SIZE;

	time_now = ktime_get_raw();
	frame_number = gadget->ops->get_frame(gadget);

	if (g_audio->fn->time_last &&
	    time_now - g_audio->fn->time_last > 1500000000ULL)
		dev_warn(g_audio->device, "PPM work scheduled too slow!\n");

	g_audio->fn->time_last = time_now;

	/*
	 * If usb is disconnected, the controller will not receive the
	 * SoF signal and frame number will be invalid. Because we can't
	 * get accurate time of disconnect and whether the gadget will be
	 * plugged into the same host next time or not. We must clear all
	 * statistics.
	 */
	if (gadget->state != USB_STATE_CONFIGURED) {
		memset(g_audio->fn, 0, sizeof(*g_audio->fn));
		dev_dbg(g_audio->device, "Disconnect. frame number is cleared\n");
		goto out;
	}

	/* Fist statistic to record begin frame number and system time */
	if (!g_audio->fn->second++) {
		g_audio->fn->time_begin = g_audio->fn->time_last;
		g_audio->fn->fn_begin = frame_number;
		g_audio->fn->fn_last = frame_number;
		goto out;
	}

	/*
	 * For DWC3 Controller, only 13 bits is used to store frame(micro)
	 * number. In other words, the frame number will overflow at most
	 * 2.047 seconds. We add another registor fn_overflow the record
	 * total frame number.
	 */
	if (frame_number <= g_audio->fn->fn_last)
		g_audio->fn->fn_overflow++;
	g_audio->fn->fn_last = frame_number;

	if (!g_audio->fn->fn_overflow)
		goto out;

	/* The lower 3 bits represent micro number frame, we don't need it */
	fn_msec = (((fn->fn_overflow - 1) << 14) +
		   (BIT(14) + fn->fn_last - fn->fn_begin) + BIT(2)) >> 3;
	time_msec_tmp = fn->time_last - fn->time_begin + 500000ULL;
	do_div(time_msec_tmp, 1000000U);
	clk_msec = (uint32_t)time_msec_tmp;

	/*
	 * According to the definition of ppm:
	 *   host_clk = (1 + ppm / 1000000) * gadget_clk
	 * we can get:
	 *   ppm = (host_clk - gadget_clk) * 1000000 / gadget_clk
	 */
	ppm = (fn_msec > clk_msec) ?
	      (fn_msec - clk_msec) * 1000000L / clk_msec :
	      -((clk_msec - fn_msec) * 1000000L / clk_msec);

	ppm_sum = ppm_sum - ppms[cnt] + ppm;
	ppms[cnt] = ppm;

	dev_dbg(g_audio->device,
		"frame %u msec %u ppm_calc %d ppm_avage(%d) %d\n",
		fn_msec, clk_msec, ppm, CLK_PPM_GROUP_SIZE,
		ppm_sum / CLK_PPM_GROUP_SIZE);

	/*
	 * We calculate the average of ppm over a period of time. If the
	 * latest frame number is too far from the average, no event will
	 * be sent.
	 */
	if (abs(ppm_sum / CLK_PPM_GROUP_SIZE - ppm) < 3) {
		ppm = ppm_sum > 0 ?
		      (ppm_sum + CLK_PPM_GROUP_SIZE / 2) / CLK_PPM_GROUP_SIZE :
		      (ppm_sum - CLK_PPM_GROUP_SIZE / 2) / CLK_PPM_GROUP_SIZE;
		if (ppm != g_audio->params.ppm) {
			g_audio->params.ppm = ppm;
			g_audio->usb_state[SET_AUDIO_CLK] = true;
			schedule_work(&g_audio->work);
		}
	}

out:
	schedule_delayed_work(&g_audio->ppm_work, 1 * HZ);
}

/* Sysfs attributes for format change notification */
static ssize_t rate_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct snd_uac_chip *uac = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", uac->current_rate);
}

static ssize_t format_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct snd_uac_chip *uac = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", uac->current_format);
}

static ssize_t channels_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct snd_uac_chip *uac = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", uac->current_channels);
}

static ssize_t feedback_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct snd_uac_chip *uac = dev_get_drvdata(dev);
	return sprintf(buf, "%u\n", uac->c_prm.pitch);
}

static ssize_t feedback_store(struct device *dev, struct device_attribute *attr, 
			      const char *buf, size_t count)
{
	struct snd_uac_chip *uac = dev_get_drvdata(dev);
	unsigned int val;
	int ret;

	ret = kstrtouint(buf, 10, &val);
	if (ret)
		return ret;

	if (val < 500000 || val > 2000000)
		return -EINVAL;

	/* Direct pitch override. DigiFox: uac2_router steers the pitch itself from the
	 * fill of the I2S queue (the PI below sees only the capture buffer, which the
	 * router keeps near empty, and drifted the wrong way: the I2S queue drained to an
	 * underrun every ~12 min). A write holds the PI off for 16000 feedback cycles;
	 * the router writes twice a second, so the PI is back within seconds if it stops.
	 * Grace = the same span: the emergency unfreeze (capture fill below 1/64) would
	 * fire at once, the router keeps the capture buffer near empty on purpose. */
	spin_lock(&uac->c_prm.lock);
	uac->c_prm.pitch = val;
	uac->c_prm.fb_freeze = 16000;
	uac->c_prm.fb_freeze_grace = 16000;
	spin_unlock(&uac->c_prm.lock);

	return count;
}

static ssize_t fb_freeze_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct snd_uac_chip *uac = dev_get_drvdata(dev);
	return sprintf(buf, "%lu\n", uac->c_prm.fb_freeze);
}

static DEVICE_ATTR_RO(rate);
static DEVICE_ATTR_RO(format);
static DEVICE_ATTR_RO(channels);
static DEVICE_ATTR_RW(feedback);
static DEVICE_ATTR_RO(fb_freeze);

static struct attribute *uac_attrs[] = {
	&dev_attr_rate.attr,
	&dev_attr_format.attr,
	&dev_attr_channels.attr,
	&dev_attr_feedback.attr,
	&dev_attr_fb_freeze.attr,
	NULL,
};

static const struct attribute_group uac_attr_group = {
	.attrs = uac_attrs,
};

int g_audio_setup(struct g_audio *g_audio, const char *pcm_name,
					const char *card_name)
{
	struct snd_uac_chip *uac;
	struct snd_card *card;
	struct snd_pcm *pcm;
	struct snd_kcontrol *kctl;
	struct uac_params *params;
	int p_chmask, c_chmask;
	int i, err;
	unsigned int max_psize;

	if (!g_audio)
		return -EINVAL;

	uac = kzalloc(sizeof(*uac), GFP_KERNEL);
	if (!uac)
		return -ENOMEM;
	g_audio->uac = uac;
	uac->audio_dev = g_audio;

	params = &g_audio->params;
	p_chmask = params->p_chmask;
	c_chmask = params->c_chmask;

	g_audio->fn = kzalloc(sizeof(*g_audio->fn), GFP_KERNEL);
	if (!g_audio->fn) {
		err = -ENOMEM;
		goto fail;
	}

	if (c_chmask) {
		struct uac_rtd_params *prm = &uac->c_prm;

		spin_lock_init(&prm->lock);
		uac->c_prm.uac = uac;
		prm->max_psize = g_audio->out_ep_maxpsize;
		prm->srate = params->c_srates[0];

		prm->reqs = kcalloc(params->req_number,
				    sizeof(struct usb_request *),
				    GFP_KERNEL);
		if (!prm->reqs) {
			err = -ENOMEM;
			goto fail;
		}

		max_psize = ALIGN(prm->max_psize, cache_line_size());
		prm->rbuf = kcalloc(params->req_number, max_psize,
				GFP_KERNEL);
		if (!prm->rbuf) {
			prm->max_psize = 0;
			err = -ENOMEM;
			goto fail;
		}
	}

	if (p_chmask) {
		struct uac_rtd_params *prm = &uac->p_prm;

		spin_lock_init(&prm->lock);
		uac->p_prm.uac = uac;
		prm->max_psize = g_audio->in_ep_maxpsize;
		prm->srate = params->p_srates[0];

		prm->reqs = kcalloc(params->req_number,
				    sizeof(struct usb_request *),
				    GFP_KERNEL);
		if (!prm->reqs) {
			err = -ENOMEM;
			goto fail;
		}

		max_psize = ALIGN(prm->max_psize, cache_line_size());
		prm->rbuf = kcalloc(params->req_number, max_psize,
				GFP_KERNEL);
		if (!prm->rbuf) {
			prm->max_psize = 0;
			err = -ENOMEM;
			goto fail;
		}
	}

	/* Choose any slot, with no id */
	err = snd_card_new(&g_audio->gadget->dev,
			-1, NULL, THIS_MODULE, 0, &card);
	if (err < 0)
		goto fail;

	uac->card = card;

	/* Initialize current format parameters */
	uac->current_rate = params->c_srates[0];
	uac->current_format = params->c_ssize;
	uac->current_channels = num_channels(c_chmask);

	/*
	 * Create first PCM device
	 * Create a substream only for non-zero channel streams
	 */
	err = snd_pcm_new(uac->card, pcm_name, 0,
			       p_chmask ? 1 : 0, c_chmask ? 1 : 0, &pcm);
	if (err < 0)
		goto snd_fail;

	strscpy(pcm->name, pcm_name, sizeof(pcm->name));
	pcm->private_data = uac;
	uac->pcm = pcm;

	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_PLAYBACK, &uac_pcm_ops);
	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_CAPTURE, &uac_pcm_ops);

	/*
	 * Create mixer and controls
	 * Create only if it's required on USB side
	 */
	if ((c_chmask && g_audio->in_ep_fback)
			|| (p_chmask && params->p_fu.id)
			|| (c_chmask && params->c_fu.id))
		strscpy(card->mixername, card_name, sizeof(card->driver));

	/* PURECORE: Always create capture pitch control for feedback */
	if (c_chmask) {
		dev_info(&uac->audio_dev->gadget->dev, "Creating UAC_FBACK_CTRL: c_chmask=%d\n", c_chmask);
		
		kctl = snd_ctl_new1(&u_audio_controls[UAC_FBACK_CTRL],
				    &uac->c_prm);
		if (!kctl) {
			dev_err(&uac->audio_dev->gadget->dev, "Failed to create UAC_FBACK_CTRL control\n");
			/* Don't fail - continue without pitch control */
		} else {
			dev_info(&uac->audio_dev->gadget->dev, "Adding UAC_FBACK_CTRL to card %d, device %d\n", 
				 card->number, pcm->device);
			
			err = snd_ctl_add(card, kctl);
			if (err < 0) {
				dev_err(&uac->audio_dev->gadget->dev, "Failed to add UAC_FBACK_CTRL: %d\n", err);
				/* Don't fail - continue without pitch control */
			} else {
				dev_info(&uac->audio_dev->gadget->dev, "UAC_FBACK_CTRL created successfully\n");
			}
		}
	}

	if (p_chmask) {
		kctl = snd_ctl_new1(&u_audio_controls[UAC_P_PITCH_CTRL],
				    &uac->p_prm);
		if (!kctl) {
			err = -ENOMEM;
			goto snd_fail;
		}

		kctl->id.device = pcm->device;
		kctl->id.subdevice = 0;

		err = snd_ctl_add(card, kctl);
		if (err < 0) {
			dev_err(&uac->audio_dev->gadget->dev, "Failed to add UAC_P_PITCH_CTRL: %d\n", err);
			goto snd_fail;
		}
	}

	for (i = 0; i <= SNDRV_PCM_STREAM_LAST; i++) {
		struct uac_rtd_params *prm;
		struct uac_fu_params *fu;
		char ctrl_name[24];
		char *direction;

		if (!pcm->streams[i].substream_count)
			continue;

		if (i == SNDRV_PCM_STREAM_PLAYBACK) {
			prm = &uac->p_prm;
			fu = &params->p_fu;
			direction = "Playback";
		} else {
			prm = &uac->c_prm;
			fu = &params->c_fu;
			direction = "Capture";
		}

		prm->fu_id = fu->id;

		if (fu->mute_present) {
			snprintf(ctrl_name, sizeof(ctrl_name),
					"PCM %s Switch", direction);

			u_audio_controls[UAC_MUTE_CTRL].name = ctrl_name;

			kctl = snd_ctl_new1(&u_audio_controls[UAC_MUTE_CTRL],
					    prm);
			if (!kctl) {
				err = -ENOMEM;
				goto snd_fail;
			}

			kctl->id.device = pcm->device;
			kctl->id.subdevice = 0;

			err = snd_ctl_add(card, kctl);
			if (err < 0)
				goto snd_fail;
			prm->snd_kctl_mute_id = kctl->id;
			prm->mute = 0;
		}

		if (fu->volume_present) {
			snprintf(ctrl_name, sizeof(ctrl_name),
					"PCM %s Volume", direction);

			u_audio_controls[UAC_VOLUME_CTRL].name = ctrl_name;

			kctl = snd_ctl_new1(&u_audio_controls[UAC_VOLUME_CTRL],
					    prm);
			if (!kctl) {
				err = -ENOMEM;
				goto snd_fail;
			}

			kctl->id.device = pcm->device;
			kctl->id.subdevice = 0;


			kctl->tlv.c = u_audio_volume_tlv;
			kctl->vd[0].access |= SNDRV_CTL_ELEM_ACCESS_TLV_READ |
					SNDRV_CTL_ELEM_ACCESS_TLV_CALLBACK;

			err = snd_ctl_add(card, kctl);
			if (err < 0)
				goto snd_fail;
			prm->snd_kctl_volume_id = kctl->id;
			prm->volume = fu->volume_max;
			prm->volume_max = fu->volume_max;
			prm->volume_min = fu->volume_min;
			prm->volume_res = fu->volume_res;
		}

		/* Add rate control */
		snprintf(ctrl_name, sizeof(ctrl_name),
				"%s Rate", direction);
		u_audio_controls[UAC_RATE_CTRL].name = ctrl_name;

		kctl = snd_ctl_new1(&u_audio_controls[UAC_RATE_CTRL], prm);
		if (!kctl) {
			err = -ENOMEM;
			goto snd_fail;
		}

		kctl->id.device = pcm->device;
		kctl->id.subdevice = 0;

		err = snd_ctl_add(card, kctl);
		if (err < 0)
			goto snd_fail;
		prm->snd_kctl_rate_id = kctl->id;
	}

	strscpy(card->driver, card_name, sizeof(card->driver));
	strscpy(card->shortname, card_name, sizeof(card->shortname));
	sprintf(card->longname, "%s %i", card_name, card->dev->id);

	snd_pcm_set_managed_buffer_all(pcm, SNDRV_DMA_TYPE_CONTINUOUS,
				       NULL, 0, BUFF_SIZE_MAX);

	err = snd_card_register(card);
	if (err < 0)
		goto snd_fail;

	/* Create sysfs attributes for format change notification */
	uac->dev = device_create(audio_class, NULL, MKDEV(0, 0), uac,
				"uac_card%d", card->number);
	if (IS_ERR(uac->dev)) {
		err = PTR_ERR(uac->dev);
		uac->dev = NULL;
		/* Continue without sysfs - not fatal */
		goto skip_sysfs;
	}

	uac->kobj = &uac->dev->kobj;
	err = sysfs_create_group(uac->kobj, &uac_attr_group);
	if (err < 0) {
		device_destroy(audio_class, uac->dev->devt);
		uac->dev = NULL;
		/* Continue without sysfs - not fatal */
		goto skip_sysfs;
	}

skip_sysfs:
	g_audio->device = device_create(audio_class, NULL, MKDEV(0, 0), NULL,
					"%s", g_audio->uac->card->longname);
	if (IS_ERR(g_audio->device)) {
		err = PTR_ERR(g_audio->device);
		goto snd_fail;
	}

	INIT_WORK(&g_audio->work, g_audio_work);
	INIT_DELAYED_WORK(&g_audio->ppm_work, ppm_calculate_work);
	ppm_calculate_work(&g_audio->ppm_work.work);

	if (!err)
		return 0;

snd_fail:
	snd_card_free(card);
fail:
	kfree(uac->p_prm.reqs);
	kfree(uac->c_prm.reqs);
	kfree(uac->p_prm.rbuf);
	kfree(uac->c_prm.rbuf);
	kfree(uac);
	kfree(g_audio->fn);

	return err;
}
EXPORT_SYMBOL_GPL(g_audio_setup);

void g_audio_cleanup(struct g_audio *g_audio)
{
	struct snd_uac_chip *uac;
	struct snd_card *card;

	if (!g_audio || !g_audio->uac)
		return;

	cancel_work_sync(&g_audio->work);
	cancel_delayed_work_sync(&g_audio->ppm_work);
	device_destroy(g_audio->device->class, g_audio->device->devt);
	g_audio->device = NULL;

	uac = g_audio->uac;
	g_audio->uac = NULL;

	/* Cleanup sysfs attributes */
	if (uac->dev) {
		sysfs_remove_group(uac->kobj, &uac_attr_group);
		device_destroy(audio_class, uac->dev->devt);
	}

	card = uac->card;
	if (card)
		snd_card_free_when_closed(card);

	kfree(uac->p_prm.reqs);
	kfree(uac->c_prm.reqs);
	kfree(uac->p_prm.rbuf);
	kfree(uac->c_prm.rbuf);
	kfree(uac);
	kfree(g_audio->fn);
}
EXPORT_SYMBOL_GPL(g_audio_cleanup);

static int __init u_audio_init(void)
{
	int err = 0;

	audio_class = class_create(THIS_MODULE, "u_audio");
	if (IS_ERR(audio_class)) {
		err = PTR_ERR(audio_class);
		audio_class = NULL;
	}

	return err;
}
module_init(u_audio_init);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("USB gadget \"ALSA sound card\" utilities");
MODULE_AUTHOR("Ruslan Bilovol");
