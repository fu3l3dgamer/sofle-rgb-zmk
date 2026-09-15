/*
 * Copyright (c) 2024 Kuba Birecki
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_rgb_fx

#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <zephyr/device.h>
#include <zephyr/drivers/led_strip.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <drivers/rgb_fx.h>
#include <drivers/ext_power.h>

#include <zmk/rgb_fx.h>
#include <zmk/activity.h>
#include <zmk/event_manager.h>
#include <zmk/events/activity_state_changed.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#define PHANDLE_TO_DEVICE(node_id, prop, idx) DEVICE_DT_GET(DT_PHANDLE_BY_IDX(node_id, prop, idx)),

#define PHANDLE_TO_PIXEL(node_id, prop, idx)                                                       \
    {                                                                                              \
        .position_x = DT_PHA_BY_IDX(node_id, prop, idx, position_x),                               \
        .position_y = DT_PHA_BY_IDX(node_id, prop, idx, position_y),                               \
    },

/**
 * LED Driver device pointers.
 */
static const struct device *drivers[] = {DT_INST_FOREACH_PROP_ELEM(0, drivers, PHANDLE_TO_DEVICE)};

/**
 * Size of the LED driver device pointers array.
 */
static const size_t drivers_size = DT_INST_PROP_LEN(0, drivers);

/**
 * Array containing the number of LEDs handled by each device.
 */
static const size_t pixels_per_driver[] = DT_INST_PROP(0, chain_lengths);

/**
 * Pointer to the root effect
 */
static const struct device *fx_root = DEVICE_DT_GET(DT_CHOSEN(zmk_rgb_fx));

/**
 * Pixel configuration.
 */
static struct rgb_fx_pixel pixels[] = {DT_INST_FOREACH_PROP_ELEM(0, pixels, PHANDLE_TO_PIXEL)};

/**
 * Size of the pixels array.
 */
static const size_t pixels_size = DT_INST_PROP_LEN(0, pixels);

/**
 * Buffer for RGB values ready to be sent to the drivers.
 */
static struct led_rgb px_buffer[DT_INST_PROP_LEN(0, pixels)];

/**
 * Counter for effect animation frames that have been requested but have yet to be executed.
 */
static uint32_t fx_timer_countdown = 0;

/* Set while this half is IDLE or asleep: the strip is blanked and nothing
 * renders. Frame requests are dropped, and a tick that was already queued
 * when the half went idle does nothing. Without this, any request arriving
 * while idle (a layer tint or RGB state relayed from the other half, a ZMK
 * Studio change) rendered a frame, and the animated effects (gradient,
 * sparkle) ask for their next frame from inside render_frame, so the strip
 * came back on and kept animating until deep sleep. */
static bool fx_suspended = false;

/* Global speed: scales the animation tick period. Step 2 = 1x.
 * All effects advance per-frame, so changing the frame rate
 * speeds them up/slows them down equally. */
static const uint16_t fx_speed_period_ms[] = {
    (1000 / CONFIG_ZMK_RGB_FX_FPS) * 4, /* 0: 0.25x */
    (1000 / CONFIG_ZMK_RGB_FX_FPS) * 2, /* 1: 0.5x  */
    (1000 / CONFIG_ZMK_RGB_FX_FPS),     /* 2: 1x    */
    (1000 / CONFIG_ZMK_RGB_FX_FPS) / 2, /* 3: 2x    */
    (1000 / CONFIG_ZMK_RGB_FX_FPS) / 4, /* 4: 4x    */
};

static uint8_t fx_speed_step = 2;

uint8_t zmk_rgb_fx_speed_get(void) { return fx_speed_step; }

void zmk_rgb_fx_speed_set(uint8_t step) {
    if (step >= ARRAY_SIZE(fx_speed_period_ms)) {
        step = 2;
    }

    fx_speed_step = step;
}

/**
 * Conditional implementation of zmk_rgb_fx_get_pixel_by_key_position
 * if key-pixels is set.
 */
/* Upstream had a typo here (key_position) that voided key-pixels. */
#if DT_INST_NODE_HAS_PROP(0, key_pixels)
static const uint8_t pixels_by_key_position[] = DT_INST_PROP(0, key_pixels);

size_t zmk_rgb_fx_get_pixel_by_key_position(size_t key_position) {
    return pixels_by_key_position[key_position];
}
#endif

#if defined(CONFIG_ZMK_RGB_FX_PIXEL_DISTANCE) && (CONFIG_ZMK_RGB_FX_PIXEL_DISTANCE == 1)

/**
 * Lookup table for distance between any two pixels.
 *
 * The values are stored as a triangular matrix which cuts the space requirement roughly in half.
 */
static uint8_t
    pixel_distance[((DT_INST_PROP_LEN(0, pixels) + 1) * DT_INST_PROP_LEN(0, pixels)) / 2];

uint8_t zmk_rgb_fx_get_pixel_distance(size_t pixel_idx, size_t other_pixel_idx) {
    if (pixel_idx < other_pixel_idx) {
        return zmk_rgb_fx_get_pixel_distance(other_pixel_idx, pixel_idx);
    }

    return pixel_distance[(((pixel_idx + 1) * pixel_idx) >> 1) + other_pixel_idx];
}

#endif

static void zmk_rgb_fx_tick(struct k_work *work) {
    static uint32_t tick_count = 0;

    if (fx_suspended) {
        return;
    }

    rgb_fx_render_frame(fx_root, &pixels[0], pixels_size);

    /* Per-layer tint (lower/raise), overwrites the whole frame. */
    zmk_rgb_fx_layer_color_apply(&pixels[0], pixels_size);

    for (size_t i = 0; i < pixels_size; ++i) {
        zmk_rgb_to_led_rgb(&pixels[i].value, &px_buffer[i]);

        // Reset values for the next cycle
        pixels[i].value.r = 0;
        pixels[i].value.g = 0;
        pixels[i].value.b = 0;
    }

#ifdef CONFIG_ZMK_USB_LOGGING
    /* DEBUG: for the first 3 seconds, solid red bypassing the effect
     * pipeline entirely. */
    if (tick_count < 90) {
        for (size_t i = 0; i < pixels_size; ++i) {
            px_buffer[i].r = 60;
            px_buffer[i].g = 0;
            px_buffer[i].b = 0;
        }
    }
#endif

    size_t pixels_updated = 0;

    for (size_t i = 0; i < drivers_size; ++i) {
        int rc = led_strip_update_rgb(drivers[i], &px_buffer[pixels_updated], pixels_per_driver[i]);

        if (rc != 0) {
            LOG_ERR("led_strip_update_rgb fallo: %d", rc);
        }

        pixels_updated += pixels_per_driver[i];
    }

    if (tick_count < 5 || tick_count % 300 == 0) {
        int ext = -1;
#if DT_HAS_COMPAT_STATUS_OKAY(zmk_ext_power_generic)
        const struct device *ext_power = DEVICE_DT_GET(DT_INST(0, zmk_ext_power_generic));
        if (device_is_ready(ext_power)) {
            ext = ext_power_get(ext_power);
        }
#endif
        LOG_INF("fx tick %u: px0=(%d,%d,%d) px14=(%d,%d,%d) ext_power=%d", tick_count,
                px_buffer[0].r, px_buffer[0].g, px_buffer[0].b, px_buffer[14].r, px_buffer[14].g,
                px_buffer[14].b, ext);
#ifdef CONFIG_SOC_NRF52840
        /* DEPURACION: registros reales del SPIM3 (base 0x4002F000).
         * PSEL: bit31=1 means "disconnected"; low bits = pin number. */
        LOG_INF("SPIM3 ENABLE=%08x PSEL.SCK=%08x PSEL.MOSI=%08x FREQ=%08x ready=%d",
                *(volatile uint32_t *)0x4002F500, *(volatile uint32_t *)0x4002F508,
                *(volatile uint32_t *)0x4002F50C, *(volatile uint32_t *)0x4002F524,
                (int)device_is_ready(drivers[0]));
#endif
    }
    tick_count++;
}

K_WORK_DEFINE(animation_work, zmk_rgb_fx_tick);

static void zmk_rgb_fx_tick_handler(struct k_timer *timer) {
    if (--fx_timer_countdown == 0) {
        k_timer_stop(timer);
    }

    k_work_submit(&animation_work);
}

K_TIMER_DEFINE(animation_tick, zmk_rgb_fx_tick_handler, NULL);

void zmk_rgb_fx_request_frames(uint32_t frames) {
    if (fx_suspended || frames <= fx_timer_countdown) {
        return;
    }

    if (fx_timer_countdown == 0) {
        uint16_t period = fx_speed_period_ms[fx_speed_step];
        k_timer_start(&animation_tick, K_MSEC(period), K_MSEC(period));
    }

    fx_timer_countdown = frames;
}

/* Physically turn off the strip (black frame). */
static void zmk_rgb_fx_blank(void) {
    memset(px_buffer, 0, sizeof(px_buffer));

    size_t pixels_updated = 0;

    for (size_t i = 0; i < drivers_size; ++i) {
        led_strip_update_rgb(drivers[i], &px_buffer[pixels_updated], pixels_per_driver[i]);
        pixels_updated += pixels_per_driver[i];
    }
}

/* Stop the effects and turn the strip off. Safe to call when already
 * suspended. */
static void zmk_rgb_fx_suspend(void) {
    fx_suspended = true;
    rgb_fx_stop(fx_root);
    k_timer_stop(&animation_tick);
    fx_timer_countdown = 0;
    zmk_rgb_fx_blank();
}

/* Revive the effects. A no-op when not suspended, so a half that is already
 * rendering doesn't restart its effect (which would jump the gradient phase). */
static void zmk_rgb_fx_resume(void) {
    if (!fx_suspended) {
        return;
    }

    fx_suspended = false;
    rgb_fx_start(fx_root);
    zmk_rgb_fx_request_frames(1);
}

/* End of a wake window: blank again unless this half saw its own activity
 * in the meantime (if it did, its next IDLE event blanks it instead). */
static void zmk_rgb_fx_wake_expired(struct k_work *work) {
    if (zmk_activity_get_state() != ZMK_ACTIVITY_ACTIVE) {
        zmk_rgb_fx_suspend();
    }
}

static K_WORK_DELAYABLE_DEFINE(fx_wake_work, zmk_rgb_fx_wake_expired);

void zmk_rgb_fx_wake(void) {
    if (!fx_suspended) {
        /* Already awake. If that is only because of an earlier wake, a new
         * change keeps it awake for another full window. */
        if (k_work_delayable_is_pending(&fx_wake_work)) {
            k_work_reschedule(&fx_wake_work, K_MSEC(CONFIG_ZMK_IDLE_TIMEOUT));
        }
        return;
    }

    zmk_rgb_fx_resume();
    k_work_reschedule(&fx_wake_work, K_MSEC(CONFIG_ZMK_IDLE_TIMEOUT));
}

static int zmk_rgb_fx_on_activity_state_changed(const zmk_event_t *event) {
    const struct zmk_activity_state_changed *activity_state_event;

    if ((activity_state_event = as_zmk_activity_state_changed(event)) == NULL) {
        // Event not supported.
        return -ENOTSUP;
    }

    switch (activity_state_event->state) {
    case ZMK_ACTIVITY_ACTIVE:
        zmk_rgb_fx_resume();
        return 0;
    /* Auto-off: on IDLE (CONFIG_ZMK_IDLE_TIMEOUT idle) the effects stop
     * and the strip turns off; ACTIVE revives them. */
    case ZMK_ACTIVITY_IDLE:
    case ZMK_ACTIVITY_SLEEP:
        zmk_rgb_fx_suspend();
        return 0;
    default:
        return 0;
    }
}

static int zmk_rgb_fx_init() {
/* Upstream left this block commented out (and under the old config name),
 * leaving the distance table zeroed: the ripple lit everything at once. */
#if defined(CONFIG_ZMK_RGB_FX_PIXEL_DISTANCE) && (CONFIG_ZMK_RGB_FX_PIXEL_DISTANCE == 1)
    // Prefill the pixel distance lookup table
    int k = 0;
    for (size_t i = 0; i < pixels_size; ++i) {
        for (size_t j = 0; j <= i; ++j) {
            // Distances are normalized to fit inside 0-255 range to fit inside uint8_t
            // for better space efficiency
            pixel_distance[k++] = sqrt(pow(pixels[i].position_x - pixels[j].position_x, 2) +
                                       pow(pixels[i].position_y - pixels[j].position_y, 2)) *
                                  255 / 360;
        }
    }
#endif

    LOG_INF("ZMK RGB FX Ready");

    rgb_fx_start(fx_root);

    return 0;
}

SYS_INIT(zmk_rgb_fx_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

ZMK_LISTENER(amk_rgb_fx, zmk_rgb_fx_on_activity_state_changed);
ZMK_SUBSCRIPTION(amk_rgb_fx, zmk_activity_state_changed);
