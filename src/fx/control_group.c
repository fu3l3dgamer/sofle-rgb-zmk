/*
 * Copyright (c) 2024 Kuba Birecki
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_rgb_fx_control_group

#include <stdio.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>

#include <drivers/behavior.h>
#include <drivers/rgb_fx.h>

#include <zmk/rgb_fx.h>
#include <zmk/rgb_fx_control_group.h>

#include <zmk/behavior.h>
#include <zmk/event_manager.h>

#if IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
#include <zmk/split/central.h>
#endif

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#define PHANDLE_TO_DEVICE(node_id, prop, idx) DEVICE_DT_GET(DT_PHANDLE_BY_IDX(node_id, prop, idx)),

struct fx_control_group_work_context {
    const struct device *control_group;
    struct k_work_delayable save_work;
};

struct fx_control_group_config {
    const struct device **fx;
    const size_t fx_size;
    const uint8_t brightness_steps;
    struct fx_control_group_work_context *work;
    struct settings_handler *settings_handler;
};

struct fx_control_group_data {
    bool active;
    uint8_t brightness;
    size_t current_fx_idx;
    uint16_t hue_offset;
    uint8_t speed_step;
};

/*
 * STATE MODEL (assumes a single control group, the `zmk,rgb-fx` chosen):
 *
 * ALL state (active, mode, hue, speed, brightness) is SHARED and owned by
 * the CENTRAL half. All `&rgbfx` commands run on the central only; after
 * each one the central pushes the resulting ABSOLUTE state to the
 * peripheral through the `rgbsync` split behavior, and re-pushes it
 * every FX_SYNC_PERIOD as a self-heal (a peripheral that missed a
 * message converges on the next push). Relative commands applied
 * per-half were the source of inverted toggles and desynced modes.
 *
 * (A per-half USB override used to force a plugged half on at 100%; it
 * was removed at the user's request — halves rendered visibly different
 * brightness.)
 */

/* Whether the current effect instance has been started (render loop). */
static bool fx_running;

/* Brightness curve: the lowest step is 10% (battery default, barely sips
 * power); the remaining steps spread evenly up to 100%. With the default
 * 4 usable steps: 10 / 40 / 70 / 100%. */
static float fx_control_group_brightness_scale(uint8_t brightness, uint8_t steps) {
    if (brightness >= steps) {
        return 1.0f;
    }

    return 0.1f + (0.9f * (float)(brightness - 1) / (float)(steps - 1));
}

/* Start/stop the current effect so it matches the active state. */
static void fx_control_group_refresh(const struct device *dev) {
    const struct fx_control_group_config *config = dev->config;
    struct fx_control_group_data *data = dev->data;

    bool want = data->active;

    if (want == fx_running) {
        return;
    }

    if (want) {
        rgb_fx_start(config->fx[data->current_fx_idx]);
    } else {
        rgb_fx_stop(config->fx[data->current_fx_idx]);
    }

    fx_running = want;
}

/* Change the current effect, restarting it if it is running. */
static void fx_control_group_set_idx(const struct device *dev, size_t idx) {
    const struct fx_control_group_config *config = dev->config;
    struct fx_control_group_data *data = dev->data;

    if (idx >= config->fx_size || idx == data->current_fx_idx) {
        return;
    }

    if (fx_running) {
        rgb_fx_stop(config->fx[data->current_fx_idx]);
    }

    data->current_fx_idx = idx;

    if (fx_running) {
        rgb_fx_start(config->fx[data->current_fx_idx]);
    }
}

/* Packed absolute state carried in the rgbsync behavior's param1. */
#define FX_SYNC_PACK(active, idx, hue, speed, brt)                                                 \
    (((active) ? 1 : 0) | (((uint32_t)(idx) & 0x1f) << 1) |                                        \
     ((((uint32_t)(hue) / 20) & 0x1f) << 6) | (((uint32_t)(speed) & 0x7) << 11) |                  \
     (((uint32_t)(brt) & 0x7) << 14))
#define FX_SYNC_ACTIVE(p) ((p) & 0x1)
#define FX_SYNC_IDX(p) (((p) >> 1) & 0x1f)
#define FX_SYNC_HUE(p) ((((p) >> 6) & 0x1f) * 20)
#define FX_SYNC_SPEED(p) (((p) >> 11) & 0x7)
#define FX_SYNC_BRT(p) (((p) >> 14) & 0x7)

#if IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL) && DT_HAS_CHOSEN(zmk_rgb_fx)
static void fx_control_group_sync_push_now(void);
#endif

static int fx_control_group_load_settings(const struct device *dev, const char *name, size_t len,
                                          settings_read_cb read_cb, void *cb_arg) {
#if IS_ENABLED(CONFIG_SETTINGS)
    const char *next;
    int rc;

    if (settings_name_steq(name, "state", &next) && !next) {
        if (len != sizeof(struct fx_control_group_data)) {
            return -EINVAL;
        }

        rc = read_cb(cb_arg, dev->data, sizeof(struct fx_control_group_data));
        if (rc >= 0) {
            zmk_rgb_fx_hue_offset = ((struct fx_control_group_data *)dev->data)->hue_offset % 360;
            zmk_rgb_fx_speed_set(((struct fx_control_group_data *)dev->data)->speed_step);
            /* Sanitize saved state: a persisted brightness of 0 leaves the RGB
             * black forever (render x0) even if the toggle is pressed. */
            const struct fx_control_group_config *config = dev->config;
            struct fx_control_group_data *data = dev->data;

            if (data->brightness == 0) {
                data->brightness = config->brightness_steps;
            }
            if (data->current_fx_idx >= config->fx_size) {
                data->current_fx_idx = 0;
            }
            /* Never restore the saved on/off state: every boot starts
             * with the RGB off until the user toggles it on. */
            data->active = false;
            return 0;
        }

        return rc;
    }

    return -ENOENT;
#else
    return 0;
#endif /* IS_ENABLED(CONFIG_SETTINGS) */
}

#if IS_ENABLED(CONFIG_SETTINGS)
static void fx_control_group_save_work(struct k_work *work) {
    struct k_work_delayable *dwork = CONTAINER_OF(work, struct k_work_delayable, work);
    struct fx_control_group_work_context *ctx = CONTAINER_OF(dwork, struct fx_control_group_work_context, save_work);

    const struct device *dev = ctx->control_group;

    char path[40];
    snprintf(path, 40, "%s/state", dev->name);

    settings_save_one(path, dev->data, sizeof(struct fx_control_group_data));
};

static int fx_control_group_save_settings(const struct device *dev) {
    const struct fx_control_group_config *config = dev->config;
    struct fx_control_group_work_context *ctx = config->work;

    k_work_cancel_delayable(&ctx->save_work);

    return k_work_reschedule(&ctx->save_work, K_MSEC(CONFIG_ZMK_SETTINGS_SAVE_DEBOUNCE));
}
#endif /* IS_ENABLED(CONFIG_SETTINGS) */

int zmk_rgb_fx_control_handle_command(const struct device *dev, uint8_t command, uint8_t param) {
    const struct fx_control_group_config *config = dev->config;
    struct fx_control_group_data *data = dev->data;

    LOG_INF("fx ctrl: cmd=%d param=%d active=%d idx=%d brt=%d/%d", command, param, (int)data->active,
            (int)data->current_fx_idx, (int)data->brightness, (int)config->brightness_steps);

    /* &rgbfx has global locality, so the other half runs it too. If that
     * half is idle, the change has to wake it to be seen at all. */
    zmk_rgb_fx_wake();

    switch (command) {
    case RGB_FX_CMD_TOGGLE:
        data->active = !data->active;

        if (data->active) {
            /* Manual power-on: brightness defaults back to the 10% step
             * (the encoder can raise it afterwards). */
            data->brightness = 1;
        }
        break;
    /* Upstream didn't stop the outgoing effect nor start the incoming one
     * on change (NEXT/PREVIOUS/SELECT): set_idx handles both. */
    case RGB_FX_CMD_NEXT:
        fx_control_group_set_idx(dev, (data->current_fx_idx + 1) % config->fx_size);
        break;
    case RGB_FX_CMD_PREVIOUS:
        fx_control_group_set_idx(dev, (data->current_fx_idx + config->fx_size - 1) %
                                          config->fx_size);
        break;
    case RGB_FX_CMD_SELECT:
        if (config->fx_size <= param) {
            return -ENOTSUP;
        }

        fx_control_group_set_idx(dev, param);
        break;
    case RGB_FX_CMD_HUE_UP:
        zmk_rgb_fx_hue_offset = (zmk_rgb_fx_hue_offset + 20) % 360;
        data->hue_offset = zmk_rgb_fx_hue_offset;
        break;
    case RGB_FX_CMD_HUE_DOWN:
        zmk_rgb_fx_hue_offset = (zmk_rgb_fx_hue_offset + 340) % 360;
        data->hue_offset = zmk_rgb_fx_hue_offset;
        break;
    case RGB_FX_CMD_SPEED_UP:
        if (zmk_rgb_fx_speed_get() < 4) {
            zmk_rgb_fx_speed_set(zmk_rgb_fx_speed_get() + 1);
        }
        data->speed_step = zmk_rgb_fx_speed_get();
        break;
    case RGB_FX_CMD_SPEED_DOWN:
        if (zmk_rgb_fx_speed_get() > 0) {
            zmk_rgb_fx_speed_set(zmk_rgb_fx_speed_get() - 1);
        }
        data->speed_step = zmk_rgb_fx_speed_get();
        break;
    case RGB_FX_CMD_DIM:
        /* Minimum 1: turning off is the TOGGLE's job. A persisted brightness 0
         * left the keyboard black forever across reflashes. */
        if (data->brightness <= 1) {
            return 0;
        }

        data->brightness--;
        break;
    case RGB_FX_CMD_BRIGHTEN:
        if (data->brightness == config->brightness_steps) {
            return 0;
        }

        data->brightness++;
        break;
    /* CODEKEEB: absolute setters (ZMK Studio). Same clamping rules as the
     * relative commands -- brightness never reaches 0 here, because
     * turning the lighting off is the TOGGLE's job and a persisted 0 used
     * to leave the board black across reflashes. */
    case RGB_FX_CMD_SET_BRIGHTNESS:
        data->brightness = CLAMP(param, 1, config->brightness_steps);
        break;
    case RGB_FX_CMD_SET_HUE:
        /* param is hue/2 so the whole 0..359 range fits in one byte. */
        zmk_rgb_fx_hue_offset = (param * 2) % 360;
        data->hue_offset = zmk_rgb_fx_hue_offset;
        break;
    case RGB_FX_CMD_SET_SPEED:
        zmk_rgb_fx_speed_set(MIN(param, 4));
        data->speed_step = zmk_rgb_fx_speed_get();
        break;
    case RGB_FX_CMD_SET_ACTIVE:
        data->active = !!param;
        if (data->active && data->brightness == 0) {
            data->brightness = 1;
        }
        break;
    }

    fx_control_group_refresh(dev);

#if IS_ENABLED(CONFIG_SETTINGS)
    fx_control_group_save_settings(dev);
#endif /* IS_ENABLED(CONFIG_SETTINGS) */

    // Force refresh
    zmk_rgb_fx_request_frames(1);

#if IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL) && DT_HAS_CHOSEN(zmk_rgb_fx)
    /* Push the resulting absolute state to the peripheral right away. */
    fx_control_group_sync_push_now();
#endif

    return 0;
}

/* CODEKEEB: apply several values in one go.
 *
 * Going through handle_command once per field meant a refresh, a flash
 * write and a push to the peripheral EACH -- five of them inside a few
 * milliseconds when a slider moves -- and a SELECT with a bad index
 * returned early, so everything after it was silently dropped. That is
 * why toggling the lighting worked from the editor but changing the
 * effect, brightness, hue or speed did not.
 *
 * Here the state is updated first and settled once at the end. */
int zmk_rgb_fx_control_apply(const struct device *dev, const struct zmk_rgb_fx_set *set) {
    if (!dev || !set) {
        return -ENODEV;
    }

    const struct fx_control_group_config *config = dev->config;
    struct fx_control_group_data *data = dev->data;

    /* Studio traffic isn't key activity, so an idle keyboard would take
     * the change without showing it. */
    zmk_rgb_fx_wake();

    if (set->active >= 0) {
        data->active = !!set->active;
        if (data->active && data->brightness == 0) {
            data->brightness = 1;
        }
    }

    if (set->effect >= 0 && set->effect < (int16_t)config->fx_size) {
        fx_control_group_set_idx(dev, (size_t)set->effect);
    }

    if (set->brightness >= 0) {
        data->brightness = CLAMP(set->brightness, 1, config->brightness_steps);
    }

    if (set->hue >= 0) {
        zmk_rgb_fx_hue_offset = set->hue % 360;
        data->hue_offset = zmk_rgb_fx_hue_offset;
    }

    if (set->speed >= 0) {
        zmk_rgb_fx_speed_set(MIN(set->speed, 4));
        data->speed_step = zmk_rgb_fx_speed_get();
    }

    fx_control_group_refresh(dev);

#if IS_ENABLED(CONFIG_SETTINGS)
    fx_control_group_save_settings(dev);
#endif

    zmk_rgb_fx_request_frames(1);

#if IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL) && DT_HAS_CHOSEN(zmk_rgb_fx)
    fx_control_group_sync_push_now();
#endif

    return 0;
}

/* CODEKEEB: read back the current state, so a client can show what the
 * keyboard is really doing rather than assuming. */
int zmk_rgb_fx_control_get_state(const struct device *dev, struct zmk_rgb_fx_state *out) {
    if (!dev || !out) {
        return -ENODEV;
    }

    const struct fx_control_group_config *config = dev->config;
    struct fx_control_group_data *data = dev->data;

    out->active = data->active;
    out->brightness = data->brightness;
    out->brightness_max = config->brightness_steps;
    out->effect = (uint8_t)data->current_fx_idx;
    out->effect_count = (uint8_t)config->fx_size;
    out->hue = data->hue_offset;
    out->speed = data->speed_step;

    return 0;
}

static void fx_control_group_render_frame(const struct device *dev, struct rgb_fx_pixel *pixels,
                                          size_t num_pixels) {
    const struct fx_control_group_config *config = dev->config;
    const struct fx_control_group_data *data = dev->data;

    if (!data->active) {
        return;
    }

    rgb_fx_render_frame(config->fx[data->current_fx_idx], pixels, num_pixels);

    if (data->brightness == config->brightness_steps) {
        return;
    }

    float brightness =
        fx_control_group_brightness_scale(data->brightness, config->brightness_steps);

    for (size_t i = 0; i < num_pixels; ++i) {
        pixels[i].value.r *= brightness;
        pixels[i].value.g *= brightness;
        pixels[i].value.b *= brightness;
    }
}

static void fx_control_group_start(const struct device *dev) {
    const struct fx_control_group_config *config = dev->config;
    const struct fx_control_group_data *data = dev->data;

    if (!data->active) {
        return;
    }

    rgb_fx_start(config->fx[data->current_fx_idx]);
}

static void fx_control_group_stop(const struct device *dev) {
    const struct fx_control_group_config *config = dev->config;
    const struct fx_control_group_data *data = dev->data;

    rgb_fx_stop(config->fx[data->current_fx_idx]);
}

static int fx_control_group_init(const struct device *dev) {
#if IS_ENABLED(CONFIG_SETTINGS)
    const struct fx_control_group_config *config = dev->config;

    settings_subsys_init();

    settings_register(config->settings_handler);

    k_work_init_delayable(&config->work->save_work, fx_control_group_save_work);

    settings_load_subtree(dev->name);
#endif /* IS_ENABLED(CONFIG_SETTINGS) */

    return 0;
}

static const struct rgb_fx_api fx_control_group_api = {
    .on_start = fx_control_group_start,
    .on_stop = fx_control_group_stop,
    .render_frame = fx_control_group_render_frame,
};

#define FX_CONTROL_GROUP_DEVICE(idx)                                                               \
                                                                                                   \
    static const struct device *fx_control_group_##idx##_fx[] = {                                  \
        DT_INST_FOREACH_PROP_ELEM(idx, fx, PHANDLE_TO_DEVICE)};                                    \
                                                                                                   \
    static struct fx_control_group_work_context fx_control_group_##idx##_work = {                  \
        .control_group = DEVICE_DT_GET(DT_DRV_INST(idx)),                                          \
    };                                                                                             \
                                                                                                   \
    static int fx_control_group_##idx##_load_settings(const char *name, size_t len,                \
                                                      settings_read_cb read_cb, void *cb_arg) {    \
        const struct device *dev = DEVICE_DT_GET(DT_DRV_INST(idx));                                \
                                                                                                   \
        return fx_control_group_load_settings(dev, name, len, read_cb, cb_arg);                    \
    }                                                                                              \
                                                                                                   \
    static struct settings_handler fx_control_group_##idx##_settings_handler = {                   \
        .name = "fx_control_group_"#idx,                                                           \
        .h_set = fx_control_group_##idx##_load_settings,                                           \
    };                                                                                             \
                                                                                                   \
    static const struct fx_control_group_config fx_control_group_##idx##_config = {                \
        .fx = fx_control_group_##idx##_fx,                                                         \
        .fx_size = DT_INST_PROP_LEN(idx, fx),                                                      \
        .brightness_steps = DT_INST_PROP(idx, brightness_steps) - 1,                               \
        .work = &fx_control_group_##idx##_work,                                                    \
        .settings_handler = &fx_control_group_##idx##_settings_handler,                            \
    };                                                                                             \
                                                                                                   \
    static struct fx_control_group_data fx_control_group_##idx##_data = {                          \
        /* OFF by default: the user toggles it on (10% step). Also keeps   \
         * 36 LEDs from slamming the rail at boot. */                      \
        .active = false,                                                                           \
        .brightness = 1,                                                                           \
        .current_fx_idx = 0,                                                                       \
        .speed_step = 2, /* 1x */                                                                  \
    };                                                                                             \
                                                                                                   \
    DEVICE_DT_INST_DEFINE(idx, &fx_control_group_init, NULL, &fx_control_group_##idx##_data,       \
                          &fx_control_group_##idx##_config, POST_KERNEL,                           \
                          CONFIG_APPLICATION_INIT_PRIORITY, &fx_control_group_api);

DT_INST_FOREACH_STATUS_OKAY(FX_CONTROL_GROUP_DEVICE);

/* ---- central: push the absolute shared state to the peripheral ----
 *
 * Immediately after every &rgbfx command, and periodically as a
 * self-heal: a peripheral that missed a push (rebooting, out of range,
 * reconnecting) converges within FX_SYNC_PERIOD without any relative
 * command ever being replayed. */
#if IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL) && DT_HAS_CHOSEN(zmk_rgb_fx)

#define FX_SYNC_PERIOD K_SECONDS(15)

static void fx_control_group_sync_work_cb(struct k_work *work) {
    const struct device *dev = DEVICE_DT_GET(DT_CHOSEN(zmk_rgb_fx));
    const struct fx_control_group_data *data = dev->data;

    struct zmk_behavior_binding binding = {
        .behavior_dev = "rgbsync",
        .param1 = FX_SYNC_PACK(data->active, data->current_fx_idx, zmk_rgb_fx_hue_offset,
                               zmk_rgb_fx_speed_get(), data->brightness),
    };
    struct zmk_behavior_binding_event event = {
        .position = 0,
        .timestamp = k_uptime_get(),
    };

    for (uint8_t source = 0; source < ZMK_SPLIT_CENTRAL_PERIPHERAL_COUNT; source++) {
        zmk_split_central_invoke_behavior(source, &binding, event, true);
    }

    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    k_work_reschedule(dwork, FX_SYNC_PERIOD);
}

static K_WORK_DELAYABLE_DEFINE(fx_sync_work, fx_control_group_sync_work_cb);

static void fx_control_group_sync_push_now(void) { k_work_reschedule(&fx_sync_work, K_NO_WAIT); }

/* Kick off the periodic push shortly after boot (gives BLE time to link). */
static int fx_control_group_sync_init(void) {
    k_work_reschedule(&fx_sync_work, K_SECONDS(5));
    return 0;
}

SYS_INIT(fx_control_group_sync_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

#endif /* central && chosen */

/* ---- behavior rgbsync: the peripheral applies the absolute state ---- */

#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT zmk_behavior_rgb_fx_sync

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT) && DT_HAS_CHOSEN(zmk_rgb_fx)

static int fx_sync_on_pressed(struct zmk_behavior_binding *binding,
                              struct zmk_behavior_binding_event event) {
    const struct device *dev = DEVICE_DT_GET(DT_CHOSEN(zmk_rgb_fx));
    const struct fx_control_group_config *config = dev->config;
    struct fx_control_group_data *data = dev->data;

    uint32_t p = binding->param1;
    bool active = FX_SYNC_ACTIVE(p);
    size_t idx = MIN(FX_SYNC_IDX(p), config->fx_size - 1);
    uint16_t hue = FX_SYNC_HUE(p) % 360;
    uint8_t speed = FX_SYNC_SPEED(p);
    uint8_t brightness = CLAMP(FX_SYNC_BRT(p), 1, config->brightness_steps);

    if (active == data->active && idx == data->current_fx_idx &&
        hue == zmk_rgb_fx_hue_offset && speed == zmk_rgb_fx_speed_get() &&
        brightness == data->brightness) {
        return ZMK_BEHAVIOR_OPAQUE; /* already in sync: don't wear flash */
    }

    LOG_INF("rgbsync: active=%d idx=%d hue=%d speed=%d brt=%d", (int)active, (int)idx, (int)hue,
            (int)speed, (int)brightness);

    zmk_rgb_fx_wake();

    data->active = active;
    data->brightness = brightness;
    zmk_rgb_fx_hue_offset = hue;
    data->hue_offset = hue;
    zmk_rgb_fx_speed_set(speed);
    data->speed_step = speed;
    fx_control_group_set_idx(dev, idx);
    fx_control_group_refresh(dev);
    zmk_rgb_fx_request_frames(1);

#if IS_ENABLED(CONFIG_SETTINGS)
    fx_control_group_save_settings(dev);
#endif

    return ZMK_BEHAVIOR_OPAQUE;
}

static int fx_sync_on_released(struct zmk_behavior_binding *binding,
                               struct zmk_behavior_binding_event event) {
    return ZMK_BEHAVIOR_OPAQUE;
}

static const struct behavior_driver_api fx_sync_driver_api = {
    .binding_pressed = fx_sync_on_pressed,
    .binding_released = fx_sync_on_released,
};

BEHAVIOR_DT_INST_DEFINE(0, NULL, NULL, NULL, NULL, POST_KERNEL,
                        CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &fx_sync_driver_api);

#endif /* DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT) && chosen */
