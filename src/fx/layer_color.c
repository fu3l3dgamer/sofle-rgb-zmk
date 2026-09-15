/*
 * SPDX-License-Identifier: MIT
 *
 * Per-layer tint: while LOWER (pink) or RAISE (purple) is active,
 * ALL thumb LEDs on both halves are painted with the warning color.
 * In LOWER the arrow cluster (right half) lights up in orange; in RAISE
 * the Bluetooth panel is shown (BT_CLR red, profiles 0-4 yellow with the
 * active one green).
 * Only the central half knows the layers, so the state is relayed to the
 * peripheral over the split behavior channel (behavior `rgblay`,
 * name <= 8 chars).
 */

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <drivers/behavior.h>
#include <drivers/rgb_fx.h>

#include <zmk/behavior.h>
#include <zmk/ble.h>
#include <zmk/event_manager.h>
#include <zmk/events/ble_active_profile_changed.h>
#include <zmk/events/layer_state_changed.h>
#include <zmk/keymap.h>
#include <zmk/rgb_fx.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

/* 0 = no tint (base layer); indices into the color table. */
static uint8_t layer_tint = 0;

/* Per-layer warning colors: [1] = LOWER (pink), [2] = RAISE (purple). */
static const struct zmk_color_hsl layer_tint_colors[] = {
    {.h = 0, .s = 0, .l = 0},
    {.h = 330, .s = 100, .l = 50},  /* LOWER = pink */
    {.h = 270, .s = 100, .l = 50},  /* RAISE = purple */
};

/* All thumb LEDs (same chain indices on both halves).
 * MX chain: pixel 0 = encoder, 1-6 = underglow, 7-35 = per-key.
 * Thumb pixels ordered outer -> inner: 7, 16, 17, 26, 27.
 * (Column orientation confirmed on hardware: the overlay map is
 * MIRRORED left/right — the PCB photo showed the back side.) */
static const uint8_t layer_tint_thumb_px[] = {7, 16, 17, 26, 27};

/* Arrow cluster in LOWER: I(UP) J(LEFT) K(DOWN) L(RIGHT) positions.
 * Only exists on the right half (peripheral). */
static const uint8_t layer_tint_arrow_px[] = {23, 29, 24, 19};

/* Saturated orange-yellow for the arrows. */
static const struct zmk_color_hsl layer_tint_arrow_color = {.h = 40, .s = 100, .l = 50};

/* FIXED warning colors: compensate the global hue offset so they
 * don't rotate with the hue adjustment. */
static struct zmk_color_rgb tint_to_rgb(struct zmk_color_hsl hsl) {
    struct zmk_color_rgb rgb;

    hsl.h = (hsl.h + 360 - zmk_rgb_fx_hue_offset) % 360;
    zmk_hsl_to_rgb(&hsl, &rgb);

    return rgb;
}

void zmk_rgb_fx_layer_color_apply(struct rgb_fx_pixel *pixels, size_t num_pixels) {
    if (layer_tint == 0 || layer_tint >= ARRAY_SIZE(layer_tint_colors)) {
        return;
    }

    struct zmk_color_rgb rgb = tint_to_rgb(layer_tint_colors[layer_tint]);

    /* ALL thumbs on this half are painted with the layer color
     * (LOWER pink / RAISE purple), on both halves. */
    for (size_t j = 0; j < ARRAY_SIZE(layer_tint_thumb_px); j++) {
        if (layer_tint_thumb_px[j] < num_pixels) {
            pixels[layer_tint_thumb_px[j]].value = rgb;
        }
    }

#if IS_ENABLED(CONFIG_ZMK_SPLIT) && !IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
    /* In LOWER, light up the arrow cluster (right half only). */
    if (layer_tint == 1) {
        struct zmk_color_rgb flechas = tint_to_rgb(layer_tint_arrow_color);

        for (size_t j = 0; j < ARRAY_SIZE(layer_tint_arrow_px); j++) {
            if (layer_tint_arrow_px[j] < num_pixels) {
                pixels[layer_tint_arrow_px[j]].value = flechas;
            }
        }
    }
#endif

#if IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL) || !IS_ENABLED(CONFIG_ZMK_SPLIT)
    /* Bluetooth panel in RAISE: BT_CLR (top-left corner) red,
     * profiles 0-4 yellow and the ACTIVE profile green. */
    if (layer_tint == 2) {
        static const uint8_t bt_clr_px = 11;                 /* top-left corner (BT_CLR) */
        static const uint8_t bt_prof_px[] = {12, 21, 22, 31, 32}; /* keys 1-5 */

        if (bt_clr_px < num_pixels) {
            pixels[bt_clr_px].value =
                tint_to_rgb((struct zmk_color_hsl){.h = 0, .s = 100, .l = 50});
        }

        struct zmk_color_rgb amarillo =
            tint_to_rgb((struct zmk_color_hsl){.h = 50, .s = 100, .l = 50});
        struct zmk_color_rgb verde =
            tint_to_rgb((struct zmk_color_hsl){.h = 120, .s = 100, .l = 50});
        uint8_t active = zmk_ble_active_profile_index();

        for (size_t i = 0; i < ARRAY_SIZE(bt_prof_px); i++) {
            if (bt_prof_px[i] < num_pixels) {
                pixels[bt_prof_px[i]].value = (i == active) ? verde : amarillo;
            }
        }
    }
#endif
}

static void layer_tint_set(uint8_t tint) {
    if (tint == layer_tint) {
        return;
    }

    layer_tint = tint;
    zmk_rgb_fx_request_frames(1);
}

/* ---- behavior rgblay: receives the state on the peripheral ---- */

#define DT_DRV_COMPAT zmk_behavior_rgb_fx_layer

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

static int on_keymap_binding_pressed(struct zmk_behavior_binding *binding,
                                     struct zmk_behavior_binding_event event) {
    /* A layer held on the other half should show here (the LOWER arrow
     * cluster lives on this half) even if nobody has typed here lately.
     * Going back to the base layer has nothing to show, so it doesn't
     * wake an idle half. */
    if (binding->param1 != 0) {
        zmk_rgb_fx_wake();
    }

    layer_tint_set(binding->param1);
    return ZMK_BEHAVIOR_OPAQUE;
}

static int on_keymap_binding_released(struct zmk_behavior_binding *binding,
                                      struct zmk_behavior_binding_event event) {
    return ZMK_BEHAVIOR_OPAQUE;
}

static const struct behavior_driver_api behavior_rgb_fx_layer_driver_api = {
    .binding_pressed = on_keymap_binding_pressed,
    .binding_released = on_keymap_binding_released,
};

BEHAVIOR_DT_INST_DEFINE(0, NULL, NULL, NULL, NULL, POST_KERNEL,
                        CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &behavior_rgb_fx_layer_driver_api);

#endif /* DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT) */

/* ---- central: listen to layer changes and relay to the peripheral ---- */

#if IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL) || !IS_ENABLED(CONFIG_ZMK_SPLIT)

#include <zmk/split/central.h>

static int layer_color_listener(const zmk_event_t *eh) {
    /* BT profile change while RAISE is held: redraw the panel. */
    if (as_zmk_ble_active_profile_changed(eh) != NULL) {
        if (layer_tint == 2) {
            zmk_rgb_fx_request_frames(1);
        }
        return 0;
    }

    if (as_zmk_layer_state_changed(eh) == NULL) {
        return -ENOTSUP;
    }

    uint8_t highest = zmk_keymap_highest_layer_active();
    uint8_t tint = (highest == 1 || highest == 2) ? highest : 0;

    layer_tint_set(tint);

#if IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
    struct zmk_behavior_binding binding = {
        .behavior_dev = "rgblay",
        .param1 = tint,
    };
    struct zmk_behavior_binding_event event = {
        .position = 0,
        .timestamp = k_uptime_get(),
    };

    for (uint8_t source = 0; source < ZMK_SPLIT_CENTRAL_PERIPHERAL_COUNT; source++) {
        zmk_split_central_invoke_behavior(source, &binding, event, true);
    }
#endif

    return 0;
}

ZMK_LISTENER(rgb_fx_layer_color, layer_color_listener);
ZMK_SUBSCRIPTION(rgb_fx_layer_color, zmk_layer_state_changed);
ZMK_SUBSCRIPTION(rgb_fx_layer_color, zmk_ble_active_profile_changed);

#endif /* central */
