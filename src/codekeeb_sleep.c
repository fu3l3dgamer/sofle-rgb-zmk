/*
 * SPDX-License-Identifier: MIT
 *
 * CODEKEEB PATCH: deep sleep con tiempo editable desde ZMK Studio.
 *
 * ZMK ya duerme por inactividad, pero el plazo es
 * CONFIG_ZMK_IDLE_SLEEP_TIMEOUT, un int de Kconfig que queda cableado en
 * el binario: cambiar los minutos exigia recompilar y reflashear las dos
 * mitades. Aqui el plazo (y el activado/desactivado) viven en flash y se
 * cambian en caliente, como el RGB o la animacion del OLED.
 *
 * No se toca activity.c de ZMK, que es codigo de upstream y no de este
 * repo: el .conf deja CONFIG_ZMK_IDLE_SLEEP_TIMEOUT en un valor enorme
 * para que el temporizador de ZMK no salte nunca antes, y la cuenta real
 * la lleva este modulo, que duerme con las mismas llamadas que usa ZMK
 * (zmk_pm_suspend_devices + sys_poweroff).
 *
 * Se compila en LAS DOS MITADES a proposito. Cada una vigila su propia
 * inactividad, porque si solo durmiera la central el periferico seguiria
 * gastando bateria a la espera. La central reparte los cambios de ajuste
 * al periferico invocando &sleepcfg, igual que hace con &oledset.
 */

#include <errno.h>

#include <zephyr/kernel.h>
#include <zephyr/settings/settings.h>
#include <zephyr/sys/poweroff.h>

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#include <zmk/activity.h>
#include <zmk/event_manager.h>
#include <zmk/events/activity_state_changed.h>
#include <zmk/events/position_state_changed.h>
#include <zmk/events/sensor_event.h>
#include <zmk/pm.h>
#if IS_ENABLED(CONFIG_USB_DEVICE_STACK)
#include <zmk/usb.h>
#endif

#include <zmk/codekeeb_sleep.h>

#define DEFAULT_MINUTES 30
#define SETTINGS_KEY "codekeeb_sleep"

struct sleep_cfg {
    uint8_t enabled;
    uint16_t minutes;
} __packed;

static struct sleep_cfg cfg = {
    .enabled = 1,
    .minutes = DEFAULT_MINUTES,
};

static uint32_t last_activity;

static bool usb_power_present(void) {
#if IS_ENABLED(CONFIG_USB_DEVICE_STACK)
    return zmk_usb_is_powered();
#else
    return false;
#endif
}

bool zmk_codekeeb_sleep_enabled(void) { return cfg.enabled != 0; }

uint16_t zmk_codekeeb_sleep_minutes(void) { return cfg.minutes; }

/* La cuenta se reinicia en cada pulsacion. En la central llegan tambien
 * las del periferico (position_state_changed viaja por el enlace), asi
 * que escribir en cualquier mitad mantiene despierta a la central; el
 * periferico solo ve las suyas, que es justo lo que debe vigilar. */
static int activity_listener(const zmk_event_t *eh) {
    last_activity = k_uptime_get_32();
    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(codekeeb_sleep, activity_listener);
ZMK_SUBSCRIPTION(codekeeb_sleep, zmk_position_state_changed);
ZMK_SUBSCRIPTION(codekeeb_sleep, zmk_activity_state_changed);
/* Encoder turns count as activity too, the same as in ZMK's own activity.c.
 * Without this, scrolling with an encoder for longer than the timeout
 * without pressing a key powered the half off mid-scroll. The peripheral
 * raises these for its own encoder; the central also gets the peripheral's. */
ZMK_SUBSCRIPTION(codekeeb_sleep, zmk_sensor_event);

static void sleep_work_handler(struct k_work *work) {
    if (!cfg.enabled) {
        return;
    }

    /* Con el cable puesto no tiene sentido dormir: hay corriente de
     * sobra y apagarse en mitad de una sesion seria un incordio. Es lo
     * mismo que hace ZMK en activity.c. El guard del include importa: en
     * el periferico no entra el stack USB y zmk_usb_is_powered() no
     * existe alli. */
    if (usb_power_present()) {
        last_activity = k_uptime_get_32();
        return;
    }

    uint32_t inactive = k_uptime_get_32() - last_activity;

    if (inactive < (uint32_t)cfg.minutes * 60U * 1000U) {
        return;
    }

    LOG_INF("codekeeb: %u min sin actividad, durmiendo", cfg.minutes);

    if (zmk_pm_suspend_devices() < 0) {
        LOG_ERR("codekeeb: fallo al suspender dispositivos, no se duerme");
        zmk_pm_resume_devices();
        return;
    }

    sys_poweroff();
}

K_WORK_DEFINE(sleep_work, sleep_work_handler);

static void sleep_timer_expiry(struct k_timer *timer) { k_work_submit(&sleep_work); }

K_TIMER_DEFINE(sleep_timer, sleep_timer_expiry, NULL);

#if IS_ENABLED(CONFIG_SETTINGS)
static int sleep_settings_set(const char *name, size_t len, settings_read_cb read_cb,
                              void *cb_arg) {
    const char *next;

    if (settings_name_steq(name, "cfg", &next) && !next) {
        if (len != sizeof(cfg)) {
            return -EINVAL;
        }

        int rc = read_cb(cb_arg, &cfg, sizeof(cfg));
        if (rc < 0) {
            return rc;
        }

        /* Un valor corrupto en flash no puede dejar el teclado
         * apagandose cada pocos segundos. */
        if (cfg.minutes < CODEKEEB_SLEEP_MIN_MINUTES ||
            cfg.minutes > CODEKEEB_SLEEP_MAX_MINUTES) {
            cfg.minutes = DEFAULT_MINUTES;
        }

        return 0;
    }

    return -ENOENT;
}

SETTINGS_STATIC_HANDLER_DEFINE(codekeeb_sleep, SETTINGS_KEY, NULL, sleep_settings_set, NULL, NULL);
#endif /* IS_ENABLED(CONFIG_SETTINGS) */

int zmk_codekeeb_sleep_set(bool enabled, uint16_t minutes) {
    if (minutes < CODEKEEB_SLEEP_MIN_MINUTES || minutes > CODEKEEB_SLEEP_MAX_MINUTES) {
        return -EINVAL;
    }

    cfg.enabled = enabled ? 1 : 0;
    cfg.minutes = minutes;

    /* Cambiar el ajuste reinicia la cuenta: si no, subir el plazo desde
     * un teclado ya inactivo lo dormiria al instante siguiente. */
    last_activity = k_uptime_get_32();

#if IS_ENABLED(CONFIG_SETTINGS)
    settings_save_one(SETTINGS_KEY "/cfg", &cfg, sizeof(cfg));
#endif

    return 0;
}

static int codekeeb_sleep_init(void) {
    last_activity = k_uptime_get_32();

    /* Un tick por segundo, como el de ZMK: el plazo son minutos, asi que
     * no hace falta mas resolucion. */
    k_timer_start(&sleep_timer, K_SECONDS(1), K_SECONDS(1));

    return 0;
}

SYS_INIT(codekeeb_sleep_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
