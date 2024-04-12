/*
 * Copyright (c) 2020 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/settings/settings.h>

#include <math.h>
#include <stdlib.h>

#include <zmk/battery.h>
#include <zmk/ble.h>
#include <zmk/endpoints.h>
#include <zmk/keymap.h>
#include <zmk/hid_indicators.h>
#include <zmk/usb.h>

#include <zephyr/logging/log.h>

#include <zephyr/drivers/led_strip.h>
#include <drivers/ext_power.h>

#include <zmk/rgb_underglow.h>

#include <zmk/activity.h>
#include <zmk/event_manager.h>
#include <zmk/events/activity_state_changed.h>
#include <zmk/events/usb_conn_state_changed.h>
#include <zmk/workqueue.h>

#if IS_ENABLED(CONFIG_ZMK_SPLIT_BLE_CENTRAL_BATTERY_LEVEL_FETCHING)
#include <zmk/split/bluetooth/central.h>
#endif

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#if !DT_HAS_CHOSEN(zmk_underglow)

#error "A zmk,underglow chosen node must be declared"

#endif

#define STRIP_CHOSEN DT_CHOSEN(zmk_underglow)
#define STRIP_NUM_PIXELS DT_PROP(STRIP_CHOSEN, chain_length)

#define HUE_MAX 360
#define SAT_MAX 100
#define BRT_MAX 100

BUILD_ASSERT(CONFIG_ZMK_RGB_UNDERGLOW_BRT_MIN <= CONFIG_ZMK_RGB_UNDERGLOW_BRT_MAX,
             "ERROR: RGB underglow maximum brightness is less than minimum brightness");

enum rgb_underglow_effect {
    UNDERGLOW_EFFECT_SOLID,
    UNDERGLOW_EFFECT_BREATHE,
    UNDERGLOW_EFFECT_SPECTRUM,
    UNDERGLOW_EFFECT_SWIRL,
    UNDERGLOW_EFFECT_NUMBER // Used to track number of underglow effects
};

struct rgb_underglow_state {
    struct zmk_led_hsb color;
    uint8_t animation_speed;
    uint8_t current_effect;
    uint16_t animation_step;
    bool on;
    bool status_active;
    uint16_t status_animation_step;
};

static const struct device *led_strip;

static struct led_rgb pixels[STRIP_NUM_PIXELS];
static struct led_rgb status_pixels[STRIP_NUM_PIXELS];

static struct rgb_underglow_state state;

#if IS_ENABLED(CONFIG_ZMK_RGB_UNDERGLOW_EXT_POWER)
static const struct device *const ext_power = DEVICE_DT_GET(DT_INST(0, zmk_ext_power_generic));
#endif

void zmk_rgb_set_ext_power(void);

static struct zmk_led_hsb hsb_scale_min_max(struct zmk_led_hsb hsb) {
    hsb.b = CONFIG_ZMK_RGB_UNDERGLOW_BRT_MIN +
            (CONFIG_ZMK_RGB_UNDERGLOW_BRT_MAX - CONFIG_ZMK_RGB_UNDERGLOW_BRT_MIN) * hsb.b / BRT_MAX;
    return hsb;
}

static struct zmk_led_hsb hsb_scale_zero_max(struct zmk_led_hsb hsb) {
    hsb.b = hsb.b * CONFIG_ZMK_RGB_UNDERGLOW_BRT_MAX / BRT_MAX;
    return hsb;
}

static struct led_rgb hsb_to_rgb(struct zmk_led_hsb hsb) {
    float r, g, b;

    uint8_t i = hsb.h / 60;
    float v = hsb.b / ((float)BRT_MAX);
    float s = hsb.s / ((float)SAT_MAX);
    float f = hsb.h / ((float)HUE_MAX) * 6 - i;
    float p = v * (1 - s);
    float q = v * (1 - f * s);
    float t = v * (1 - (1 - f) * s);

    switch (i % 6) {
    case 0:
        r = v;
        g = t;
        b = p;
        break;
    case 1:
        r = q;
        g = v;
        b = p;
        break;
    case 2:
        r = p;
        g = v;
        b = t;
        break;
    case 3:
        r = p;
        g = q;
        b = v;
        break;
    case 4:
        r = t;
        g = p;
        b = v;
        break;
    case 5:
        r = v;
        g = p;
        b = q;
        break;
    }

    struct led_rgb rgb = {r : r * 255, g : g * 255, b : b * 255};

    return rgb;
}

static void zmk_rgb_underglow_effect_solid(void) {
    for (int i = 0; i < STRIP_NUM_PIXELS; i++) {
        pixels[i] = hsb_to_rgb(hsb_scale_min_max(state.color));
    }
}

static void zmk_rgb_underglow_effect_breathe(void) {
    for (int i = 0; i < STRIP_NUM_PIXELS; i++) {
        struct zmk_led_hsb hsb = state.color;
        hsb.b = abs(state.animation_step - 1200) / 12;

        pixels[i] = hsb_to_rgb(hsb_scale_zero_max(hsb));
    }

    state.animation_step += state.animation_speed * 10;

    if (state.animation_step > 2400) {
        state.animation_step = 0;
    }
}

static void zmk_rgb_underglow_effect_spectrum(void) {
    for (int i = 0; i < STRIP_NUM_PIXELS; i++) {
        struct zmk_led_hsb hsb = state.color;
        hsb.h = state.animation_step;

        pixels[i] = hsb_to_rgb(hsb_scale_min_max(hsb));
    }

    state.animation_step += state.animation_speed;
    state.animation_step = state.animation_step % HUE_MAX;
}

static void zmk_rgb_underglow_effect_swirl(void) {
    for (int i = 0; i < STRIP_NUM_PIXELS; i++) {
        struct zmk_led_hsb hsb = state.color;
        hsb.h = (HUE_MAX / STRIP_NUM_PIXELS * i + state.animation_step) % HUE_MAX;

        pixels[i] = hsb_to_rgb(hsb_scale_min_max(hsb));
    }

    state.animation_step += state.animation_speed * 2;
    state.animation_step = state.animation_step % HUE_MAX;
}

static int zmk_led_generate_status(void);

static void valdur_indicate_custom_layers(void);

static void zmk_led_write_pixels(void) {
    static struct led_rgb led_buffer[STRIP_NUM_PIXELS];
    int bat0 = zmk_battery_state_of_charge();
    int blend = 0;
    int reset_ext_power = 0;

    valdur_indicate_custom_layers();

    if (state.status_active) {
        blend = zmk_led_generate_status();
    }

    // fast path: no status indicators, battery level OK
    if (blend == 0 && bat0 >= 20) {
        led_strip_update_rgb(led_strip, pixels, STRIP_NUM_PIXELS);
        return;
    }
    // battery below minimum charge
    if (bat0 < 10) {
        memset(pixels, 0, sizeof(struct led_rgb) * STRIP_NUM_PIXELS);
        if (state.on) {
            int c_power = ext_power_get(ext_power);
            if (c_power && !state.status_active) {
                // power is on, RGB underglow is on, but battery is too low
                state.on = false;
                reset_ext_power = true;
            }
        }
    }

    if (blend == 0) {
        for (int i = 0; i < STRIP_NUM_PIXELS; i++) {
            led_buffer[i] = pixels[i];
        }
    } else if (blend >= 256) {
        for (int i = 0; i < STRIP_NUM_PIXELS; i++) {
            led_buffer[i] = status_pixels[i];
        }
    } else if (blend < 256) {
        uint16_t blend_l = blend;
        uint16_t blend_r = 256 - blend;
        for (int i = 0; i < STRIP_NUM_PIXELS; i++) {
            led_buffer[i].r =
                ((status_pixels[i].r * blend_l) >> 8) + ((pixels[i].r * blend_r) >> 8);
            led_buffer[i].g =
                ((status_pixels[i].g * blend_l) >> 8) + ((pixels[i].g * blend_r) >> 8);
            led_buffer[i].b =
                ((status_pixels[i].b * blend_l) >> 8) + ((pixels[i].b * blend_r) >> 8);
        }
    }

    // battery below 20%, reduce LED brightness
    if (bat0 < 20) {
        for (int i = 0; i < STRIP_NUM_PIXELS; i++) {
            led_buffer[i].r = led_buffer[i].r >> 1;
            led_buffer[i].g = led_buffer[i].g >> 1;
            led_buffer[i].b = led_buffer[i].b >> 1;
        }
    }

    int err = led_strip_update_rgb(led_strip, led_buffer, STRIP_NUM_PIXELS);
    if (err < 0) {
        LOG_ERR("Failed to update the RGB strip (%d)", err);
    }

    if (reset_ext_power) {
        zmk_rgb_set_ext_power();
    }
}

#define UNDERGLOW_INDICATORS DT_PATH(underglow_indicators)

#if defined(DT_N_S_underglow_indicators_EXISTS)
#define UNDERGLOW_INDICATORS_ENABLED 1
#else
#define UNDERGLOW_INDICATORS_ENABLED 0
#endif

#if !UNDERGLOW_INDICATORS_ENABLED
static int zmk_led_generate_status(void) { return 0; }
static void valdur_indicate_custom_layers(void) {}

#else

const uint8_t underglow_layer_state[] = DT_PROP(UNDERGLOW_INDICATORS, layer_state);
const uint8_t underglow_ble_state[] = DT_PROP(UNDERGLOW_INDICATORS, ble_state);
const uint8_t underglow_bat_lhs[] = DT_PROP(UNDERGLOW_INDICATORS, bat_lhs);
const uint8_t underglow_bat_rhs[] = DT_PROP(UNDERGLOW_INDICATORS, bat_rhs);

#define HEXRGB(R, G, B)                                                                            \
    ((struct led_rgb){                                                                             \
        r : (CONFIG_ZMK_RGB_UNDERGLOW_BRT_MAX * (R)) / 0xff,                                       \
        g : (CONFIG_ZMK_RGB_UNDERGLOW_BRT_MAX * (G)) / 0xff,                                       \
        b : (CONFIG_ZMK_RGB_UNDERGLOW_BRT_MAX * (B)) / 0xff                                        \
    })

const struct led_rgb red = HEXRGB(0xff, 0x00, 0x00);
const struct led_rgb orange = HEXRGB(0xff, 0x88, 0x00);
const struct led_rgb yellow = HEXRGB(0xff, 0xff, 0x00);
const struct led_rgb green = HEXRGB(0x00, 0xff, 0x00);
const struct led_rgb nice_blue = HEXRGB(0x00, 0xbe, 0xff);
const struct led_rgb magenta = HEXRGB(0xff, 0x00, 0xff);
const struct led_rgb white = HEXRGB(0xff, 0xff, 0xff);
const struct led_rgb lilac = HEXRGB(0x6b, 0x1f, 0xce);
const struct led_rgb greenish = HEXRGB(0x00, 0xff, 0x44);

/*
  MoErgo 40 LEDs

  34 28 22 16 10
  35 29 23 17 11 6
  36 30 24 18 12 7
  37 31 25 19 13 8
  38 32 26 20 14 9
  39 33 27 21 15
                0 1 2
                3 4 5
*/

static void valdur_indicate_custom_layers(void) {
    uint8_t gaming_layer = 1;
    uint8_t lower_layer = 2;
    uint8_t numeric_layer = 3;

    if (zmk_keymap_layer_active(lower_layer)) {

        // indicator
        pixels[37] = yellow;

        // arrows
        pixels[18] = yellow;
        pixels[25] = yellow;
        pixels[19] = yellow;
        pixels[13] = yellow;

        // ctrl arrows
        pixels[8] = yellow;
        pixels[31] = yellow;

        // home, end, pgup, pgdn
        pixels[17] = nice_blue;
        pixels[20] = nice_blue;
        pixels[24] = nice_blue;
        pixels[12] = nice_blue;

        // ctrl home, end
        pixels[7] = nice_blue;
        pixels[30] = nice_blue;

        // enter, backspace, delete
        pixels[14] = lilac;
        pixels[26] = lilac;
        pixels[32] = lilac;
    } else if (zmk_keymap_layer_active(gaming_layer)) {

        // indicator
        pixels[38] = red;

        // wsad
        pixels[18] = red;
        pixels[25] = red;
        pixels[19] = red;
        pixels[13] = red;

        // enter, backspace, delete
        pixels[5] = lilac;
        pixels[27] = lilac;
        pixels[33] = lilac;

    } else if (zmk_keymap_layer_active(numeric_layer)) {

        // indicator
        pixels[36] = greenish;

        // numbers
        pixels[23] = greenish;
        pixels[17] = greenish;
        pixels[11] = greenish;

        pixels[24] = greenish;
        pixels[18] = greenish;
        pixels[12] = greenish;

        pixels[25] = greenish;
        pixels[19] = greenish;
        pixels[13] = greenish;

        pixels[26] = greenish;

        // operators
        pixels[31] = yellow;
        pixels[32] = yellow;
        pixels[27] = yellow; // dot

        pixels[12] = yellow;
        pixels[13] = yellow;
        pixels[14] = yellow;
    }
    /*
     * Copyright (c) 2020 The ZMK Contributors
     * Copyright (c) 2023 Innaworks Development Limited, trading as MoErgo
     *
     * SPDX-License-Identifier: MIT
     */

    /* THIS FILE WAS GENERATED BY GLOVE80 LAYOUT EDITOR
     *
     * This file was generated automatically. You may or may not want to
     * edit it directly.
     */

#include <behaviors.dtsi>
/* Include all behaviour includes needed */
#include <dt-bindings/zmk/outputs.h>
#include <dt-bindings/zmk/keys.h>
#include <dt-bindings/zmk/bt.h>
#include <dt-bindings/zmk/rgb.h>

/* Automatically generated layer name #define */
#define LAYER_Base 0
#define LAYER_Gaming 1
#define LAYER_Lower 2
#define LAYER_Numeric 3
#define LAYER_Magic 4

/* To deal with the situation where there is no Lower layer, to keep &lower happy */
#ifndef LAYER_Lower
#define LAYER_Lower 0
#endif

    /* Custom Device-tree */

    /* Glove80 system behavior & macros */
    / {
        behaviors {
        // For the "layer" key, it'd nice to be able to use it as either a shift or a toggle.
        // Configure it as a tap dance, so the first tap (or hold) is a &mo and the second tap
        // is a &to
        lower:
            lower {
                compatible = "zmk,behavior-tap-dance";
                label = "LAYER_TAP_DANCE";
#binding - cells = < 0>;
                tapping - term - ms = <200>;
                bindings = <&mo LAYER_Lower>, <&to LAYER_Lower>;
            };
        };
    };

    / {
        macros {
        rgb_ug_status_macro:
            rgb_ug_status_macro {
                label = "RGB_UG_STATUS";
                compatible = "zmk,behavior-macro";
#binding - cells = < 0>;
                bindings = <&rgb_ug RGB_STATUS>;
            };
        };
    };

    / {
#ifdef BT_DISC_CMD
        behaviors {
        bt_0:
            bt_0 {
                compatible = "zmk,behavior-tap-dance";
                label = "BT_0";
#binding - cells = < 0>;
                tapping - term - ms = <200>;
                bindings = <&bt_select_0>, <&bt BT_DISC 0>;
            };
        bt_1:
            bt_1 {
                compatible = "zmk,behavior-tap-dance";
                label = "BT_1";
#binding - cells = < 0>;
                tapping - term - ms = <200>;
                bindings = <&bt_select_1>, <&bt BT_DISC 1>;
            };
        bt_2:
            bt_2 {
                compatible = "zmk,behavior-tap-dance";
                label = "BT_2";
#binding - cells = < 0>;
                tapping - term - ms = <200>;
                bindings = <&bt_select_2>, <&bt BT_DISC 2>;
            };
        bt_3:
            bt_3 {
                compatible = "zmk,behavior-tap-dance";
                label = "BT_3";
#binding - cells = < 0>;
                tapping - term - ms = <200>;
                bindings = <&bt_select_3>, <&bt BT_DISC 3>;
            };
        };
        macros {
        bt_select_0:
            bt_select_0 {
                label = "BT_SELECT_0";
                compatible = "zmk,behavior-macro";
#binding - cells = < 0>;
                bindings = <&out OUT_BLE>, <&bt BT_SEL 0>;
            };
        bt_select_1:
            bt_select_1 {
                label = "BT_SELECT_1";
                compatible = "zmk,behavior-macro";
#binding - cells = < 0>;
                bindings = <&out OUT_BLE>, <&bt BT_SEL 1>;
            };
        bt_select_2:
            bt_select_2 {
                label = "BT_SELECT_2";
                compatible = "zmk,behavior-macro";
#binding - cells = < 0>;
                bindings = <&out OUT_BLE>, <&bt BT_SEL 2>;
            };
        bt_select_3:
            bt_select_3 {
                label = "BT_SELECT_3";
                compatible = "zmk,behavior-macro";
#binding - cells = < 0>;
                bindings = <&out OUT_BLE>, <&bt BT_SEL 3>;
            };
        };
#else
        macros {
        bt_0:
            bt_0 {
                label = "BT_0";
                compatible = "zmk,behavior-macro";
#binding - cells = < 0>;
                bindings = <&out OUT_BLE>, <&bt BT_SEL 0>;
            };
        bt_1:
            bt_1 {
                label = "BT_1";
                compatible = "zmk,behavior-macro";
#binding - cells = < 0>;
                bindings = <&out OUT_BLE>, <&bt BT_SEL 1>;
            };
        bt_2:
            bt_2 {
                label = "BT_2";
                compatible = "zmk,behavior-macro";
#binding - cells = < 0>;
                bindings = <&out OUT_BLE>, <&bt BT_SEL 2>;
            };
        bt_3:
            bt_3 {
                label = "BT_3";
                compatible = "zmk,behavior-macro";
#binding - cells = < 0>;
                bindings = <&out OUT_BLE>, <&bt BT_SEL 3>;
            };
        };
#endif
    };

    / {
        behaviors {
        magic:
            magic {
                compatible = "zmk,behavior-hold-tap";
                label = "MAGIC_HOLD_TAP";
#binding - cells = < 2>;
                flavor = "tap-preferred";
                tapping - term - ms = <200>;
                bindings = <&mo>, <&rgb_ug_status_macro>;
            };
        };
    };

/* #define for key positions */
#define POS_LH_T1 52
#define POS_LH_T2 53
#define POS_LH_T3 54
#define POS_LH_T4 69
#define POS_LH_T5 70
#define POS_LH_T6 71
#define POS_LH_C1R2 15
#define POS_LH_C1R3 27
#define POS_LH_C1R4 39
#define POS_LH_C1R5 51
#define POS_LH_C2R1 4
#define POS_LH_C2R2 14
#define POS_LH_C2R3 26
#define POS_LH_C2R4 38
#define POS_LH_C2R5 50
#define POS_LH_C2R6 68
#define POS_LH_C3R1 3
#define POS_LH_C3R2 13
#define POS_LH_C3R3 25
#define POS_LH_C3R4 37
#define POS_LH_C3R5 49
#define POS_LH_C3R6 67
#define POS_LH_C4R1 2
#define POS_LH_C4R2 12
#define POS_LH_C4R3 24
#define POS_LH_C4R4 36
#define POS_LH_C4R5 48
#define POS_LH_C4R6 66
#define POS_LH_C5R1 1
#define POS_LH_C5R2 11
#define POS_LH_C5R3 23
#define POS_LH_C5R4 35
#define POS_LH_C5R5 47
#define POS_LH_C5R6 65
#define POS_LH_C6R1 0
#define POS_LH_C6R2 10
#define POS_LH_C6R3 22
#define POS_LH_C6R4 34
#define POS_LH_C6R5 46
#define POS_LH_C6R6 64
#define POS_RH_T1 57
#define POS_RH_T2 56
#define POS_RH_T3 55
#define POS_RH_T4 74
#define POS_RH_T5 73
#define POS_RH_T6 72
#define POS_RH_C1R2 16
#define POS_RH_C1R3 28
#define POS_RH_C1R4 40
#define POS_RH_C1R5 58
#define POS_RH_C2R1 5
#define POS_RH_C2R2 17
#define POS_RH_C2R3 29
#define POS_RH_C2R4 41
#define POS_RH_C2R5 59
#define POS_RH_C2R6 75
#define POS_RH_C3R1 6
#define POS_RH_C3R2 18
#define POS_RH_C3R3 30
#define POS_RH_C3R4 42
#define POS_RH_C3R5 60
#define POS_RH_C3R6 76
#define POS_RH_C4R1 7
#define POS_RH_C4R2 19
#define POS_RH_C4R3 31
#define POS_RH_C4R4 43
#define POS_RH_C4R5 61
#define POS_RH_C4R6 77
#define POS_RH_C5R1 8
#define POS_RH_C5R2 20
#define POS_RH_C5R3 32
#define POS_RH_C5R4 44
#define POS_RH_C5R5 62
#define POS_RH_C5R6 78
#define POS_RH_C6R1 9
#define POS_RH_C6R2 21
#define POS_RH_C6R3 33
#define POS_RH_C6R4 45
#define POS_RH_C6R5 63
#define POS_RH_C6R6 79

    /* Custom Defined Behaviors */
    / {
        // NOTE: Use the `#define` settings below to customize this keymap!
// For example, here are the main optional features you can enable:
#define DIFFICULTY_LEVEL 0 // 0:custom, 1:easy -> 5:hard (see below)
// TIP: Add more setting overrides here instead of editing them below.

//////////////////////////////////////////////////////////////////////////////
//
// Advanced Home Row Mods (HRM) Example based on
// Sunaku's Keymap v35 -- "Glorious Engrammer"
// - https://github.com/sunaku/glove80-keymaps
//
//////////////////////////////////////////////////////////////////////////////

//
// DIFFICULTY_LEVEL specifies your level of expertise with this keymap.
// It's meant to help newcomers gradually work their way up to mastery.
// You can disable this setting by omitting it or assigning a `0` zero.
//
// #define DIFFICULTY_LEVEL 0 // custom (see defaults below)
// #define DIFFICULTY_LEVEL 1 // novice (500ms)
// #define DIFFICULTY_LEVEL 2 // slower (400ms)
// #define DIFFICULTY_LEVEL 3 // normal (300ms)
// #define DIFFICULTY_LEVEL 4 // faster (200ms)
// #define DIFFICULTY_LEVEL 5 // expert (100ms)
//
#if defined(DIFFICULTY_LEVEL) && DIFFICULTY_LEVEL > 0
#define DIFFICULTY_THRESHOLD ((6 - DIFFICULTY_LEVEL) * 100)
#ifndef HOMEY_HOLDING_TIME
#define HOMEY_HOLDING_TIME DIFFICULTY_THRESHOLD
#endif
#ifndef INDEX_HOLDING_TIME
#define INDEX_HOLDING_TIME DIFFICULTY_THRESHOLD
#endif
#endif

        behaviors {

//////////////////////////////////////////////////////////////////////////
//
// Miryoku layers and home row mods (ported from my QMK endgame)
// - https://sunaku.github.io/home-row-mods.html#porting-to-zmk
// - https://github.com/urob/zmk-config#timeless-homerow-mods
//
//////////////////////////////////////////////////////////////////////////

//
// HOMEY_HOLDING_TYPE defines the flavor of ZMK hold-tap behavior to use
// for the pinky, ring, and middle fingers (which are assigned to Super,
// Alt, and Ctrl respectively in the Miryoku system) on home row keys.
//
#ifndef HOMEY_HOLDING_TYPE
#define HOMEY_HOLDING_TYPE "tap-preferred"
#endif

//
// HOMEY_HOLDING_TIME defines how long you need to hold (milliseconds)
// home row mod keys in order to send their modifiers to the computer
// (i.e. "register" them) for mod-click mouse usage (e.g. Ctrl-Click).
//
#ifndef HOMEY_HOLDING_TIME
#define HOMEY_HOLDING_TIME 270 // TAPPING_TERM + ALLOW_CROSSOVER_AFTER
#endif

//
// HOMEY_STREAK_DECAY defines how long you need to wait (milliseconds)
// after typing before you can use home row mods again.  It prevents
// unintended activation of home row mods when you're actively typing.
//
#ifndef HOMEY_STREAK_DECAY
#define HOMEY_STREAK_DECAY 250
#endif

//
// HOMEY_REPEAT_DECAY defines how much time you have left (milliseconds)
// after tapping a key to hold it again in order to make it auto-repeat.
//
#ifndef HOMEY_REPEAT_DECAY
#define HOMEY_REPEAT_DECAY 300 // "tap then hold" for key auto-repeat
#endif

//
// INDEX_HOLDING_TYPE defines the flavor of ZMK hold-tap behavior to use
// for index fingers (which Miryoku assigns to Shift) on home row keys.
//
#ifndef INDEX_HOLDING_TYPE
#define INDEX_HOLDING_TYPE "tap-preferred"
#endif

//
// INDEX_HOLDING_TIME defines how long you need to hold (milliseconds)
// index finger keys in order to send their modifiers to the computer
// (i.e. "register" them) for mod-click mouse usage (e.g. Shift-Click).
//
#ifndef INDEX_HOLDING_TIME
#define INDEX_HOLDING_TIME 170
#endif

//
// INDEX_STREAK_DECAY defines how long you need to wait (milliseconds)
// after typing before you can use home row mods again.  It prevents
// unintended activation of home row mods when you're actively typing.
//
#ifndef INDEX_STREAK_DECAY
#define INDEX_STREAK_DECAY 150
#endif

//
// INDEX_REPEAT_DECAY defines how much time you have left (milliseconds)
// after tapping a key to hold it again in order to make it auto-repeat.
//
#ifndef INDEX_REPEAT_DECAY
#define INDEX_REPEAT_DECAY 300 // "tap then hold" for key auto-repeat
#endif

//
// Glove80 key positions index for positional hold-tap
// - https://discord.com/channels/877392805654306816/937645688244826154/1066713913351221248
// - https://media.discordapp.net/attachments/937645688244826154/1066713913133121556/image.png
//
// |------------------------|------------------------|
// | LEFT_HAND_KEYS         |        RIGHT_HAND_KEYS |
// |                        |                        |
// |  0  1  2  3  4         |          5  6  7  8  9 |
// | 10 11 12 13 14 15      |      16 17 18 19 20 21 |
// | 22 23 24 25 26 27      |      28 29 30 31 32 33 |
// | 34 35 36 37 38 39      |      40 41 42 43 44 45 |
// | 46 47 48 49 50 51      |      58 59 60 61 62 63 |
// | 64 65 66 67 68         |         75 76 77 78 79 |
// |                69 52   |   57 74                |
// |                 70 53  |  56 73                 |
// |                  71 54 | 55 72                  |
// |------------------------|------------------------|
//
#define LEFT_HAND_KEYS                                                                             \
    0 1 2 3 4 10 11 12 13 14 15 22 23 24 25 26 27 34 35 36 37 38 39 46 47 48 49 50 51 64 65 66 67 68
#define RIGHT_HAND_KEYS                                                                            \
    5 6 7 8 9 16 17 18 19 20 21 28 29 30 31 32 33 40 41 42 43 44 45 58 59 60 61 62 63 75 76 77 78 79
#define THUMB_KEYS 69 52 57 74 70 53 56 73 71 54 55 72

//
// Home row mods with per-finger configuration settings
//
#ifndef PINKY_HOLDING_TYPE
#define PINKY_HOLDING_TYPE HOMEY_HOLDING_TYPE
#endif
#ifndef PINKY_HOLDING_TIME
#define PINKY_HOLDING_TIME HOMEY_HOLDING_TIME
#endif
#ifndef PINKY_STREAK_DECAY
#define PINKY_STREAK_DECAY HOMEY_STREAK_DECAY
#endif
#ifndef PINKY_REPEAT_DECAY
#define PINKY_REPEAT_DECAY HOMEY_REPEAT_DECAY
#endif
#ifndef RING1_HOLDING_TYPE
#define RING1_HOLDING_TYPE HOMEY_HOLDING_TYPE
#endif
#ifndef RING1_HOLDING_TIME
#define RING1_HOLDING_TIME HOMEY_HOLDING_TIME
#endif
#ifndef RING1_STREAK_DECAY
#define RING1_STREAK_DECAY HOMEY_STREAK_DECAY
#endif
#ifndef RING1_REPEAT_DECAY
#define RING1_REPEAT_DECAY HOMEY_REPEAT_DECAY
#endif
#ifndef RING2_HOLDING_TYPE
#define RING2_HOLDING_TYPE HOMEY_HOLDING_TYPE
#endif
#ifndef RING2_HOLDING_TIME
#define RING2_HOLDING_TIME HOMEY_HOLDING_TIME
#endif
#ifndef RING2_STREAK_DECAY
#define RING2_STREAK_DECAY HOMEY_STREAK_DECAY
#endif
#ifndef RING2_REPEAT_DECAY
#define RING2_REPEAT_DECAY HOMEY_REPEAT_DECAY
#endif
#ifndef MIDDY_HOLDING_TYPE
#define MIDDY_HOLDING_TYPE HOMEY_HOLDING_TYPE
#endif
#ifndef MIDDY_HOLDING_TIME
#define MIDDY_HOLDING_TIME HOMEY_HOLDING_TIME
#endif
#ifndef MIDDY_STREAK_DECAY
#define MIDDY_STREAK_DECAY HOMEY_STREAK_DECAY
#endif
#ifndef MIDDY_REPEAT_DECAY
#define MIDDY_REPEAT_DECAY HOMEY_REPEAT_DECAY
#endif
#ifndef INDEX_HOLDING_TYPE
#define INDEX_HOLDING_TYPE HOMEY_HOLDING_TYPE
#endif
#ifndef INDEX_HOLDING_TIME
#define INDEX_HOLDING_TIME HOMEY_HOLDING_TIME
#endif
#ifndef INDEX_STREAK_DECAY
#define INDEX_STREAK_DECAY HOMEY_STREAK_DECAY
#endif
#ifndef INDEX_REPEAT_DECAY
#define INDEX_REPEAT_DECAY HOMEY_REPEAT_DECAY
#endif
#ifndef LEFT_PINKY_HOLDING_TYPE
#define LEFT_PINKY_HOLDING_TYPE PINKY_HOLDING_TYPE
#endif
#ifndef LEFT_PINKY_HOLDING_TIME
#define LEFT_PINKY_HOLDING_TIME PINKY_HOLDING_TIME
#endif
#ifndef LEFT_PINKY_STREAK_DECAY
#define LEFT_PINKY_STREAK_DECAY PINKY_STREAK_DECAY
#endif
#ifndef LEFT_PINKY_REPEAT_DECAY
#define LEFT_PINKY_REPEAT_DECAY PINKY_REPEAT_DECAY
#endif
        left_pinky:
            homey_left_pinky {
                compatible = "zmk,behavior-hold-tap";
                flavor = LEFT_PINKY_HOLDING_TYPE;
                hold - trigger - key - positions = <RIGHT_HAND_KEYS THUMB_KEYS>;
                hold - trigger - on - release; // wait for other home row mods
                tapping - term - ms = <LEFT_PINKY_HOLDING_TIME>;
                quick - tap - ms = <LEFT_PINKY_REPEAT_DECAY>;
                require - prior - idle - ms = <LEFT_PINKY_STREAK_DECAY>;
#binding - cells = < 2>;
                bindings = <&kp>, <&kp>;
            };
#ifndef RIGHT_PINKY_HOLDING_TYPE
#define RIGHT_PINKY_HOLDING_TYPE PINKY_HOLDING_TYPE
#endif
#ifndef RIGHT_PINKY_HOLDING_TIME
#define RIGHT_PINKY_HOLDING_TIME PINKY_HOLDING_TIME
#endif
#ifndef RIGHT_PINKY_STREAK_DECAY
#define RIGHT_PINKY_STREAK_DECAY PINKY_STREAK_DECAY
#endif
#ifndef RIGHT_PINKY_REPEAT_DECAY
#define RIGHT_PINKY_REPEAT_DECAY PINKY_REPEAT_DECAY
#endif
        right_pinky:
            homey_right_pinky {
                compatible = "zmk,behavior-hold-tap";
                flavor = RIGHT_PINKY_HOLDING_TYPE;
                hold - trigger - key - positions = <LEFT_HAND_KEYS THUMB_KEYS>;
                hold - trigger - on - release; // wait for other home row mods
                tapping - term - ms = <RIGHT_PINKY_HOLDING_TIME>;
                quick - tap - ms = <RIGHT_PINKY_REPEAT_DECAY>;
                require - prior - idle - ms = <RIGHT_PINKY_STREAK_DECAY>;
#binding - cells = < 2>;
                bindings = <&kp>, <&kp>;
            };
#ifndef LEFT_RING1_HOLDING_TYPE
#define LEFT_RING1_HOLDING_TYPE RING1_HOLDING_TYPE
#endif
#ifndef LEFT_RING1_HOLDING_TIME
#define LEFT_RING1_HOLDING_TIME RING1_HOLDING_TIME
#endif
#ifndef LEFT_RING1_STREAK_DECAY
#define LEFT_RING1_STREAK_DECAY RING1_STREAK_DECAY
#endif
#ifndef LEFT_RING1_REPEAT_DECAY
#define LEFT_RING1_REPEAT_DECAY RING1_REPEAT_DECAY
#endif
        left_ring1:
            homey_left_ring1 {
                compatible = "zmk,behavior-hold-tap";
                flavor = LEFT_RING1_HOLDING_TYPE;
                hold - trigger - key - positions = <RIGHT_HAND_KEYS THUMB_KEYS>;
                hold - trigger - on - release; // wait for other home row mods
                tapping - term - ms = <LEFT_RING1_HOLDING_TIME>;
                quick - tap - ms = <LEFT_RING1_REPEAT_DECAY>;
                require - prior - idle - ms = <LEFT_RING1_STREAK_DECAY>;
#binding - cells = < 2>;
                bindings = <&kp>, <&kp>;
            };
#ifndef RIGHT_RING1_HOLDING_TYPE
#define RIGHT_RING1_HOLDING_TYPE RING1_HOLDING_TYPE
#endif
#ifndef RIGHT_RING1_HOLDING_TIME
#define RIGHT_RING1_HOLDING_TIME RING1_HOLDING_TIME
#endif
#ifndef RIGHT_RING1_STREAK_DECAY
#define RIGHT_RING1_STREAK_DECAY RING1_STREAK_DECAY
#endif
#ifndef RIGHT_RING1_REPEAT_DECAY
#define RIGHT_RING1_REPEAT_DECAY RING1_REPEAT_DECAY
#endif
        right_ring1:
            homey_right_ring1 {
                compatible = "zmk,behavior-hold-tap";
                flavor = RIGHT_RING1_HOLDING_TYPE;
                hold - trigger - key - positions = <LEFT_HAND_KEYS THUMB_KEYS>;
                hold - trigger - on - release; // wait for other home row mods
                tapping - term - ms = <RIGHT_RING1_HOLDING_TIME>;
                quick - tap - ms = <RIGHT_RING1_REPEAT_DECAY>;
                require - prior - idle - ms = <RIGHT_RING1_STREAK_DECAY>;
#binding - cells = < 2>;
                bindings = <&kp>, <&kp>;
            };
#ifndef LEFT_RING2_HOLDING_TYPE
#define LEFT_RING2_HOLDING_TYPE RING2_HOLDING_TYPE
#endif
#ifndef LEFT_RING2_HOLDING_TIME
#define LEFT_RING2_HOLDING_TIME RING2_HOLDING_TIME
#endif
#ifndef LEFT_RING2_STREAK_DECAY
#define LEFT_RING2_STREAK_DECAY RING2_STREAK_DECAY
#endif
#ifndef LEFT_RING2_REPEAT_DECAY
#define LEFT_RING2_REPEAT_DECAY RING2_REPEAT_DECAY
#endif
        left_ring2:
            homey_left_ring2 {
                compatible = "zmk,behavior-hold-tap";
                flavor = LEFT_RING2_HOLDING_TYPE;
                hold - trigger - key - positions = <RIGHT_HAND_KEYS THUMB_KEYS>;
                hold - trigger - on - release; // wait for other home row mods
                tapping - term - ms = <LEFT_RING2_HOLDING_TIME>;
                quick - tap - ms = <LEFT_RING2_REPEAT_DECAY>;
                require - prior - idle - ms = <LEFT_RING2_STREAK_DECAY>;
#binding - cells = < 2>;
                bindings = <&kp>, <&kp>;
            };
#ifndef RIGHT_RING2_HOLDING_TYPE
#define RIGHT_RING2_HOLDING_TYPE RING2_HOLDING_TYPE
#endif
#ifndef RIGHT_RING2_HOLDING_TIME
#define RIGHT_RING2_HOLDING_TIME RING2_HOLDING_TIME
#endif
#ifndef RIGHT_RING2_STREAK_DECAY
#define RIGHT_RING2_STREAK_DECAY RING2_STREAK_DECAY
#endif
#ifndef RIGHT_RING2_REPEAT_DECAY
#define RIGHT_RING2_REPEAT_DECAY RING2_REPEAT_DECAY
#endif
        right_ring2:
            homey_right_ring2 {
                compatible = "zmk,behavior-hold-tap";
                flavor = RIGHT_RING2_HOLDING_TYPE;
                hold - trigger - key - positions = <LEFT_HAND_KEYS THUMB_KEYS>;
                hold - trigger - on - release; // wait for other home row mods
                tapping - term - ms = <RIGHT_RING2_HOLDING_TIME>;
                quick - tap - ms = <RIGHT_RING2_REPEAT_DECAY>;
                require - prior - idle - ms = <RIGHT_RING2_STREAK_DECAY>;
#binding - cells = < 2>;
                bindings = <&kp>, <&kp>;
            };
#ifndef LEFT_MIDDY_HOLDING_TYPE
#define LEFT_MIDDY_HOLDING_TYPE MIDDY_HOLDING_TYPE
#endif
#ifndef LEFT_MIDDY_HOLDING_TIME
#define LEFT_MIDDY_HOLDING_TIME MIDDY_HOLDING_TIME
#endif
#ifndef LEFT_MIDDY_STREAK_DECAY
#define LEFT_MIDDY_STREAK_DECAY MIDDY_STREAK_DECAY
#endif
#ifndef LEFT_MIDDY_REPEAT_DECAY
#define LEFT_MIDDY_REPEAT_DECAY MIDDY_REPEAT_DECAY
#endif
        left_middy:
            homey_left_middy {
                compatible = "zmk,behavior-hold-tap";
                flavor = LEFT_MIDDY_HOLDING_TYPE;
                hold - trigger - key - positions = <RIGHT_HAND_KEYS THUMB_KEYS>;
                hold - trigger - on - release; // wait for other home row mods
                tapping - term - ms = <LEFT_MIDDY_HOLDING_TIME>;
                quick - tap - ms = <LEFT_MIDDY_REPEAT_DECAY>;
                require - prior - idle - ms = <LEFT_MIDDY_STREAK_DECAY>;
#binding - cells = < 2>;
                bindings = <&kp>, <&kp>;
            };
#ifndef RIGHT_MIDDY_HOLDING_TYPE
#define RIGHT_MIDDY_HOLDING_TYPE MIDDY_HOLDING_TYPE
#endif
#ifndef RIGHT_MIDDY_HOLDING_TIME
#define RIGHT_MIDDY_HOLDING_TIME MIDDY_HOLDING_TIME
#endif
#ifndef RIGHT_MIDDY_STREAK_DECAY
#define RIGHT_MIDDY_STREAK_DECAY MIDDY_STREAK_DECAY
#endif
#ifndef RIGHT_MIDDY_REPEAT_DECAY
#define RIGHT_MIDDY_REPEAT_DECAY MIDDY_REPEAT_DECAY
#endif
        right_middy:
            homey_right_middy {
                compatible = "zmk,behavior-hold-tap";
                flavor = RIGHT_MIDDY_HOLDING_TYPE;
                hold - trigger - key - positions = <LEFT_HAND_KEYS THUMB_KEYS>;
                hold - trigger - on - release; // wait for other home row mods
                tapping - term - ms = <RIGHT_MIDDY_HOLDING_TIME>;
                quick - tap - ms = <RIGHT_MIDDY_REPEAT_DECAY>;
                require - prior - idle - ms = <RIGHT_MIDDY_STREAK_DECAY>;
#binding - cells = < 2>;
                bindings = <&kp>, <&kp>;
            };
#ifndef LEFT_INDEX_HOLDING_TYPE
#define LEFT_INDEX_HOLDING_TYPE INDEX_HOLDING_TYPE
#endif
#ifndef LEFT_INDEX_HOLDING_TIME
#define LEFT_INDEX_HOLDING_TIME INDEX_HOLDING_TIME
#endif
#ifndef LEFT_INDEX_STREAK_DECAY
#define LEFT_INDEX_STREAK_DECAY INDEX_STREAK_DECAY
#endif
#ifndef LEFT_INDEX_REPEAT_DECAY
#define LEFT_INDEX_REPEAT_DECAY INDEX_REPEAT_DECAY
#endif
        left_index:
            homey_left_index {
                compatible = "zmk,behavior-hold-tap";
                flavor = LEFT_INDEX_HOLDING_TYPE;
                hold - trigger - key - positions = <RIGHT_HAND_KEYS THUMB_KEYS>;
                hold - trigger - on - release; // wait for other home row mods
                tapping - term - ms = <LEFT_INDEX_HOLDING_TIME>;
                quick - tap - ms = <LEFT_INDEX_REPEAT_DECAY>;
                require - prior - idle - ms = <LEFT_INDEX_STREAK_DECAY>;
#binding - cells = < 2>;
                bindings = <&kp>, <&kp>;
            };
#ifndef RIGHT_INDEX_HOLDING_TYPE
#define RIGHT_INDEX_HOLDING_TYPE INDEX_HOLDING_TYPE
#endif
#ifndef RIGHT_INDEX_HOLDING_TIME
#define RIGHT_INDEX_HOLDING_TIME INDEX_HOLDING_TIME
#endif
#ifndef RIGHT_INDEX_STREAK_DECAY
#define RIGHT_INDEX_STREAK_DECAY INDEX_STREAK_DECAY
#endif
#ifndef RIGHT_INDEX_REPEAT_DECAY
#define RIGHT_INDEX_REPEAT_DECAY INDEX_REPEAT_DECAY
#endif
        right_index:
            homey_right_index {
                compatible = "zmk,behavior-hold-tap";
                flavor = RIGHT_INDEX_HOLDING_TYPE;
                hold - trigger - key - positions = <LEFT_HAND_KEYS THUMB_KEYS>;
                hold - trigger - on - release; // wait for other home row mods
                tapping - term - ms = <RIGHT_INDEX_HOLDING_TIME>;
                quick - tap - ms = <RIGHT_INDEX_REPEAT_DECAY>;
                require - prior - idle - ms = <RIGHT_INDEX_STREAK_DECAY>;
#binding - cells = < 2>;
                bindings = <&kp>, <&kp>;
            };
        };
    };

    /* Automatically generated macro definitions */
    / {
        macros{

        };
    };

    /* Automatically generated keymap */
    / {
        keymap {
            compatible = "zmk,keymap";

            layer_Base {
                bindings = <&kp F2 & kp F3 & kp F4 & kp F5 & kp F6 & kp F7 & kp F8 & kp F9 &
                            kp F10 & kp F11 & kp F1 & kp N1 & kp N2 & kp N3 & kp N4 & kp N5 &
                            kp N6 & kp N7 & kp N8 & kp N9 & kp N0 & kp F12 & to 3 & kp Q & kp W &
                            kp E & kp R & kp T & kp Y & kp U & kp I & kp O & kp P & kp BSLH & to 2 &
                            kp A & kp S & kp D & kp F & kp G & kp H & kp J & kp K & kp L & kp SEMI &
                            kp SQT & to 1 & kp Z & kp X & kp C & kp V & kp B & kp LCTRL & kp LALT &
                            kp C_STOP & kp DEL & kp LALT & kp LCTRL & kp N & kp M & kp COMMA &
                            kp DOT & kp FSLH & kp LS(MINUS) & magic LAYER_Magic 0 & kp LBKT &
                            kp RBKT & kp TAB & kp ESC & kp SPACE & kp LSHFT & kp LGUI & kp RGUI &
                            kp LSHFT & kp RALT & kp RET & kp BSPC & kp MINUS & kp EQUAL & kp GRAVE>;
            };

            layer_Gaming {
                bindings = <&trans & trans & trans & trans & trans & trans & trans & trans & trans &
                            trans & trans & trans & trans & trans & trans & trans & trans & trans &
                            trans & trans & trans & trans & trans & kp T & kp Q & kp W & kp E &
                            kp R & trans & trans & trans & trans & trans & trans & trans & kp G &
                            kp A & kp S & kp D & kp F & trans & trans & trans & trans & trans &
                            trans & to 0 & kp B & kp Z & kp X & kp C & kp V & trans & trans &
                            trans & trans & trans & trans & trans & trans & trans & trans & trans &
                            trans & trans & kp DEL & kp BSPC & trans & trans & trans & trans &
                            kp RET & trans & trans & trans & trans & trans & trans & trans & trans>;
            };

            layer_Lower {
                bindings =
                    <&kp C_BRI_DN & kp C_BRI_UP & kp C_VOL_DN & kp C_MUTE & kp C_VOL_UP &
                     kp C_PREV & kp C_PP & kp C_NEXT & kp C_EJECT & none & none & none & none &
                     kp PG_UP & none & none & none & none & kp PG_UP & none & none & none & trans &
                     kp LC(HOME) & kp HOME & kp UP & kp END & kp LC(END) & kp LC(HOME) & kp HOME &
                     kp UP & kp END & kp LC(END) & none & to 0 & kp LC(LEFT) & kp LEFT & kp DOWN &
                     kp RIGHT & kp RC(RIGHT) & kp LC(LEFT) & kp LEFT & kp DOWN & kp RIGHT &
                     kp LC(RIGHT) & none & trans & kp DEL & kp BSPC & kp PG_DN & kp RET & none &
                     trans & trans & trans & trans & trans & trans & none & kp LA(LEFT) & kp PG_DN &
                     kp LA(RIGHT) & none & none & trans & kp INS & kp LS(INS) & trans & trans &
                     trans & trans & trans & trans & trans & trans & trans & trans & kp K_APP &
                     caps_word & none>;
            };

            layer_Numeric {
                bindings =
                    <&kp PAUSE_BREAK & none & kp CAPS & kp KP_NUM & kp SLCK & kp PSCRN &
                     kp LC(PSCRN) & kp LA(PSCRN) & kp LS(PSCRN) & kp LC(LS(PSCRN)) & none & none &
                     kp KP_N7 & kp KP_N8 & kp KP_N9 & none & none & kp KP_N7 & kp KP_N8 & kp KP_N9 &
                     none & none & to 0 & none & kp KP_N4 & kp KP_N5 & kp KP_N6 & kp KP_ENTER &
                     kp KP_ENTER & kp KP_N4 & kp KP_N5 & kp KP_N6 & none & none & trans &
                     kp KP_MULTIPLY & kp KP_N1 & kp KP_N2 & kp KP_N3 & kp KP_PLUS & kp KP_PLUS &
                     kp KP_N1 & kp KP_N2 & kp KP_N3 & kp KP_MULTIPLY & none & trans & kp KP_SLASH &
                     kp KP_N0 & kp COMMA & kp DOT & kp KP_MINUS & trans & trans & trans & trans &
                     trans & trans & kp KP_MINUS & kp KP_N0 & trans & trans & kp KP_SLASH & none &
                     trans & none & kp KP_DOT & trans & trans & trans & trans & trans & trans &
                     trans & trans & trans & trans & kp KP_DOT & none & kp C_AL_CALC>;
            };

            layer_Magic {
                bindings =
                    <&bt BT_CLR & none & none & none & none & none & none & none & none &
                     bt BT_CLR_ALL & none & none & none & none & none & none & none & none & none &
                     none & none & none & none & rgb_ug RGB_SPI & rgb_ug RGB_SAI & rgb_ug RGB_HUI &
                     rgb_ug RGB_BRI & rgb_ug RGB_TOG & none & none & none & none & none & none &
                     bootloader & rgb_ug RGB_SPD & rgb_ug RGB_SAD & rgb_ug RGB_HUD &
                     rgb_ug RGB_BRD & rgb_ug RGB_EFF & none & none & none & none & none &
                     bootloader & sys_reset & none & none & none & none & none & bt_2 & bt_3 &
                     none & none & none & none & none & none & none & none & none & sys_reset &
                     none & none & none & none & none & bt_0 & bt_1 & out OUT_USB & none & none &
                     none & none & none & none & none & none>;
            };
        };
    };
}

static void zmk_led_battery_level(int bat_level, const uint8_t *addresses, size_t addresses_len) {
    struct led_rgb bat_colour;

    if (bat_level > 40) {
        bat_colour = nice_blue;
    } else if (bat_level > 20) {
        bat_colour = yellow;
    } else {
        bat_colour = red;
    }

    // originally, six levels, 0 .. 100

    for (int i = 0; i < addresses_len; i++) {
        int min_level = (i * 100) / (addresses_len - 1);
        if (bat_level >= min_level) {
            status_pixels[addresses[i]] = bat_colour;
        }
    }
}

static void zmk_led_fill(struct led_rgb color, const uint8_t *addresses, size_t addresses_len) {
    for (int i = 0; i < addresses_len; i++) {
        status_pixels[addresses[i]] = color;
    }
}

#define ZMK_LED_NUMLOCK_BIT BIT(0)
#define ZMK_LED_CAPSLOCK_BIT BIT(1)
#define ZMK_LED_SCROLLLOCK_BIT BIT(2)

static int zmk_led_generate_status(void) {
    for (int i = 0; i < STRIP_NUM_PIXELS; i++) {
        status_pixels[i] = (struct led_rgb){r : 0, g : 0, b : 0};
    }

    // BATTERY STATUS
    zmk_led_battery_level(zmk_battery_state_of_charge(), underglow_bat_lhs,
                          DT_PROP_LEN(UNDERGLOW_INDICATORS, bat_lhs));

#if IS_ENABLED(CONFIG_ZMK_SPLIT_BLE_CENTRAL_BATTERY_LEVEL_FETCHING)
    uint8_t peripheral_level = 0;
    int rc = zmk_split_get_peripheral_battery_level(0, &peripheral_level);

    if (rc == 0) {
        zmk_led_battery_level(peripheral_level, underglow_bat_rhs,
                              DT_PROP_LEN(UNDERGLOW_INDICATORS, bat_rhs));
    } else if (rc == -ENOTCONN) {
        zmk_led_fill(red, underglow_bat_rhs, DT_PROP_LEN(UNDERGLOW_INDICATORS, bat_rhs));
    } else if (rc == -EINVAL) {
        LOG_ERR("Invalid peripheral index requested for battery level read: 0");
    }
#endif

    // CAPSLOCK/NUMLOCK/SCROLLOCK STATUS
    zmk_hid_indicators_t led_flags = zmk_hid_indicators_get_current_profile();

    if (led_flags & ZMK_LED_CAPSLOCK_BIT)
        status_pixels[DT_PROP(UNDERGLOW_INDICATORS, capslock)] = yellow;
    if (led_flags & ZMK_LED_NUMLOCK_BIT)
        status_pixels[DT_PROP(UNDERGLOW_INDICATORS, numlock)] = yellow;
    if (led_flags & ZMK_LED_SCROLLLOCK_BIT)
        status_pixels[DT_PROP(UNDERGLOW_INDICATORS, scrolllock)] = yellow;

    // LAYER STATUS
    for (uint8_t i = 0; i < DT_PROP_LEN(UNDERGLOW_INDICATORS, layer_state); i++) {
        if (zmk_keymap_layer_active(i))
            status_pixels[underglow_layer_state[i]] = lilac;
    }

    struct zmk_endpoint_instance active_endpoint = zmk_endpoints_selected();

    if (!zmk_endpoints_preferred_transport_is_active())
        status_pixels[DT_PROP(UNDERGLOW_INDICATORS, output_fallback)] = red;

    int active_ble_profile_index = zmk_ble_active_profile_index();
    for (uint8_t i = 0;
         i < MIN(ZMK_BLE_PROFILE_COUNT, DT_PROP_LEN(UNDERGLOW_INDICATORS, ble_state)); i++) {
        int8_t status = zmk_ble_profile_status(i);
        int ble_pixel = underglow_ble_state[i];
        if (status == 2 && active_endpoint.transport == ZMK_TRANSPORT_BLE &&
            active_ble_profile_index == i) { // connected AND active
            status_pixels[ble_pixel] = white;
        } else if (status == 2) { // connected
            status_pixels[ble_pixel] = nice_blue;
        } else if (status == 1) { // paired
            status_pixels[ble_pixel] = red;
        } else if (status == 0) { // unused
            status_pixels[ble_pixel] = lilac;
        }
    }

    enum zmk_usb_conn_state usb_state = zmk_usb_get_conn_state();
    if (usb_state == ZMK_USB_CONN_HID &&
        active_endpoint.transport == ZMK_TRANSPORT_USB) { // connected AND active
        status_pixels[DT_PROP(UNDERGLOW_INDICATORS, usb_state)] = white;
    } else if (usb_state == ZMK_USB_CONN_HID) { // connected
        status_pixels[DT_PROP(UNDERGLOW_INDICATORS, usb_state)] = nice_blue;
    } else if (usb_state == ZMK_USB_CONN_POWERED) { // powered
        status_pixels[DT_PROP(UNDERGLOW_INDICATORS, usb_state)] = red;
    } else if (usb_state == ZMK_USB_CONN_NONE) { // disconnected
        status_pixels[DT_PROP(UNDERGLOW_INDICATORS, usb_state)] = lilac;
    }

    int16_t blend = 256;
    if (state.status_animation_step < (500 / 25)) {
        blend = ((state.status_animation_step * 256) / (500 / 25));
    } else if (state.status_animation_step > (8000 / 25)) {
        blend = 256 - (((state.status_animation_step - (8000 / 25)) * 256) / (2000 / 25));
    }
    if (blend < 0)
        blend = 0;
    if (blend > 256)
        blend = 256;

    return blend;
}
#endif // underglow_indicators exists

static void zmk_rgb_underglow_tick(struct k_work *work) {
    switch (state.current_effect) {
    case UNDERGLOW_EFFECT_SOLID:
        zmk_rgb_underglow_effect_solid();
        break;
    case UNDERGLOW_EFFECT_BREATHE:
        zmk_rgb_underglow_effect_breathe();
        break;
    case UNDERGLOW_EFFECT_SPECTRUM:
        zmk_rgb_underglow_effect_spectrum();
        break;
    case UNDERGLOW_EFFECT_SWIRL:
        zmk_rgb_underglow_effect_swirl();
        break;
    }

    zmk_led_write_pixels();
}

K_WORK_DEFINE(underglow_tick_work, zmk_rgb_underglow_tick);

static void zmk_rgb_underglow_tick_handler(struct k_timer *timer) {
    if (!state.on) {
        return;
    }

    k_work_submit_to_queue(zmk_workqueue_lowprio_work_q(), &underglow_tick_work);
}

K_TIMER_DEFINE(underglow_tick, zmk_rgb_underglow_tick_handler, NULL);

#if IS_ENABLED(CONFIG_SETTINGS)
static int rgb_settings_set(const char *name, size_t len, settings_read_cb read_cb, void *cb_arg) {
    const char *next;
    int rc;

    if (settings_name_steq(name, "state", &next) && !next) {
        if (len != sizeof(state)) {
            return -EINVAL;
        }

        rc = read_cb(cb_arg, &state, sizeof(state));
        if (rc >= 0) {
            return 0;
        }

        return rc;
    }

    return -ENOENT;
}

struct settings_handler rgb_conf = {.name = "rgb/underglow", .h_set = rgb_settings_set};

static void zmk_rgb_underglow_save_state_work(struct k_work *_work) {
    settings_save_one("rgb/underglow/state", &state, sizeof(state));
}

static struct k_work_delayable underglow_save_work;
#endif

static int zmk_rgb_underglow_init(void) {
    led_strip = DEVICE_DT_GET(STRIP_CHOSEN);

#if IS_ENABLED(CONFIG_ZMK_RGB_UNDERGLOW_EXT_POWER)
    if (!device_is_ready(ext_power)) {
        LOG_ERR("External power device \"%s\" is not ready", ext_power->name);
        return -ENODEV;
    }
#endif

    state = (struct rgb_underglow_state){
        color : {
            h : CONFIG_ZMK_RGB_UNDERGLOW_HUE_START,
            s : CONFIG_ZMK_RGB_UNDERGLOW_SAT_START,
            b : CONFIG_ZMK_RGB_UNDERGLOW_BRT_START,
        },
        animation_speed : CONFIG_ZMK_RGB_UNDERGLOW_SPD_START,
        current_effect : CONFIG_ZMK_RGB_UNDERGLOW_EFF_START,
        animation_step : 0,
        on : IS_ENABLED(CONFIG_ZMK_RGB_UNDERGLOW_ON_START)
    };

#if IS_ENABLED(CONFIG_SETTINGS)
    settings_subsys_init();

    int err = settings_register(&rgb_conf);
    if (err) {
        LOG_ERR("Failed to register the ext_power settings handler (err %d)", err);
        return err;
    }

    k_work_init_delayable(&underglow_save_work, zmk_rgb_underglow_save_state_work);

    settings_load_subtree("rgb/underglow");
#endif

#if IS_ENABLED(CONFIG_ZMK_RGB_UNDERGLOW_AUTO_OFF_USB)
    state.on = zmk_usb_is_powered();
#endif

    if (state.on) {
        k_timer_start(&underglow_tick, K_NO_WAIT, K_MSEC(25));
    }

    return 0;
}

int zmk_rgb_underglow_save_state(void) {
#if IS_ENABLED(CONFIG_SETTINGS)
    int ret = k_work_reschedule(&underglow_save_work, K_MSEC(CONFIG_ZMK_SETTINGS_SAVE_DEBOUNCE));
    return MIN(ret, 0);
#else
    return 0;
#endif
}

int zmk_rgb_underglow_get_state(bool *on_off) {
    if (!led_strip)
        return -ENODEV;

    *on_off = state.on;
    return 0;
}

void zmk_rgb_set_ext_power(void) {
#if IS_ENABLED(CONFIG_ZMK_RGB_UNDERGLOW_EXT_POWER)
    if (ext_power == NULL)
        return;
    int c_power = ext_power_get(ext_power);
    if (c_power < 0) {
        LOG_ERR("Unable to examine EXT_POWER: %d", c_power);
        c_power = 0;
    }
    int desired_state = state.on || state.status_active;
    // force power off, when battery low (<10%)
    if (state.on && !state.status_active) {
        if (zmk_battery_state_of_charge() < 10) {
            desired_state = false;
        }
    }
    if (desired_state && !c_power) {
        int rc = ext_power_enable(ext_power);
        if (rc != 0) {
            LOG_ERR("Unable to enable EXT_POWER: %d", rc);
        }
    } else if (!desired_state && c_power) {
        int rc = ext_power_disable(ext_power);
        if (rc != 0) {
            LOG_ERR("Unable to disable EXT_POWER: %d", rc);
        }
    }
#endif
}

int zmk_rgb_underglow_on(void) {
    if (!led_strip)
        return -ENODEV;

    state.on = true;
    zmk_rgb_set_ext_power();

    state.animation_step = 0;
    k_timer_start(&underglow_tick, K_NO_WAIT, K_MSEC(25));

    return zmk_rgb_underglow_save_state();
}

static void zmk_rgb_underglow_off_handler(struct k_work *work) {
    for (int i = 0; i < STRIP_NUM_PIXELS; i++) {
        pixels[i] = (struct led_rgb){r : 0, g : 0, b : 0};
    }
    zmk_led_write_pixels();
}

K_WORK_DEFINE(underglow_off_work, zmk_rgb_underglow_off_handler);

int zmk_rgb_underglow_off(void) {
    if (!led_strip)
        return -ENODEV;

    k_work_submit_to_queue(zmk_workqueue_lowprio_work_q(), &underglow_off_work);

    k_timer_stop(&underglow_tick);
    state.on = false;
    zmk_rgb_set_ext_power();

    return zmk_rgb_underglow_save_state();
}

int zmk_rgb_underglow_calc_effect(int direction) {
    return (state.current_effect + UNDERGLOW_EFFECT_NUMBER + direction) % UNDERGLOW_EFFECT_NUMBER;
}

int zmk_rgb_underglow_select_effect(int effect) {
    if (!led_strip)
        return -ENODEV;

    if (effect < 0 || effect >= UNDERGLOW_EFFECT_NUMBER) {
        return -EINVAL;
    }

    state.current_effect = effect;
    state.animation_step = 0;

    return zmk_rgb_underglow_save_state();
}

int zmk_rgb_underglow_cycle_effect(int direction) {
    return zmk_rgb_underglow_select_effect(zmk_rgb_underglow_calc_effect(direction));
}

int zmk_rgb_underglow_toggle(void) {
    return state.on ? zmk_rgb_underglow_off() : zmk_rgb_underglow_on();
}

static void zmk_led_write_pixels_work(struct k_work *work);
static void zmk_rgb_underglow_status_update(struct k_timer *timer);

K_WORK_DEFINE(underglow_write_work, zmk_led_write_pixels_work);
K_TIMER_DEFINE(underglow_status_update_timer, zmk_rgb_underglow_status_update, NULL);

static void zmk_rgb_underglow_status_update(struct k_timer *timer) {
    if (!state.status_active)
        return;
    state.status_animation_step++;
    if (state.status_animation_step > (10000 / 25)) {
        state.status_active = false;
        k_timer_stop(&underglow_status_update_timer);
    }
    if (!k_work_is_pending(&underglow_write_work))
        k_work_submit(&underglow_write_work);
}

static void zmk_led_write_pixels_work(struct k_work *work) {
    zmk_led_write_pixels();
    if (!state.status_active) {
        zmk_rgb_set_ext_power();
    }
}

int zmk_rgb_underglow_status(void) {
    if (!state.status_active) {
        state.status_animation_step = 0;
    } else {
        if (state.status_animation_step > (500 / 25)) {
            state.status_animation_step = 500 / 25;
        }
    }
    state.status_active = true;
    zmk_led_write_pixels();
    zmk_rgb_set_ext_power();

    k_timer_start(&underglow_status_update_timer, K_NO_WAIT, K_MSEC(25));

    return 0;
}

int zmk_rgb_underglow_set_hsb(struct zmk_led_hsb color) {
    if (color.h > HUE_MAX || color.s > SAT_MAX || color.b > BRT_MAX) {
        return -ENOTSUP;
    }

    state.color = color;

    return 0;
}

struct zmk_led_hsb zmk_rgb_underglow_calc_hue(int direction) {
    struct zmk_led_hsb color = state.color;

    color.h += HUE_MAX + (direction * CONFIG_ZMK_RGB_UNDERGLOW_HUE_STEP);
    color.h %= HUE_MAX;

    return color;
}

struct zmk_led_hsb zmk_rgb_underglow_calc_sat(int direction) {
    struct zmk_led_hsb color = state.color;

    int s = color.s + (direction * CONFIG_ZMK_RGB_UNDERGLOW_SAT_STEP);
    if (s < 0) {
        s = 0;
    } else if (s > SAT_MAX) {
        s = SAT_MAX;
    }
    color.s = s;

    return color;
}

struct zmk_led_hsb zmk_rgb_underglow_calc_brt(int direction) {
    struct zmk_led_hsb color = state.color;

    int b = color.b + (direction * CONFIG_ZMK_RGB_UNDERGLOW_BRT_STEP);
    color.b = CLAMP(b, 0, BRT_MAX);

    return color;
}

int zmk_rgb_underglow_change_hue(int direction) {
    if (!led_strip)
        return -ENODEV;

    state.color = zmk_rgb_underglow_calc_hue(direction);

    return zmk_rgb_underglow_save_state();
}

int zmk_rgb_underglow_change_sat(int direction) {
    if (!led_strip)
        return -ENODEV;

    state.color = zmk_rgb_underglow_calc_sat(direction);

    return zmk_rgb_underglow_save_state();
}

int zmk_rgb_underglow_change_brt(int direction) {
    if (!led_strip)
        return -ENODEV;

    state.color = zmk_rgb_underglow_calc_brt(direction);

    return zmk_rgb_underglow_save_state();
}

int zmk_rgb_underglow_change_spd(int direction) {
    if (!led_strip)
        return -ENODEV;

    if (state.animation_speed == 1 && direction < 0) {
        return 0;
    }

    state.animation_speed += direction;

    if (state.animation_speed > 5) {
        state.animation_speed = 5;
    }

    return zmk_rgb_underglow_save_state();
}

#if IS_ENABLED(CONFIG_ZMK_RGB_UNDERGLOW_AUTO_OFF_IDLE) ||                                          \
    IS_ENABLED(CONFIG_ZMK_RGB_UNDERGLOW_AUTO_OFF_USB)
static int rgb_underglow_auto_state(bool *prev_state, bool new_state) {
    if (state.on == new_state) {
        return 0;
    }
    if (new_state) {
        state.on = *prev_state;
        *prev_state = false;
        return zmk_rgb_underglow_on();
    } else {
        state.on = false;
        *prev_state = true;
        return zmk_rgb_underglow_off();
    }
}

static int rgb_underglow_event_listener(const zmk_event_t *eh) {

#if IS_ENABLED(CONFIG_ZMK_RGB_UNDERGLOW_AUTO_OFF_IDLE)
    if (as_zmk_activity_state_changed(eh)) {
        static bool prev_state = false;
        return rgb_underglow_auto_state(&prev_state,
                                        zmk_activity_get_state() == ZMK_ACTIVITY_ACTIVE);
    }
#endif

#if IS_ENABLED(CONFIG_ZMK_RGB_UNDERGLOW_AUTO_OFF_USB)
    if (as_zmk_usb_conn_state_changed(eh)) {
        static bool prev_state = false;
        return rgb_underglow_auto_state(&prev_state, zmk_usb_is_powered());
    }
#endif

    return -ENOTSUP;
}

ZMK_LISTENER(rgb_underglow, rgb_underglow_event_listener);
#endif // IS_ENABLED(CONFIG_ZMK_RGB_UNDERGLOW_AUTO_OFF_IDLE) ||
       // IS_ENABLED(CONFIG_ZMK_RGB_UNDERGLOW_AUTO_OFF_USB)

#if IS_ENABLED(CONFIG_ZMK_RGB_UNDERGLOW_AUTO_OFF_IDLE)
ZMK_SUBSCRIPTION(rgb_underglow, zmk_activity_state_changed);
#endif

#if IS_ENABLED(CONFIG_ZMK_RGB_UNDERGLOW_AUTO_OFF_USB)
ZMK_SUBSCRIPTION(rgb_underglow, zmk_usb_conn_state_changed);
#endif

SYS_INIT(zmk_rgb_underglow_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
