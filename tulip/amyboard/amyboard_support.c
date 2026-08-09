#include <stdio.h>
#include <inttypes.h>
#include <string.h>
#include <math.h>
#include "amy.h"

// Stuff just for amyboard -- cv in/out direct , i2c in/out

#ifdef ESP_PLATFORM
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/message_buffer.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "esp_task.h"
#include "driver/i2c.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "driver/i2s_std.h"


#include "pins.h"


TaskHandle_t i2c_check_for_data_handle;


#define AMY_I2C_CHECK_FOR_DATA_TASK_COREID (1)
#define ALLES_I2C_CHECK_FOR_DATA_TASK_PRIORITY (ESP_TASK_PRIO_MAX-1)
#define ALLES_I2C_CHECK_FOR_DATA_TASK_NAME "amyboard_i2c"
#define ALLES_I2C_CHECK_FOR_DATA_TASK_STACK_SIZE (16 * 1024)


// i2c stuff
#define I2C_CLK_FREQ 400000
#define DATA_LENGTH MAX_MESSAGE_LEN
#define _I2C_NUMBER(num) I2C_NUM_##num
#define I2C_NUMBER(num) _I2C_NUMBER(num)
#define I2C_FOLLOWER_NUM I2C_NUMBER(1) /*!< I2C port number for follower dev */
#define I2C_FOLLOWER_TX_BUF_LEN (2 * DATA_LENGTH)              /*!< I2C follower tx buffer size */
#define I2C_FOLLOWER_RX_BUF_LEN (2 * DATA_LENGTH)              /*!< I2C follower rx buffer size */

#define AMYCHIP_ADDR 0x3F
#define PCM9211_ADDR 0x40
#define ADS1015_ADDR 0x48
#define GP8413_ADDR  0x58

esp_err_t i2c_follower_init() {
    i2c_port_t i2c_follower_port = I2C_FOLLOWER_NUM;
    i2c_config_t conf_follower;
    conf_follower.sda_io_num = I2C_FOLLOWER_SDA;
    conf_follower.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf_follower.scl_io_num = I2C_FOLLOWER_SCL;
    conf_follower.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf_follower.mode = I2C_MODE_SLAVE;
    conf_follower.slave.addr_10bit_en = 0;
    conf_follower.slave.slave_addr = AMYCHIP_ADDR;
    conf_follower.slave.maximum_speed = I2C_CLK_FREQ; // expected maximum clock speed
    conf_follower.clk_flags =0;
    i2c_param_config(i2c_follower_port, &conf_follower);
    return i2c_driver_install(i2c_follower_port, conf_follower.mode,
                            I2C_FOLLOWER_RX_BUF_LEN, I2C_FOLLOWER_TX_BUF_LEN, 0);
}


uint8_t i2c_buffer[MAX_MESSAGE_LEN];

void i2c_check_for_data() {
    while(1) {
        size_t size = i2c_slave_read_buffer(I2C_FOLLOWER_NUM, i2c_buffer, MAX_MESSAGE_LEN, 10 / portTICK_PERIOD_MS);
        if(size>0) {
            i2c_buffer[size] = 0;
            //fprintf(stderr, "%s\n", i2c_buffer);
            amy_add_message((char*)i2c_buffer);
        }
    }
}



// ---------------------------------------------------------------------------
// Background I2C write queue.  The OLED framebuffer flush takes ~200ms of bus
// time at 400kHz; done synchronously from MicroPython it froze Python (and
// starved the CV tasks' port mutex) for that long.  Python instead enqueues
// each transaction here and returns immediately; this low-priority task plays
// them out in order on I2C_NUM_0.  The legacy IDF driver's per-port mutex
// serializes us against machine.I2C and the CV read/write hooks, and because
// Python chunks large payloads into small transactions, those users interleave
// between chunks instead of timing out.  Single producer (the MicroPython
// task) / single consumer (this task), which is all a MessageBuffer allows.

#define I2C_BG_QUEUE_BYTES 16384      // > one full 8KB OLED frame incl. overhead
#define I2C_BG_MAX_PAYLOAD 1024       // per-transaction cap (Python sends ~256B)
#define I2C_BG_TASK_STACK_SIZE 4096
#define I2C_BG_TASK_PRIORITY (ESP_TASK_PRIO_MIN + 1)  // same as cv_read_task
#define I2C_BG_TASK_COREID (0)                        // MicroPython runs on core 1

static MessageBufferHandle_t i2c_bg_mb = NULL;
static volatile uint32_t i2c_bg_errors_count = 0;
static volatile uint8_t i2c_bg_in_flight = 0;

static void i2c_bg_task(void *pvParameter) {
    static uint8_t msg[I2C_BG_MAX_PAYLOAD + 1];  // [addr][payload...]
    for(;;) {
        size_t n = xMessageBufferReceive(i2c_bg_mb, msg, sizeof(msg), portMAX_DELAY);
        if(n < 2) continue;
        i2c_bg_in_flight = 1;
        esp_err_t ret = i2c_master_write_to_device(I2C_NUM_0, msg[0], msg + 1, n - 1, pdMS_TO_TICKS(100));
        i2c_bg_in_flight = 0;
        if(ret != ESP_OK) i2c_bg_errors_count++;
    }
}

// Enqueue one I2C write transaction. Returns 0 on success, 1 if it had to be
// dropped (queue stayed full / payload too big) -- the failure also bumps
// i2c_bg_errors() so Python can schedule a full panel resync.
uint8_t amyboard_i2c_bg_write(uint8_t addr, const uint8_t *buf, uint32_t len) {
    // Only ever called from the MicroPython task (single producer).
    static uint8_t msg[I2C_BG_MAX_PAYLOAD + 1];
    if(len == 0 || len > I2C_BG_MAX_PAYLOAD) {
        i2c_bg_errors_count++;
        return 1;
    }
    if(i2c_bg_mb == NULL) {
        i2c_bg_mb = xMessageBufferCreate(I2C_BG_QUEUE_BYTES);
        xTaskCreatePinnedToCore(i2c_bg_task, "i2c_bg", I2C_BG_TASK_STACK_SIZE / sizeof(StackType_t),
                                NULL, I2C_BG_TASK_PRIORITY, NULL, I2C_BG_TASK_COREID);
    }
    msg[0] = addr;
    memcpy(msg + 1, buf, len);
    // Blocking here is backpressure: it only happens when more than a whole
    // queued frame is outstanding, and it bounds how far Python can run ahead.
    if(xMessageBufferSend(i2c_bg_mb, msg, len + 1, pdMS_TO_TICKS(500)) != len + 1) {
        i2c_bg_errors_count++;
        return 1;
    }
    return 0;
}

// Bytes still queued (plus any transaction currently on the wire); 0 == idle.
uint32_t amyboard_i2c_bg_pending(void) {
    if(i2c_bg_mb == NULL) return 0;
    uint32_t queued = I2C_BG_QUEUE_BYTES - (uint32_t)xMessageBufferSpacesAvailable(i2c_bg_mb);
    return queued + i2c_bg_in_flight;
}

uint32_t amyboard_i2c_bg_errors(void) {
    return i2c_bg_errors_count;
}

#define ADS1015_REGISTER_CONFIG (0x01)
#define ADS1015_CQUE_DISABLE (0x0003)

#define ADS1015_CLAT_NONLAT (0x0000)
#define ADS1015_CPOL_ACTVLOW (0x0000)
#define ADS1015_CMODE_TRAD (0x0000)
#define ADS1015_DR_3300SPS (0x00E0)  // 0x00E0 = 3300 SPS (max) on the ADS1015
#define ADS1015_MODE_SINGLE (0x0100)
#define ADS1015_MODE_CONT (0x0100)
#define ADS1015_OS_SINGLE (0x8000)  // initiate single conversion / check converter status
#define ADS1015_OS_READY (0x8000) // OS bit reads 1 when conversion is complete
#define ADS1015_PGA_2_048V (0x0400)
//#define ADS1015_PGA_4_096V (0x0200)
#define ADS1015_MUX_SINGLE_0 (0x4000)
//#define ADS1015_MUX_PER_CHAN (0x1000)
// To get the offset to ADS1015_MUX_SINGLE_0 for chan C, use (C << ADS1015_MUX_CHAN_SHIFTL)
#define ADS1015_MUX_CHAN_SHIFTL (12)
//#define ADS1015_MUX_SINGLE_1 (0x5000)
//#define ADS1015_MUX_SINGLE_2 (0x6000)
//#define ADS1015_MUX_SINGLE_3 (0x7000)
#define ADS1015_REGISTER_CONVERT (0x00)

static esp_err_t ads1015_write_register(uint8_t reg, uint16_t data) {
    i2c_cmd_handle_t cmd;
    esp_err_t ret;
    uint8_t out[2];

    out[0] = data >> 8; // get 8 greater bits
    out[1] = data & 0xFF; // get 8 lower bits
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd); // generate a start command
    i2c_master_write_byte(cmd,(ADS1015_ADDR<<1) | I2C_MASTER_WRITE,1); // specify address and write command
    i2c_master_write_byte(cmd,reg,1); // specify register
    i2c_master_write(cmd,out,2,1); // write it
    i2c_master_stop(cmd); // generate a stop command
    ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(10)); // send the i2c command
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t ads1015_read_register(uint8_t reg, uint8_t* data, uint8_t len) {
    i2c_cmd_handle_t cmd;
    esp_err_t ret;

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd,(ADS1015_ADDR<<1) | I2C_MASTER_WRITE,1);
    i2c_master_write_byte(cmd,reg,1);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(10));
    i2c_cmd_link_delete(cmd);

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd); // generate start command
    i2c_master_write_byte(cmd,(ADS1015_ADDR<<1) | I2C_MASTER_READ,1); // specify address and read command
    i2c_master_read(cmd, data, len, 0); // read all wanted data
    i2c_master_stop(cmd); // generate stop command
    ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(10)); // send the i2c command
    i2c_cmd_link_delete(cmd);
    return ret;
}

static int ads1015_pending_channel = -1;

void ads1015_start_conversion(uint8_t channel) {
    uint16_t channel_mux = ADS1015_MUX_SINGLE_0 + (channel << ADS1015_MUX_CHAN_SHIFTL);
    uint16_t data = (ADS1015_CQUE_DISABLE | ADS1015_CLAT_NONLAT |
                     ADS1015_CPOL_ACTVLOW | ADS1015_CMODE_TRAD | ADS1015_DR_3300SPS |
                     ADS1015_MODE_SINGLE | ADS1015_OS_SINGLE | ADS1015_PGA_2_048V |
                     channel_mux);
    ads1015_write_register(ADS1015_REGISTER_CONFIG, data);
    ads1015_pending_channel = channel;
}

uint16_t ads1015_get_result(void) {
    // Wait for the single-shot conversion on the last-selected mux channel to
    // finish before reading the result. The OS bit reads 0 while converting and
    // 1 when done; at 3300 SPS each conversion takes ~0.3ms. Without this wait the
    // CONVERT register still holds the *previous* conversion (the other channel),
    // which made cv_in(0) and cv_in(1) return the same input. Bound the poll so a
    // missing/unresponsive ADC can't stall the cv_read_task forever.
    uint8_t buffer[2];
    for(int i = 0; i < 20; i++) {
        if(ads1015_read_register(ADS1015_REGISTER_CONFIG, buffer, 2) != ESP_OK) break;
        if((((uint16_t)buffer[0] << 8) | buffer[1]) & ADS1015_OS_READY) break; // conversion done
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    ads1015_read_register(ADS1015_REGISTER_CONVERT, buffer, 2);
    return ((uint16_t)buffer[0] << 8) | (uint16_t)buffer[1];
}

// Which channel to speculatively convert after a read. -1 = the other one,
// which suits the default alternating scan; the CV quantizer (below) sets it
// to its own input, which it reads every tick -- re-reading a channel whose
// conversion is already done skips the ~1ms wait for a fresh one.
static int ads1015_speculate_channel = -1;

uint16_t read_ads1015_raw(uint8_t channel) {
    if (channel != ads1015_pending_channel) {
        ads1015_start_conversion(channel);
        // The ADS1015 takes ~25us to wake from single-shot shutdown, during
        // which OS still reads "idle" and CONVERT still holds the *previous*
        // conversion -- the other channel's. Polling immediately can win that
        // race and hand channel A a copy of channel B (seen on hardware as
        // the chord following the quantizer's CV). One tick guarantees the
        // fresh conversion (0.3ms at 3300 SPS) is underway or already done.
        // The default alternating scan never takes this branch after boot,
        // so it costs nothing outside the quantizer's fast mode.
        vTaskDelay(1);
    }
    uint16_t result = ads1015_get_result();
    // Speculatively start the next conversion.  Assumes we're just using channels 0 and 1.
    ads1015_start_conversion(ads1015_speculate_channel >= 0 ?
                             (uint8_t)ads1015_speculate_channel : (channel + 1) % 2);
    return result;
}


#endif // ESP_PLATFORM

extern uint8_t * external_map;
float cv_local_value[2];
uint8_t cv_local_override[2];

// Cached CV input values — updated by a dedicated FreeRTOS task so the
// audio render thread never blocks on I2C.
float cv_cached_value[2] = {0, 0};

// Called from the coef hook on the audio thread — just returns the cached value.
float cv_input_hook(uint16_t channel) {
    if(cv_local_override[channel]) {
        return cv_local_value[channel];
    }
    return cv_cached_value[channel];
}

#ifdef ESP_PLATFORM
// ---------------------------------------------------------------------------
// CV pitch quantizer, run inside cv_read_task (cvquant fork).
//
// A MicroPython sketch quantizing CV can only answer on the AMY sequencer
// grid (~5ms at best), on top of this task's scan period -- ~7-12ms in all.
// Here the whole read->quantize->write chain runs in this task instead, and
// the scan period drops to 1 tick while enabled, so a CV step is answered in
// ~1-2ms. The sketch stays in charge of *what* is allowed: it pushes a
// pitch-class mask and its own calibration fits down via tulip.cv_quant_*()
// and reads the held note back for its display.
//
// Concurrency: the tulip.cv_quant_*() bindings run on the MicroPython task,
// this loop on its own task. Every config field is a single aligned 32-bit-
// or-smaller store, and a step that sees a half-pushed config produces at
// most one scan's worth of output through mixed old/new values -- the next
// step uses the settled config. The one 64-bit value (the trigger deadline)
// is only ever torn into "a pulse ends early", which writes an already-low
// line low again.

typedef struct {
    volatile uint8_t enabled;
    volatile uint16_t mask;       // bit p set = pitch class p allowed; 0 = hold
    volatile float in_slope, in_offset;  // sketch's CV-in fit: real = s*raw + o
    volatile float pitch_a, pitch_b;     // sketch's CV-out fit: true = a*cmd + b
    volatile float trig_a, trig_b;
    volatile uint8_t in_ch, pitch_ch, trig_ch;
    volatile float trig_volts;
    volatile int32_t trig_ms;
    volatile float hyst;          // semitones; move only when this much closer
} cv_quant_cfg_t;

static cv_quant_cfg_t qcfg = {
    .enabled = 0, .mask = 0,
    .in_slope = 1.0f, .in_offset = 0.0f,
    .pitch_a = 1.0f, .pitch_b = 0.0f,
    .trig_a = 1.0f, .trig_b = 0.0f,
    .in_ch = 1, .pitch_ch = 1, .trig_ch = 0,
    .trig_volts = 5.0f, .trig_ms = 10,
    .hyst = 0.6f,
};

#define CV_QUANT_NONE INT16_MIN
static volatile int16_t q_held = CV_QUANT_NONE;  // semitones above C at 0V
static int16_t q_sent = CV_QUANT_NONE;
static volatile int64_t q_trig_until_us = -1;

// While the quantizer is enabled the scan runs every tick (1ms) instead of
// once per audio block; 0 means "use the default period". The extra I2C
// traffic (~40% bus duty) is only paid while a quant mode is active.
static volatile TickType_t cv_scan_ticks = 0;

// Write real volts to a GP8413 channel. Same scaling as amyboard.cv_out():
// 0x0000 -> -10V nominal, 0x7fff -> +10V, low byte first after the register.
static void gp8413_write_volts(uint8_t channel, float volts) {
    int32_t val = (int32_t)(((volts + 10.0f) / 20.0f) * 0x8000);
    if(val < 0) val = 0;
    if(val > 0x7fff) val = 0x7fff;
    uint8_t bytes[3];
    bytes[0] = (channel == 1) ? 0x04 : 0x02;
    bytes[1] = val & 0xff;
    bytes[2] = (val >> 8) & 0xff;
    i2c_master_write_to_device(I2C_NUM_0, GP8413_ADDR, bytes, 3, pdMS_TO_TICKS(10));
}

// Volts out through the sketch's fitted correction (its cv_out_volts()).
static void cv_quant_out(uint8_t channel, float volts, float a, float b) {
    gp8413_write_volts(channel, (volts - b) / a);
}

static void cv_quant_step(void) {
    int64_t now = esp_timer_get_time();
    // End an expired trigger pulse even when disabled mid-pulse.
    if(q_trig_until_us >= 0 && now >= q_trig_until_us) {
        cv_quant_out(qcfg.trig_ch, 0.0f, qcfg.trig_a, qcfg.trig_b);
        q_trig_until_us = -1;
    }
    if(!qcfg.enabled) return;
    uint16_t mask = qcfg.mask;
    if(mask == 0) return;             // e.g. mid patch-load: hold the pitch
    float x = (cv_input_hook(qcfg.in_ch) * qcfg.in_slope + qcfg.in_offset) * 12.0f;
    // Nearest allowed semitone: any non-empty pitch-class set repeats every
    // 12, so +/-6 around the rounded input always contains the nearest. Scan
    // upward with strict <, so ties resolve to the lower note -- the same
    // note the sketch's Python quantizer picks.
    int centre = (int)lroundf(x);
    int best = 0;
    float best_d = 1e9f;
    for(int n = centre - 6; n <= centre + 6; n++) {
        int pc = ((n % 12) + 12) % 12;
        if(mask & (1 << pc)) {
            float d = fabsf(x - (float)n);
            if(d < best_d) { best_d = d; best = n; }
        }
    }
    int16_t held = q_held;
    if(held == CV_QUANT_NONE || !(mask & (1 << ((((int)held % 12) + 12) % 12)))) {
        held = (int16_t)best;         // held note left the allowed set: re-snap
    } else if(best != held && fabsf(x - (float)held) - best_d >= qcfg.hyst) {
        held = (int16_t)best;         // move only once meaningfully closer
    }
    q_held = held;
    if(held != q_sent) {
        // Pitch first, trigger second: an envelope fired from the trigger
        // must open onto the new note, never the old one. Same invariant as
        // the sketch's Python quantizer.
        cv_quant_out(qcfg.pitch_ch, (float)held / 12.0f, qcfg.pitch_a, qcfg.pitch_b);
        q_sent = held;
        if(q_trig_until_us >= 0)      // live pulse: drop it for a fresh edge
            cv_quant_out(qcfg.trig_ch, 0.0f, qcfg.trig_a, qcfg.trig_b);
        cv_quant_out(qcfg.trig_ch, qcfg.trig_volts, qcfg.trig_a, qcfg.trig_b);
        q_trig_until_us = now + (int64_t)qcfg.trig_ms * 1000;
    }
}

// ---- the tulip.cv_quant_*() surface, called from modtulip.c ----

void amyboard_cv_quant_config(uint8_t in_ch, float in_slope, float in_offset,
                              uint8_t pitch_ch, float pitch_a, float pitch_b,
                              uint8_t trig_ch, float trig_a, float trig_b,
                              float trig_volts, int32_t trig_ms, float hyst) {
    qcfg.in_ch = in_ch & 1;      qcfg.in_slope = in_slope;  qcfg.in_offset = in_offset;
    qcfg.pitch_ch = pitch_ch & 1; qcfg.pitch_a = pitch_a;   qcfg.pitch_b = pitch_b;
    qcfg.trig_ch = trig_ch & 1;  qcfg.trig_a = trig_a;      qcfg.trig_b = trig_b;
    qcfg.trig_volts = trig_volts; qcfg.trig_ms = trig_ms;   qcfg.hyst = hyst;
}

void amyboard_cv_quant_mask(uint16_t mask) {
    qcfg.mask = mask & 0x0fff;
}

void amyboard_cv_quant_enable(uint8_t on) {
    if(on) {
        q_held = CV_QUANT_NONE;   // first step sends the pitch and a trigger
        q_sent = CV_QUANT_NONE;
        ads1015_speculate_channel = qcfg.in_ch;
        qcfg.enabled = 1;
        cv_scan_ticks = 1;
    } else {
        qcfg.enabled = 0;
        cv_scan_ticks = 0;
        ads1015_speculate_channel = -1;
        // Let an in-flight step finish before cleaning up after it -- the
        // scan period while enabled is 1 tick, so 3ms always covers one.
        // Callers (the sketch's calibration walkthrough) rely on the outputs
        // being theirs once this returns.
        vTaskDelay(pdMS_TO_TICKS(3));
        if(q_trig_until_us >= 0) {
            cv_quant_out(qcfg.trig_ch, 0.0f, qcfg.trig_a, qcfg.trig_b);
            q_trig_until_us = -1;
        }
        q_held = CV_QUANT_NONE;
        q_sent = CV_QUANT_NONE;
    }
}

int amyboard_cv_quant_note(void) {
    int16_t held = q_held;
    return (held == CV_QUANT_NONE) ? INT16_MIN : (int)held;
}

// FreeRTOS task: reads both ADS1015 channels in a loop, updates cv_cached_value.
void cv_read_task(void *pvParameter) {
    // Bench-calibrated (loopback vs multimeter, 2026-08-07): raw = 20080 + 2003*V.
    // The ADC saturates at raw 32752 / raw 0, so the readable window is about
    // -10V to +6.3V -- inputs above +6.3V clip (the jack itself is fine to ±10V).
    int32_t min = 10064; // -5V
    int32_t max = 30096; // +5V
    // Scan both CV channels once per AMY audio block (AMY_BLOCK_SIZE / AMY_SAMPLE_RATE,
    // ~5.8ms at 256/44100) so CV tracks the audio block cadence. Expressed in RTOS ticks,
    // rounded to nearest, so it follows the audio rate regardless of tick rate; clamped to
    // >=1 tick. (At configTICK_RATE_HZ=1000 this is 6 ticks; 5.8ms can't be hit exactly.)
    TickType_t cv_period = (AMY_BLOCK_SIZE * configTICK_RATE_HZ + AMY_SAMPLE_RATE / 2) / AMY_SAMPLE_RATE;
    if(cv_period == 0) cv_period = 1;
    // xTaskDelayUntil holds a fixed period regardless of how long the two ADS1015
    // conversions take, unlike vTaskDelay which would add the read time on top.
    TickType_t last_wake = xTaskGetTickCount();
    uint32_t pass = 0;
    for(;;) {
        TickType_t fast = cv_scan_ticks;   // snapshot: quantizer enabled?
        for(uint8_t ch = 0; ch < 2; ch++) {
            // In fast mode only the quantizer's input runs every tick; the
            // other channel keeps the audio-block cadence, so the extra I2C
            // load (which competes with the OLED's background writes) stays
            // about half of what scanning both would cost.
            if(fast && ch != qcfg.in_ch && (pass % (uint32_t)cv_period)) continue;
            if(!cv_local_override[ch]) {
                int32_t raw = read_ads1015_raw(ch);  // Put uint16_t into int32_t.
                // Map [min, max] -> [-5v, +5v]
                cv_cached_value[ch] = (
                    (((float)(raw - min))
                     / ((float)(max - min)))
                    * 10.0
                ) - 5.0;
            }
        }
        cv_quant_step();
        pass++;
        // 1ms while the quantizer is enabled, the audio-block period otherwise.
        xTaskDelayUntil(&last_wake, fast ? fast : cv_period);
    }
}
#endif

