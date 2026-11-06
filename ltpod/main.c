/*
 * Copyright (C) 2026 The LineageOS Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define LOG_TAG "vendor.ltpod"

#include <android/log.h>
#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

/*
 * vendor.ltpod - LTPO / AOD refresh rate driver for OPLUS AA567 panels
 *
 * 1) Screen-on LTPO
 *    msm_drm.ko resets sa_min_fps to 120 on every power-mode / resolution / fps
 *    switch (oplus_adfr_status_reset), and oplus_adfr_idle_mode_handle skips the
 *    idle step-down while sa_min_fps >= 60. Writing 1 to
 *    /sys/kernel/oplus_display/min_fps lets the panel idle down to 1Hz. That
 *    node ships as root:root 0644, hence the daemon runs as root.
 *
 * 2) AOD 1Hz (ultra low power AOD)
 *    Writing ultra_low_power_aod_mode does not work on AOSP-based ROMs: the attr
 *    sets is_ultra_low_power_aod_on, and the next sde_encoder_kickoff() calls
 *    oplus_ofp_ultra_low_power_aod_update(), which keeps ULP only while
 *    (ofp_prop & 0x281) == 0x80 - a bit only ColorOS' proprietary SF/HWC sets.
 *    The kernel therefore sends ultra-low-power-aod-off-command straight back
 *    (measured ULP windows 0.07s - 6.1s, average still 30Hz).
 *
 *    Pushing the same command through the raw DSI path (dsi_cmd) does not touch
 *    is_ultra_low_power_aod_on, so the driver believes ULP is off and the
 *    kickoff off-branch stays a no-op. Measured on device: 30.3Hz -> 1.0-1.2Hz,
 *    confirmed by the panel register (page0x02 0xB1 = ...1d, page0x2d 0xD0 =
 *    0x03) and by the raw TE irq counter.
 *
 *    CRITICAL: the matching off command must be sent when leaving AOD. A panel
 *    left in ULP does not recover by itself - the next AOD entry stays at
 *    ~120Hz with normal backlight and a wrong (yellow-green) gamma.
 *
 *    CRITICAL: do not push ULP too early. A manual power-button lock still runs
 *    the lock/doze animation after STATE_DOZE; entering ULP mid-animation
 *    freezes the AOD. 10s settle is deliberate.
 *
 * 3) Hooking into Settings
 *    The "LTPO" switch in crDroid Settings is com.crdroid.ltpo.oplus, which
 *    writes 0x109f (on) or 0x0 (off) to adfr_config. Reading that node is enough
 *    to follow the switch.
 */

#define DISPLAY_DIR "/sys/kernel/oplus_display"

#define PATH_POWER_STATUS DISPLAY_DIR "/power_status"
#define PATH_ADFR_CONFIG DISPLAY_DIR "/adfr_config"
#define PATH_MIN_FPS DISPLAY_DIR "/min_fps"
#define PATH_DSI_CMD DISPLAY_DIR "/dsi_cmd"

#define CMD_ULP_ON "qcom,mdss-dsi-ultra-low-power-aod-on-command"
#define CMD_ULP_OFF "qcom,mdss-dsi-ultra-low-power-aod-off-command"

#define POLL_MS 300
#define AOD_SETTLE_MS 10000
#define POST_ON_VERIFY_MS 1000
#define DEFAULT_MIN_FPS 120

static volatile sig_atomic_t g_stop = 0;

static void on_signal(int sig) {
    (void)sig;
    g_stop = 1;
}

/* Reads a one-line sysfs node and trims surrounding whitespace. */
static int read_trim(const char* path, char* buf, size_t size) {
    if (size == 0) return 0;

    int fd = open(path, O_RDONLY | O_CLOEXEC);
    if (fd < 0) return 0;

    ssize_t n = read(fd, buf, size - 1);
    close(fd);
    if (n <= 0) return 0;

    buf[n] = '\0';

    size_t begin = 0;
    while (buf[begin] == ' ' || buf[begin] == '\t' || buf[begin] == '\r' || buf[begin] == '\n') {
        begin++;
    }

    size_t end = strlen(buf);
    while (end > begin) {
        char c = buf[end - 1];
        if (c != ' ' && c != '\t' && c != '\r' && c != '\n') break;
        end--;
    }

    size_t len = end - begin;
    if (begin > 0) memmove(buf, buf + begin, len);
    buf[len] = '\0';
    return 1;
}

static int write_value(const char* path, const char* value) {
    int fd = open(path, O_WRONLY | O_CLOEXEC);
    if (fd < 0) {
        __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, "open %s: %s", path, strerror(errno));
        return 0;
    }

    size_t len = strlen(value);
    ssize_t n = write(fd, value, len);
    close(fd);

    if (n != (ssize_t)len) {
        __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, "write %s: %s", path, strerror(errno));
        return 0;
    }
    return 1;
}

/* sysfs stores usually tolerate a missing newline, but the shell paths always
 * sent one, so keep it for parity with the tested behaviour. */
static int write_line(const char* path, const char* value) {
    char buf[128];
    int written = snprintf(buf, sizeof(buf), "%s\n", value);
    if (written <= 0 || (size_t)written >= sizeof(buf)) return 0;
    return write_value(path, buf);
}

static void sleep_ms(int ms) {
    usleep((useconds_t)ms * 1000);
}

static int in_aod(void) {
    char status[256];
    if (!read_trim(PATH_POWER_STATUS, status, sizeof(status))) return 0;
    return strstr(status, "kernel power :3") != NULL || strstr(status, "kernel power :1") != NULL;
}

/* The Settings switch writes 0x109f when enabled and 0x0 when disabled.
 * A missing node means unsupported hardware: stay completely idle. */
static int ltpo_enabled(void) {
    char value[64];
    if (!read_trim(PATH_ADFR_CONFIG, value, sizeof(value))) return 0;
    if (value[0] == '\0') return 0;
    if (strcmp(value, "0x0") == 0 || strcmp(value, "0X0") == 0 || strcmp(value, "0") == 0) {
        return 0;
    }
    return 1;
}

static void send_panel_command(const char* command) {
    if (!write_line(PATH_DSI_CMD, command)) return;
    write_line(PATH_DSI_CMD, "send");
}

/* Waits for the AOD entry sequence to settle. Returns 0 if AOD was left or the
 * daemon is shutting down. */
static int wait_aod_settle(void) {
    for (int waited = 0; waited < AOD_SETTLE_MS; waited += POLL_MS) {
        sleep_ms(POLL_MS);
        if (g_stop) return 0;
        if (!in_aod()) return 0;
    }
    return 1;
}

int main(void) {
    signal(SIGTERM, on_signal);
    signal(SIGINT, on_signal);

    int ulp_on = 0;
    int was_enabled = 0;

    /* A previous boot or a crashed instance may have left the DDIC in ULP. */
    send_panel_command(CMD_ULP_OFF);
    __android_log_print(ANDROID_LOG_INFO, LOG_TAG, "started, cleared stale ULP state");

    while (!g_stop) {
        int aod = in_aod();
        int enabled = ltpo_enabled();

        if (!enabled) {
            if (was_enabled) {
                __android_log_print(ANDROID_LOG_INFO, LOG_TAG, "ltpo off");
            }
            if (ulp_on) {
                send_panel_command(CMD_ULP_OFF);
                ulp_on = 0;
                __android_log_print(ANDROID_LOG_INFO, LOG_TAG, "ltpo off -> ULP off");
            }
            if (was_enabled && !aod) {
                char min_fps[32];
                if (read_trim(PATH_MIN_FPS, min_fps, sizeof(min_fps)) && min_fps[0] != '\0' &&
                    strcmp(min_fps, "120") != 0) {
                    char target[8];
                    snprintf(target, sizeof(target), "%d", DEFAULT_MIN_FPS);
                    if (write_line(PATH_MIN_FPS, target)) {
                        __android_log_print(ANDROID_LOG_INFO, LOG_TAG,
                                            "ltpo off -> min_fps %s -> %d", min_fps,
                                            DEFAULT_MIN_FPS);
                    }
                }
            }
            was_enabled = 0;
            sleep_ms(POLL_MS);
            continue;
        }

        if (!was_enabled) {
            __android_log_print(ANDROID_LOG_INFO, LOG_TAG, "ltpo on");
            was_enabled = 1;
        }

        if (aod) {
            if (!ulp_on && wait_aod_settle()) {
                send_panel_command(CMD_ULP_ON);
                ulp_on = 1;
                __android_log_print(ANDROID_LOG_INFO, LOG_TAG, "aod enter -> ULP on (1Hz)");

                sleep_ms(POST_ON_VERIFY_MS);
                if (!in_aod()) {
                    send_panel_command(CMD_ULP_OFF);
                    ulp_on = 0;
                    __android_log_print(ANDROID_LOG_INFO, LOG_TAG,
                                        "woke right after ULP on -> ULP off (recovered)");
                }
            }
        } else {
            if (ulp_on) {
                send_panel_command(CMD_ULP_OFF);
                ulp_on = 0;
                __android_log_print(ANDROID_LOG_INFO, LOG_TAG, "aod exit -> ULP off");
            }

            char min_fps[32];
            if (read_trim(PATH_MIN_FPS, min_fps, sizeof(min_fps)) && min_fps[0] != '\0' &&
                strcmp(min_fps, "1") != 0) {
                if (write_line(PATH_MIN_FPS, "1")) {
                    __android_log_print(ANDROID_LOG_INFO, LOG_TAG,
                                        "screen-on -> min_fps %s -> 1", min_fps);
                }
            }
        }

        sleep_ms(POLL_MS);
    }

    if (ulp_on) {
        send_panel_command(CMD_ULP_OFF);
    }
    __android_log_print(ANDROID_LOG_INFO, LOG_TAG, "stopped");
    return 0;
}
