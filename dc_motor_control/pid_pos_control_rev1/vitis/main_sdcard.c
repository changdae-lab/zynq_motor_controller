#include <stdio.h>
#include <stdbool.h>
#include "xparameters.h"
#include "xil_io.h"
#include "ff.h"
#include "xtime_l.h"

#define BASEADDR        XPAR_MAXON_TOP_0_BASEADDR
#define REG_KPKI        0x00
#define REG_KD          0x04
#define REG_ACTUAL      0x08
#define REG_DESIRED     0x0C
#define COUNTS_PER_MS (XPAR_CPU_CORTEXA9_0_CPU_CLK_FREQ_HZ / 1000)

FATFS fs;
FIL fil;
bool log_header_written = false;
char log_filename[32];
int log_file_counter = 1;
XTime t_start, t_prev;

void flush_stdin() {
    int c;
    while ((c = getchar()) != '\n' && c != EOF);
}

int float_to_q78(float val) {
    if (val < 0.0f) val = 0.0f;
    if (val > 127.996f) val = 127.996f;
    return ((int)(val * 256.0f)) & 0x7FFF;
}

float q78_to_float(int qval) {
    return (qval & 0x7FFF) / 256.0f;
}

void log_to_sd(float kp, float ki, float kd, int desired, int actual, u32 t_ms, u32 delta_ms) {
    int error = desired - actual;

    if (f_mount(&fs, "0:/", 1) != FR_OK) return;
    if (f_open(&fil, log_filename, FA_OPEN_APPEND | FA_WRITE) != FR_OK) return;

    if (!log_header_written) {
        f_printf(&fil, "Time_ms,Delta_ms,Kp,Ki,Kd,Desired,Actual,Error\n");
        log_header_written = true;
    }

    f_printf(&fil, "%lu,%lu,%.3f,%.3f,%.3f,%d,%d,%d\n", t_ms, delta_ms, kp, ki, kd, desired, actual, error);

    f_close(&fil);
    f_mount(NULL, "0:/", 1);
}

int main() {
    int mode;
    float kp_f = 0, ki_f = 0, kd_f = 0;
    int desired_pos = 0;

    XTime_GetTime(&t_start);
    t_prev = t_start;

    while (1) {
        printf("\n======= PID Menu (1234 CLI) =======\n");
        printf("1. Set PID Gains\n");
        printf("2. Set Desired Position (Log for 5s)\n");
        printf("3. Read All Status\n");
        printf("4. Reset All\n");

        bool valid = false;
        while (!valid) {
            printf("Select mode (1-4): ");
            if (scanf("%d", &mode) == 1 && mode >= 1 && mode <= 4) {
                valid = true;
            } else {
                printf("[X] Invalid input. Try again.\n");
                flush_stdin();
            }
        }

        if (mode == 1) {
            valid = false;
            while (!valid) {
                printf("Enter Kp (0.0 ~ 127.996): ");
                if (scanf("%f", &kp_f) == 1 && kp_f >= 0.0f && kp_f <= 127.996f) {
                    valid = true;
                } else {
                    printf("[X] Invalid Kp. Try again.\n");
                    flush_stdin();
                }
            }

            valid = false;
            while (!valid) {
                printf("Enter Ki (0.0 ~ 127.996): ");
                if (scanf("%f", &ki_f) == 1 && ki_f >= 0.0f && ki_f <= 127.996f) {
                    valid = true;
                } else {
                    printf("[X] Invalid Ki. Try again.\n");
                    flush_stdin();
                }
            }

            valid = false;
            while (!valid) {
                printf("Enter Kd (0.0 ~ 127.996): ");
                if (scanf("%f", &kd_f) == 1 && kd_f >= 0.0f && kd_f <= 127.996f) {
                    valid = true;
                } else {
                    printf("[X] Invalid Kd. Try again.\n");
                    flush_stdin();
                }
            }

            u32 kpki_val = (float_to_q78(ki_f) << 16) | float_to_q78(kp_f);
            u32 kd_val = float_to_q78(kd_f);

            Xil_Out32(BASEADDR + REG_KPKI, kpki_val);
            Xil_Out32(BASEADDR + REG_KD,   kd_val);

            printf("[OK] PID Gains Written\n");
        }
        else if (mode == 2) {
            valid = false;
            while (!valid) {
                printf("Enter Desired Position (int): ");
                if (scanf("%d", &desired_pos) == 1) {
                    valid = true;
                } else {
                    printf("[X] Invalid input.\n");
                    flush_stdin();
                }
            }

            snprintf(log_filename, sizeof(log_filename), "logg_%02d.csv", log_file_counter++);
            log_header_written = false;

            printf("[LOG] Recording motor response for 5 seconds to %s...\n", log_filename);
            XTime start_time, current_time, end_time;
            XTime_GetTime(&start_time);
            XTime interval_5s = (XTime)COUNTS_PER_MS * 5000;
            end_time = start_time + interval_5s;

            // 로그 시작 전에 desired 값을 설정
            Xil_Out32(BASEADDR + REG_DESIRED, desired_pos);

            do {
                u32 val_actual = Xil_In32(BASEADDR + REG_ACTUAL);
                XTime_GetTime(&current_time);
                u32 t_ms = (current_time - t_start) / COUNTS_PER_MS;
                u32 delta_ms = (current_time - t_prev) / COUNTS_PER_MS;
                t_prev = current_time;
                log_to_sd(kp_f, ki_f, kd_f, desired_pos, (int)val_actual, t_ms, delta_ms);
            } while (current_time < end_time);

            printf("[OK] Logging completed.\n");
        }
        else if (mode == 3) {
            u32 val_kpki = Xil_In32(BASEADDR + REG_KPKI);
            u32 val_kd = Xil_In32(BASEADDR + REG_KD);
            u32 val_actual = Xil_In32(BASEADDR + REG_ACTUAL);
            u32 val_desired = Xil_In32(BASEADDR + REG_DESIRED);

            float kp = q78_to_float(val_kpki & 0x7FFF);
            float ki = q78_to_float((val_kpki >> 16) & 0x7FFF);
            float kd = q78_to_float(val_kd & 0x7FFF);

            printf("--- Current PID + Position ---\n");
            printf("Kp=%.3f, Ki=%.3f, Kd=%.3f\n", kp, ki, kd);
            printf("Desired=%d, Actual=%d\n", (int)val_desired, (int)val_actual);
        }
        else if (mode == 4) {
            kp_f = ki_f = kd_f = 0.0f;
            desired_pos = 0;
            Xil_Out32(BASEADDR + REG_KPKI, 0);
            Xil_Out32(BASEADDR + REG_KD, 0);
            Xil_Out32(BASEADDR + REG_DESIRED, 0);
            log_header_written = false;
            printf("[OK] All values reset\n");
        }
    }

    return 0;
}
