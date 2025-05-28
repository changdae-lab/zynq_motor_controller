// quintic_pid_logger_persistent.c: SD 로깅 파일명 접두사 제거 및 8.3 파일명 사용
// 트라젝틱 주파수 조정 가능 (기본 5 kHz)

#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "xparameters.h"
#include "xil_io.h"
#include "ff.h"
#include "xtime_l.h"

#define BASEADDR1      XPAR_MAXON_TOP_0_BASEADDR
#define BASEADDR2      XPAR_MAXON_TOP_1_BASEADDR
#define REG_KPKI       0x00
#define REG_KD         0x04
#define REG_ACTUAL     0x08
#define REG_DESIRED    0x0C
#define COUNTS_PER_MS  (XPAR_CPU_CORTEXA9_0_CPU_CLK_FREQ_HZ / 1000)

// 트라젝틱 주파수 (Hz)
#define CMD_FREQ_HZ    5000
#define COUNTS_PER_CMD (XPAR_CPU_CORTEXA9_0_CPU_CLK_FREQ_HZ / CMD_FREQ_HZ)

FATFS fs;
FIL fil;
bool log_enabled = true;
bool log_header_written = false;
char log_filename[12];  // "LOGxx.CSV"
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

int quintic_trajectory(u32 t_ms, u32 T_ms, int q0, int qf) {
    float tau = (float)t_ms / (float)T_ms;
    if (tau < 0.0f) tau = 0.0f;
    if (tau > 1.0f) tau = 1.0f;
    float q = 6*tau*tau*tau*tau*tau
            - 15*tau*tau*tau*tau
            + 10*tau*tau*tau;
    return q0 + (int)((qf - q0) * q);
}

int main() {
    int mode;
    float kp_f = 0.0f, ki_f = 0.0f, kd_f = 0.0f;
    int target_pos1 = 0, target_pos2 = 0;
    FRESULT res;

    // SD 마운트 (프로그램 시작 시 한 번)
    res = f_mount(&fs, "0:", 1);
    if (res != FR_OK) {
        printf("[ERR] SD mount failed: %d\n", res);
        return -1;
    }
    printf("[INFO] SD mounted.\n");

    // 초기 로그 파일 생성
    snprintf(log_filename, sizeof(log_filename), "LOG%02d.CSV", log_file_counter++);
    res = f_open(&fil, log_filename, FA_CREATE_ALWAYS | FA_WRITE);
    if (res == FR_OK) {
        printf("[INFO] Logging started: %s\n", log_filename);
    } else {
        printf("[ERR] initial f_open failed: %d\n", res);
    }

    XTime_GetTime(&t_start);
    t_prev = t_start;

    while (1) {
        printf("\n======= 2-Axis Control Menu =======\n");
        printf("1. Set PID Gains\n");
        printf("2. Quintic Trajectory Planning (Forward & Return)\n");
        printf("3. Read All Status\n");
        printf("4. Reset All\n");
        printf("5. Toggle SD Logging (currently: %s)\n", log_enabled ? "ON" : "OFF");

        bool valid = false;
        while (!valid) {
            printf("Select mode (1-5): ");
            if (scanf("%d", &mode) == 1 && mode >= 1 && mode <= 5) valid = true;
            else { printf("[X] Invalid input.\n"); flush_stdin(); }
        }

        if (mode == 1) {
            // 1. Set PID Gains
            valid = false;
            while (!valid) {
                printf("Enter Kp (0.0 ~ 127.996): ");
                if (scanf("%f", &kp_f) == 1 && kp_f >= 0.0f && kp_f <= 127.996f) valid = true;
                else { printf("[X] Invalid Kp.\n"); flush_stdin(); }
            }
            valid = false;
            while (!valid) {
                printf("Enter Ki (0.0 ~ 127.996): ");
                if (scanf("%f", &ki_f) == 1 && ki_f >= 0.0f && ki_f <= 127.996f) valid = true;
                else { printf("[X] Invalid Ki.\n"); flush_stdin(); }
            }
            valid = false;
            while (!valid) {
                printf("Enter Kd (0.0 ~ 127.996): ");
                if (scanf("%f", &kd_f) == 1 && kd_f >= 0.0f && kd_f <= 127.996f) valid = true;
                else { printf("[X] Invalid Kd.\n"); flush_stdin(); }
            }
            u32 kpki_val = (float_to_q78(ki_f) << 16) | float_to_q78(kp_f);
            u32 kd_val   = float_to_q78(kd_f);
            Xil_Out32(BASEADDR1 + REG_KPKI, kpki_val);
            Xil_Out32(BASEADDR1 + REG_KD,   kd_val);
            Xil_Out32(BASEADDR2 + REG_KPKI, kpki_val);
            Xil_Out32(BASEADDR2 + REG_KD,   kd_val);
            printf("[OK] PID Gains written.\n");
        }
        else if (mode == 2) {
            // 2. Quintic Trajectory 실행
            printf("Enter target pos Axis1: "); scanf("%d", &target_pos1);
            printf("Enter target pos Axis2: "); scanf("%d", &target_pos2);

            int q0_1 = Xil_In32(BASEADDR1 + REG_ACTUAL);
            int q0_2 = Xil_In32(BASEADDR2 + REG_ACTUAL);

            XTime t_phase, t_cmd;
            XTime_GetTime(&t_phase);
            t_cmd = t_phase;

            u32 phase_ms = 1000;
            u32 total_ms = 2 * phase_ms;
            XTime interval = (XTime)COUNTS_PER_CMD;

            char buf[128];
            UINT bw;

            while (1) {
                XTime now;
                XTime_GetTime(&now);
                u32 elapsed = (u32)((now - t_phase) / COUNTS_PER_MS);
                if (elapsed > total_ms) break;

                if ((now - t_cmd) >= interval) {
                    int des1 = (elapsed <= phase_ms)
                             ? quintic_trajectory(elapsed, phase_ms, q0_1, target_pos1)
                             : quintic_trajectory(elapsed - phase_ms, phase_ms, target_pos1, q0_1);
                    int des2 = (elapsed <= phase_ms)
                             ? quintic_trajectory(elapsed, phase_ms, q0_2, target_pos2)
                             : quintic_trajectory(elapsed - phase_ms, phase_ms, target_pos2, q0_2);

					Xil_Out32(BASEADDR1 + REG_DESIRED, des1);
					Xil_Out32(BASEADDR2 + REG_DESIRED, des2);

                    int act1 = Xil_In32(BASEADDR1 + REG_ACTUAL);
                    int act2 = Xil_In32(BASEADDR2 + REG_ACTUAL);

                    u32 sys = (u32)((now - t_start) / COUNTS_PER_MS);
                    u32 del = (u32)((now - t_prev) / COUNTS_PER_MS);
                    t_prev = now;

                    if (log_enabled) {
                        if (!log_header_written) {
                            strcpy(buf, "Time_ms,Delta_ms,Kp,Ki,Kd,Des1,Act1,Err1,Des2,Act2,Err2\n");
                            f_write(&fil, buf, strlen(buf), &bw);
                            log_header_written = true;
                        }
                        int len = snprintf(buf, sizeof(buf), "%lu,%lu,%.3f,%.3f,%.3f,%d,%d,%d,%d,%d,%d\n",
                                           sys, del, kp_f, ki_f, kd_f,
                                           des1, act1, des1-act1,
                                           des2, act2, des2-act2);
                        f_write(&fil, buf, len, &bw);
                    }

                    t_cmd = now;
                }
            }
            f_sync(&fil);
            printf("[OK] Trajectory done.\n");
        }
        else if (mode == 3) {
            // Read All Status
            u32 val_kpki1 = Xil_In32(BASEADDR1 + REG_KPKI);
            u32 val_kd1   = Xil_In32(BASEADDR1 + REG_KD);
            u32 val_des1  = Xil_In32(BASEADDR1 + REG_DESIRED);
            u32 val_act1  = Xil_In32(BASEADDR1 + REG_ACTUAL);

            u32 val_kpki2 = Xil_In32(BASEADDR2 + REG_KPKI);
            u32 val_kd2   = Xil_In32(BASEADDR2 + REG_KD);
            u32 val_des2  = Xil_In32(BASEADDR2 + REG_DESIRED);
            u32 val_act2  = Xil_In32(BASEADDR2 + REG_ACTUAL);

            printf("--- Axis 1 ---\n");
            printf("Kp=%.3f, Ki=%.3f, Kd=%.3f\n", q78_to_float(val_kpki1 & 0x7FFF), q78_to_float((val_kpki1 >> 16) & 0x7FFF), q78_to_float(val_kd1 & 0x7FFF));
            printf("Desired=%d, Actual=%d\n", (int)val_des1, (int)val_act1);

            printf("--- Axis 2 ---\n");
            printf("Kp=%.3f, Ki=%.3f, Kd=%.3f\n", q78_to_float(val_kpki2 & 0x7FFF), q78_to_float((val_kpki2 >> 16) & 0x7FFF), q78_to_float(val_kd2 & 0x7FFF));
            printf("Desired=%d, Actual=%d\n", (int)val_des2, (int)val_act2);
        }
        else if (mode == 4) {
            // Reset All: 로그 헤더 플래그 초기화
            log_header_written = false;
            Xil_Out32(BASEADDR1 + REG_KPKI, 0);
            Xil_Out32(BASEADDR1 + REG_KD,   0);
            Xil_Out32(BASEADDR1 + REG_DESIRED, 0);
            Xil_Out32(BASEADDR2 + REG_KPKI, 0);
            Xil_Out32(BASEADDR2 + REG_KD,   0);
            Xil_Out32(BASEADDR2 + REG_DESIRED, 0);
            printf("[OK] All values reset.\n");
        }
        else if (mode == 5) {
            // 5. Toggle Logging
            log_enabled = !log_enabled;
            if (log_enabled) {
                snprintf(log_filename, sizeof(log_filename), "LOG%02d.CSV", log_file_counter++);
                res = f_open(&fil, log_filename, FA_CREATE_ALWAYS | FA_WRITE);
                if (res == FR_OK) {
                    log_header_written = false;
                    printf("[LOG] ENABLED: %s\n", log_filename);
                } else {
                    printf("[ERR] f_open toggle: %d\n", res);
                    log_enabled = false;
                }
            } else {
                f_close(&fil);
                printf("[LOG] DISABLED.\n");
            }
        }
    }
    return 0;
}
