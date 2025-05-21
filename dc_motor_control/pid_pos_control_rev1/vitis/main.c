#include <stdio.h>
#include <stdbool.h>
#include "xparameters.h"
#include "xil_io.h"

#define BASEADDR        XPAR_MAXON_TOP_0_BASEADDR
#define REG_KPKI        0x00
#define REG_KD          0x04
#define REG_ACTUAL      0x08
#define REG_DESIRED     0x0C

// Clear input buffer
void flush_stdin() {
    int c;
    while ((c = getchar()) != '\n' && c != EOF);
}

// Q7.8 fixed-point conversion
int float_to_q78(float val) {
    if (val < 0.0f) val = 0.0f;
    if (val > 127.996f) val = 127.996f;
    int qval = (int)(val * 256.0f);
    qval &= 0x7FFF;
    return qval;
}


float q78_to_float(int qval) {
    qval &= 0x7FFF;
    return qval / 256.0f;
}

int main() {
    int mode;
    float kp_f, ki_f, kd_f;
    int desired_pos;
    u32 kpki_val, kd_val;

    while (1) {
        printf("\n======= AXI4-Lite Q7.8 PID Control Interface =======\n");
        printf("1. WRITE (Enter Kp, Ki, Kd, Desired Position)\n");
        printf("2. READ  (Display all values and error)\n");
        printf("3. RESET (Reset all registers to zero)\n");
        printf("4. EXIT  (Exit the program)\n");
        printf("Select a mode (1-4): ");

        bool valid = false;
        while (!valid) {
            if (scanf("%d", &mode) == 1 && mode >= 1 && mode <= 4) {
                valid = true;
            } else {
                printf("[X] Invalid input. Please enter a number between 1 and 4.\n");
                flush_stdin();
            }
        }

        if (mode == 1) {
            // Kp
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

            // Ki
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

            // Kd
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

            // Desired Position
            valid = false;
            while (!valid) {
                printf("Enter Desired Position (integer): ");
                if (scanf("%d", &desired_pos) == 1) {
                    valid = true;
                } else {
                    printf("[X] Invalid number. Please enter an integer.\n");
                    flush_stdin();
                }
            }

            // Convert to Q7.8 and write to registers
            kpki_val = (float_to_q78(ki_f) << 16) | float_to_q78(kp_f);
            kd_val   = float_to_q78(kd_f);

            Xil_Out32(BASEADDR + REG_KPKI, kpki_val);
            Xil_Out32(BASEADDR + REG_KD, kd_val);
            Xil_Out32(BASEADDR + REG_DESIRED, desired_pos);

            printf("[OK] Values written: Kp=%.3f, Ki=%.3f, Kd=%.3f, Desired=%d\n",
                   kp_f, ki_f, kd_f, desired_pos);

        } else if (mode == 2) {
            // Read all registers
            u32 val_kpki = Xil_In32(BASEADDR + REG_KPKI);
            u32 val_kd = Xil_In32(BASEADDR + REG_KD);
            u32 val_actual = Xil_In32(BASEADDR + REG_ACTUAL);
            u32 val_desired = Xil_In32(BASEADDR + REG_DESIRED);

            int kp_q78 = val_kpki & 0x7FFF;
            int ki_q78 = (val_kpki >> 16) & 0x7FFF;
            int kd_q78 = val_kd & 0x7FFF;

            float kp = q78_to_float(kp_q78);
            float ki = q78_to_float(ki_q78);
            float kd = q78_to_float(kd_q78);

            int actual_pos = (int)val_actual;
            int desired_pos = (int)val_desired;
            int error = desired_pos - actual_pos;

            printf("\n--- Current Status ---\n");
            printf("Kp = %.3f, Ki = %.3f, Kd = %.3f\n", kp, ki, kd);
            printf("Desired Position = %d\n", desired_pos);
            printf("Actual  Position = %d\n", actual_pos);
            printf("Error = Desired - Actual = %d\n", error);

        } else if (mode == 3) {
            // Read all registers
            u32 val_kpki = Xil_In32(BASEADDR + REG_KPKI);
            u32 val_kd = Xil_In32(BASEADDR + REG_KD);
            u32 val_actual = Xil_In32(BASEADDR + REG_ACTUAL);
            u32 val_desired = Xil_In32(BASEADDR + REG_DESIRED);

            int kp_q78 = val_kpki & 0x7FFF;
            int ki_q78 = (val_kpki >> 16) & 0x7FFF;
            int kd_q78 = val_kd & 0x7FFF;

            float kp = q78_to_float(kp_q78);
            float ki = q78_to_float(ki_q78);
            float kd = q78_to_float(kd_q78);

            int actual_pos = (int)val_actual;
            int desired_pos = (int)val_desired;
            int error = desired_pos - actual_pos;

            printf("\n--- Current Status ---\n");
            printf("Kp = %.3f, Ki = %.3f, Kd = %.3f\n", kp, ki, kd);
            printf("Desired Position = %d\n", desired_pos);
            printf("Actual  Position = %d\n", actual_pos);
            printf("Error = Desired - Actual = %d\n", error);


        } else if (mode == 4) {
            Xil_Out32(BASEADDR + REG_KPKI, 0);
            Xil_Out32(BASEADDR + REG_KD, 0);
            Xil_Out32(BASEADDR + REG_DESIRED, 0);
            printf("[OK] All values reset to zero.\n");
        }
    }

    return 0;
}
