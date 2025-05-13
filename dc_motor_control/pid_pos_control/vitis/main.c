#include <stdio.h>
#include "xparameters.h"
#include "xil_io.h"

// 명령어 정의
#define WRITE 1  // 레지스터에 값 쓰기
#define READ 2   // 레지스터에서 값 읽기
#define EXIT 3   // 프로그램 종료
#define RESET 4  // 모든 레지스터 초기화
#define AXI_DATA_BYTE 4  // AXI 데이터 크기 (바이트 단위)

// AXI 인터페이스 주소
#define BASEADDR XPAR_MAXON_TOP_0_BASEADDR  // AXI 인터페이스의 시작 주소

int main() {
    int mode, reg_num, data, kp, ki;  // 사용자 입력 변수

    while (1) {
        // 메뉴 출력
        printf("\n======= AXI4-Lite Motor Control Interface =======\n");
        printf("1. WRITE (레지스터에 값 쓰기)\n");
        printf("2. READ (레지스터에서 값 읽기)\n");
        printf("3. EXIT (프로그램 종료)\n");
        printf("4. RESET (모든 레지스터 초기화)\n");
        printf("모드를 선택하세요: ");
        scanf("%d", &mode);

        if (mode == EXIT) {
            printf("프로그램을 종료합니다.\n");
            break;
        }

        if (mode == WRITE) {
            printf("값을 쓸 레지스터 번호를 입력하세요 (0 ~ 3): ");
            scanf("%d", &reg_num);

            if (reg_num < 0 || reg_num > 3) {
                printf("잘못된 레지스터 번호입니다. 다시 시도하세요.\n");
                continue;
            }

            if (reg_num == 0) {
                // Kp와 Ki 값 입력받기
                printf("Kp 값을 입력하세요 (0 ~ 65535): ");
                scanf("%d", &kp);
                if (kp < 0 || kp > 65535) {
                    printf("잘못된 Kp 값입니다. 다시 시도하세요.\n");
                    continue;
                }

                printf("Ki 값을 입력하세요 (0 ~ 65535): ");
                scanf("%d", &ki);
                if (ki < 0 || ki > 65535) {
                    printf("잘못된 Ki 값입니다. 다시 시도하세요.\n");
                    continue;
                }

                // Kp와 Ki를 하나의 데이터로 합쳐서 저장
                data = (ki << 16) | (kp & 0xFFFF);
                Xil_Out32(BASEADDR + (reg_num * AXI_DATA_BYTE), (u32)data);
                printf("레지스터 0에 쓰기 완료 (Kp=%d, Ki=%d, 합쳐진 값=0x%X)\n", kp, ki, data);
            } else {
                // 다른 레지스터에 값 쓰기
                printf("쓸 값을 입력하세요: ");
                scanf("%d", &data);
                Xil_Out32(BASEADDR + (reg_num * AXI_DATA_BYTE), (u32)data);
                printf("레지스터 %d에 쓰기 완료 (값=%d)\n", reg_num, data);
            }

        } else if (mode == READ) {
            printf("값을 읽을 레지스터 번호를 입력하세요 (0 ~ 3): ");
            scanf("%d", &reg_num);

            if (reg_num < 0 || reg_num > 3) {
                printf("잘못된 레지스터 번호입니다. 다시 시도하세요.\n");
                continue;
            }

            data = Xil_In32(BASEADDR + (reg_num * AXI_DATA_BYTE));
            if (reg_num == 0) {
                // Kp와 Ki 분리해서 출력
                kp = data & 0xFFFF;
                ki = (data >> 16) & 0xFFFF;
                printf("레지스터 0 읽기 완료 (Kp=%d, Ki=%d, 합쳐진 값=0x%X)\n", kp, ki, data);
            } else {
                printf("레지스터 %d 읽기 완료 (값=%d)\n", reg_num, data);
            }

        } else if (mode == RESET) {
            // 모든 레지스터 초기화
            printf("모든 레지스터를 0으로 초기화합니다...\n");
            for (reg_num = 0; reg_num < 4; reg_num++) {
                Xil_Out32(BASEADDR + (reg_num * AXI_DATA_BYTE), 0);
            }
            printf("초기화 완료. 모든 레지스터가 0으로 설정되었습니다.\n");

        } else {
            printf("잘못된 모드입니다. 1, 2, 3, 4 중에서 선택하세요.\n");
        }
    }

    return 0;
}
