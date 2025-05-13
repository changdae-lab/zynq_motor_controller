#include <stdio.h>
#include "xparameters.h"
#include "xil_io.h"

// 명령어 정의
#define WRITE 1  // 쓰기 명령
#define READ 2   // 읽기 명령
#define EXIT 3   // 종료 명령
#define RESET 4  // 초기화 명령
#define AXI_DATA_BYTE 4  // AXI 데이터버스의 데이터 크기 (바이트 단위)

// 인터페이스 주소 정의
#define BASEADDR XPAR_MAXON_TOP_0_BASEADDR  // AXI 인터페이스가 매핑된 주소

int main() {
    int mode, reg_num, data, kp, ki;  // 사용자 입력 변수

    while (1) {
        // 메뉴 출력: 사용자가 선택할 수 있는 옵션을 화면에 표시
        printf("======= AXI4-Lite Motor Control Interface =======\n");
        printf("Select operation mode:\n");
        printf("1. WRITE (쓰기)\n");
        printf("2. READ (읽기)\n");
        printf("3. EXIT (종료)\n");
        printf("4. RESET (초기화)\n");  // 초기화 명령 추가
        printf("Enter mode: ");
        scanf("%d", &mode);  // 사용자가 선택한 모드를 입력받음

        if (mode == EXIT) {
            printf("Exiting program.\n");
            break;  // 종료 명령을 선택하면 프로그램 종료
        }

        if (mode == WRITE) {  // WRITE 모드를 선택한 경우
            printf("Enter register number to WRITE (0 ~ 3): ");
            scanf("%d", &reg_num);  // 어떤 레지스터에 데이터를 쓸지 입력받음
            if (reg_num < 0 || reg_num > 3) {
                printf("Invalid register number. Please try again.\n");
                continue;  // 잘못된 레지스터 번호를 입력하면 다시 입력받음
            }

            if (reg_num == 0) {  // 0번 레지스터에 Kp와 Ki 값을 쓰는 경우
                // Kp와 Ki 값 입력 받기
                printf("Enter Kp (0 ~ 65535): ");
                scanf("%d", &kp);  // Kp 값 입력받음
                if (kp < 0 || kp > 65535) {
                    printf("Invalid Kp value. Please try again.\n");
                    continue;  // 잘못된 Kp 값 입력 시 다시 입력받음
                }

                printf("Enter Ki (0 ~ 65535): ");
                scanf("%d", &ki);  // Ki 값 입력받음
                if (ki < 0 || ki > 65535) {
                    printf("Invalid Ki value. Please try again.\n");
                    continue;  // 잘못된 Ki 값 입력 시 다시 입력받음
                }

                // Kp와 Ki 값을 하나의 데이터로 합쳐서 레지스터에 저장 (Ki는 상위 16비트, Kp는 하위 16비트)
                data = (ki << 16) | (kp & 0xFFFF);  // Ki는 상위 16비트로 이동, Kp는 하위 16비트 마스킹
                Xil_Out32(BASEADDR + (reg_num * AXI_DATA_BYTE), (u32)data);  // 레지스터에 데이터 쓰기
                printf("WRITE complete. Register 0 (Kp=%d, Ki=%d), Combined Value: 0x%X\n", kp, ki, data);
            } else {  // 0번 레지스터가 아닌 다른 레지스터에 데이터를 쓰는 경우
                printf("Enter value to WRITE: ");
                scanf("%d", &data);  // 쓸 데이터 입력받음
                Xil_Out32(BASEADDR + (reg_num * AXI_DATA_BYTE), (u32)data);  // 레지스터에 데이터 쓰기
                printf("WRITE complete. Register %d, Value: %d\n", reg_num, data);
            }

        } else if (mode == READ) {  // READ 모드를 선택한 경우
            printf("Enter register number to READ (0 ~ 3): ");
            scanf("%d", &reg_num);  // 읽을 레지스터 번호 입력받음
            if (reg_num < 0 || reg_num > 3) {
                printf("Invalid register number. Please try again.\n");
                continue;  // 잘못된 레지스터 번호 입력 시 다시 입력받음
            }

            data = Xil_In32(BASEADDR + (reg_num * AXI_DATA_BYTE));  // 해당 레지스터에서 데이터 읽기
            if (reg_num == 0) {  // 0번 레지스터의 경우, Kp와 Ki 값을 분리해서 출력
                // Kp와 Ki 값 분리
                kp = data & 0xFFFF;  // 하위 16비트는 Kp
                ki = (data >> 16) & 0xFFFF;  // 상위 16비트는 Ki
                printf("READ complete. Register 0: Kp=%d, Ki=%d (Combined Value: 0x%X)\n", kp, ki, data);
            } else {
                printf("READ complete. Register %d, Value: %d\n", reg_num, data);  // 다른 레지스터는 값만 출력
            }

        } else if (mode == RESET) {  // RESET 모드를 선택한 경우 (초기화)
            // 모든 레지스터를 0으로 초기화
            for (reg_num = 0; reg_num < 4; reg_num++) {  // 0번부터 3번까지 모든 레지스터 초기화
                Xil_Out32(BASEADDR + (reg_num * AXI_DATA_BYTE), 0);  // 각 레지스터에 0 쓰기
            }
            printf("RESET complete. All registers set to 0.\n");  // 초기화 완료 메시지 출력

        } else {
            printf("Invalid mode selected. Please enter 1, 2, 3, or 4.\n");  // 잘못된 모드 선택 시 메시지 출력
        }
    }

    return 0;  // 프로그램 종료
}
