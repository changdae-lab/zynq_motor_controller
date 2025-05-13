module motor_top (
    input wire clk,              // 150 MHz 시스템 클럭
    input wire reset_n,                 // 리셋 신호 (Active Low)
    input wire encoder_a,               // 엔코더 A 채널
    input wire encoder_b,               // 엔코더 B 채널
    input wire encoder_index,           // 엔코더 Index 신호
    input wire [15:0] Kp_axi,           // PID Kp 상수
    input wire [15:0] Ki_axi,           // PID Ki 상수
    input wire [15:0] Kd_axi,           // PID Kd 상수
    input wire signed [31:0] desired_pos, // 목표 위치

    output wire dir1,                   // 방향 제어 1
    output wire dir2,                   // 방향 제어 2
    output wire pwm_out,                // PWM 출력
    output wire signed [31:0] actual_pos, // 실제 위치 출력
    output wire error_flag,              // 에러 플래그 출력
    output wire signed [15:0] pid_control_signal          // PID 제어 신호 출력
);

    // 내부 신호 정의
    wire signed [31:0] encoder_position;      // 엔코더에서 계산된 위치 값
    wire signed [15:0] pid_control_signal;    // PID 제어 신호

    clk_di

    // 쿼드러쳐 엔코더 모듈 인스턴스화
    (* dont_touch = "true" *)
    quadrature_encoder u_quadrature_encoder (
        .clk(clk),             // 150 MHz 클럭
        .reset_n(reset_n),                   // 리셋 신호
        .A(encoder_a),                       // 엔코더 A 신호
        .B(encoder_b),                       // 엔코더 B 신호
        .Index(encoder_index),               // 엔코더 Index 신호
        .actual_position(encoder_position),  // 위치 출력
        .error_flag(error_flag)              // 에러 플래그 출력
    );

    // PID 컨트롤러 모듈 인스턴스화
    (* dont_touch = "true" *)
    pid_controller u_pid_controller (
        .clk(clk),                    // 150 MHz 클럭
        .reset_n(reset_n),                   // 리셋 신호
        .desired_pos(desired_pos),           // 목표 위치
        .actual_pos(encoder_position),       // 실제 위치
        .Kp_axi(Kp_axi),                     // Kp 값
        .Ki_axi(Ki_axi),                     // Ki 값
        .Kd_axi(Kd_axi),                     // Kd 값
        .control_signal(pid_control_signal)  // PID 제어 신호 출력
    );

    // PWM 생성기 모듈 인스턴스화
    (* dont_touch = "true" *)
    pwm_generator_bidirectional u_pwm_generator (
        .clk(clk),                    // 150 MHz 클럭
        .reset_n(reset_n),                   // 리셋 신호
        .pid_control_signal(pid_control_signal), // PID 제어 신호 입력
        .dir1(dir1),                         // 방향 제어 1
        .dir2(dir2),                          // 방향 제어 2
        .pwm_out(pwm_out)                    // PWM 출력
    );

    // 실제 위치 출력 연결
    assign actual_pos = encoder_position;

endmodule
