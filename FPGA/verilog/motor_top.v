module motor_top (
    input wire clk,                         // 100 MHz 시스템 클럭
    input wire reset_n,                     // 리셋 신호 (Active Low)
    input wire encoder_a,                   // 엔코더 A 채널
    input wire encoder_b,                   // 엔코더 B 채널
    input wire encoder_index,               // 엔코더 Index 신호
    input wire [15:0] Kp_axi,               // PI Kp 상수
    input wire [15:0] Ki_axi,               // PI Ki 상수
    input wire [15:0] Kd_axi,               // PI Kd 상수 (사용하지 않음)
    input wire signed [31:0] desired_pos,   // 목표 속도

    output wire dir1,                       // 방향 제어 1
    output wire dir2,                       // 방향 제어 2
    output wire pwm_out,                    // PWM 출력
    output wire signed [15:0] pid_control_signal, // PI 제어 신호 출력
    output wire signed [31:0] actual_position             // 실제 위치 출력
);

    assign actual_position = encoder_position; // 엔코더 위치를 실제 위치로 설정

    // 내부 신호 정의
        wire signed [31:0] encoder_position;


    // 쿼드러쳐 엔코더 모듈 인스턴스화
    (* dont_touch = "true" *)
    quadrature_encoder u_quadrature_encoder (
        .clk(clk),                           // 100 MHz 클럭
        .reset_n(reset_n),                   // 리셋 신호
        .A(encoder_a),                       // 엔코더 A 신호
        .B(encoder_b),                       // 엔코더 B 신호
        .Index(encoder_index),               // 엔코더 Index 신호
        .actual_position(encoder_position)  // 위치 출력
    );

    // PI velocity 컨트롤러 모듈 인스턴스화
    (* dont_touch = "true" *)
    pi_velocity_controller u_pi_velocity_controller (
        .clk(clk),                       // 20 kHz 클럭
        .reset_n(reset_n),                   // 리셋 신호
        .desired_pos(desired_pos),           // 목표 위치
        .actual_pos(encoder_position),       // 실제 위치
        .Kp_axi(Kp_axi),                   // Kp 값
        .Ki_axi(Ki_axi),                   // Ki 값
        .Kd_axi(Kd_axi),                   // Kd 값 (사용하지 않음)
        .control_signal(pid_control_signal)   // PID 제어 신호 출력
    );

    // PWM 생성기 모듈 인스턴스화
    (* dont_touch = "true" *)
    pwm_generator_bidirectional u_pwm_generator (
        .clk(clk),                    // 100 MHz 클럭
        .reset_n(reset_n),                   // 리셋 신호
        .pid_control_signal(pid_control_signal), // PID 제어 신호 입력
        .dir1(dir1),                         // 방향 제어 1
        .dir2(dir2),                          // 방향 제어 2
        .pwm_out(pwm_out)                    // PWM 출력
    );

endmodule
