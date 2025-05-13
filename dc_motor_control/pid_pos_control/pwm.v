`timescale 1ns / 1ps

module pwm_generator_bidirectional (
    input wire clk,                    // 100MHz 클럭 입력
    input wire reset_n,                // 비동기 리셋 (Active Low)
    input wire signed [15:0] pid_control_signal, // PID 제어 입력 (-4000 ~ 4000)
    output reg dir1,                   // 방향 제어 1
    output reg dir2,                   // 방향 제어 2
    output reg pwm_out                 // PWM 출력
);

    // 100MHz -> 10kHz 변환을 위한 파라미터
    parameter signed [15:0] MAX_COUNT = 4000;  // PWM 주기 (100MHz / 25kHz)

    reg signed [15:0] counter;         // 16비트 signed counter
    reg signed [15:0] duty_cycle;      // 16비트 signed duty cycle
    reg signed [15:0] prev_pid_control_signal; // 이전 PID 제어 입력

    // PID 입력을 기반으로 Duty Cycle 및 방향 설정
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            duty_cycle <= 16'sd0;
            dir1 <= 1'b0;
            dir2 <= 1'b0;
            prev_pid_control_signal <= 16'sd0;
        end else begin
            // PID 제어 입력 변경 시에만 업데이트
            if (pid_control_signal != prev_pid_control_signal) begin
                // PID 제어 입력 제한 및 방향 설정
                if (pid_control_signal > 0) begin
                    duty_cycle <= pid_control_signal; // 양수 PID 값 반영
                    dir1 <= 1'b1;
                    dir2 <= 1'b0;
                end else if (pid_control_signal < 0) begin
                    duty_cycle <= -pid_control_signal; // 음수 PID 값 반영
                    dir1 <= 1'b0;
                    dir2 <= 1'b1;
                end else begin
                    duty_cycle <= 16'sd0; // 0일 때
                    dir1 <= 1'b0;
                    dir2 <= 1'b0;
                end

                // PID 제어 신호 변경을 반영하여 저장
                prev_pid_control_signal <= pid_control_signal;
            end
        end
    end

    // PWM 생성
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            counter <= 16'sd0;
            pwm_out <= 1'b0;
        end else begin
            // 카운터 증가 및 초기화
            if (counter < MAX_COUNT - 1) begin
                counter <= counter + 1;
            end else begin
                counter <= 16'sd0; // PWM 주기 초기화
            end

            // PWM 신호 생성
            if (counter < duty_cycle) begin
                pwm_out <= 1'b1;
            end else begin
                pwm_out <= 1'b0;
            end
        end
    end

endmodule