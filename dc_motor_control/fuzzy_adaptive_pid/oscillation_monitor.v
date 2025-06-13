`timescale 1ns / 1ps

module oscillation_detection (
    input wire clk,                      // 원래 클럭 (100mhz)
    input wire reset_n,                  // 비동기 리셋 (Active Low)
    input wire signed [31:0] error_pos, // 목표 속도도
    input wire signed [31:0] delta_error,  // 실제 위치
    input wire clk_100k_enable,      // 100kHz 클럭 Enable 신호 
    output reg oscillating_flag          // 진동 감지 플래그
);

    // 내부 변수
    reg signed [31:0] prev_error;       // 이전 오차
    reg signed [31:0] error_pos;        // 현재 오차
    reg signed [31:0] delta_error;      // 오차 차이
    reg signed [31:0] prev_delta_error; // 이전 오차 차이

    // 진동 감지 변수
    reg [3:0] oscillation_count;        // 진동 카운트
    reg [3:0] oscillation_window;       // 진동 윈도우

    parameter ERROR_THRESHOLD = 32'd10; // 오차 임계값

   // Overshoot & Oscillation detection (100kHz)
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            oscillation_window <= 0;
            oscillation_count <= 0;
            oscillating_flag <= 0;
        end else if (clk_100k_enable) begin
            // Oscillation detection
            if (error_pos[31] != prev_error[31] && oscillation_window < 8'd100) begin
                oscillation_count <= oscillation_count + 1;
                oscillation_window <= oscillation_window + 1;
            end else if (oscillation_window >= 8'd100) begin
                oscillating_flag <= (oscillation_count >= 4);
                oscillation_count <= 0;
                oscillation_window <= 0;
            end else begin
                oscillation_window <= oscillation_window + 1;
            end
        end
    end
endmodule
