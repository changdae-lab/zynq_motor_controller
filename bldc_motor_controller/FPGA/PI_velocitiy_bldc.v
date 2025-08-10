`timescale 1ns / 1ps
module pi_speed_controller #(
    parameter integer CMD_MAX = 10000        // 전류 명령 해상도(±CMD_MAX) 전류 센서에 따라 선택
)(
    input  wire         clk,                      // 100MHz
    input  wire         reset_n,                  // Active-low
    input  wire         clk_20k_enable,           // 외부 20kHz enable(권장)
    input  wire signed [31:0] desired_speed,      // ticks/sample
    input  wire signed [31:0] actual_speed,       // ticks/sample
    input  wire [15:0]  Kp_vel_axi,               // Q8.8
    input  wire [15:0]  Ki_vel_axi,               // Q8.8
    output reg  signed [15:0] control_signal,     // 전류 명령(±CMD_MAX)
    output reg           sat_flag                 // 포화 여부(안티윈드업 지표)
);
    // PI 변수
    reg signed [31:0] error_speed, integral;
    reg signed [47:0] p_term, i_term, u_sum;
    reg signed [47:0] u_int; // Q40.8 -> int

    // 오차
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) error_speed <= 32'sd0;
        else if (clk_20k_enable)  error_speed <= desired_speed - actual_speed;
    end

    // 적분(포화 시 정지)
    parameter signed [31:0] INTEGRAL_LIMIT = 32'sd2000000000;
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) integral <= 32'sd0;
        else if (clk_20k_enable) begin
            if (sat_flag) begin
                // 포화 중엔 서서히 감쇠
                integral <= integral - (integral >>> 6);
            end else begin
                if ((integral + error_speed) >  INTEGRAL_LIMIT) integral <=  INTEGRAL_LIMIT;
                else if ((integral + error_speed) < -INTEGRAL_LIMIT) integral <= -INTEGRAL_LIMIT;
                else integral <= integral + error_speed;
            end
        end
    end

    // P/I
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) p_term <= 48'sd0;
        else if (clk_20k_enable) p_term <= $signed(Kp_vel_axi) * error_speed;
    end
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) i_term <= 48'sd0;
        else if (clk_20k_enable) i_term <= $signed(Ki_vel_axi) * integral;
    end

    // 합산/포화
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            u_sum<=0; u_int<=0; control_signal<=0; sat_flag<=1'b0;
        end else if (clk_20k_enable) begin
            u_sum <= p_term + i_term;       // Q40.8
            u_int <= u_sum >>> 8;           // -> 정수

            // 소수점 아래는 버리고, 안전 범위 검사
            if (u_int >  $signed(CMD_MAX)) begin
                control_signal <=  $signed(CMD_MAX);
                sat_flag <= 1'b1;
            end else if (u_int < -$signed(CMD_MAX)) begin
                control_signal <= -$signed(CMD_MAX);
                sat_flag <= 1'b1;
            end else begin
                control_signal <= u_int[15:0]; // 안전 캐스팅
                sat_flag <= 1'b0;
            end
        end
    end
endmodule
