`timescale 1ns / 1ps
module pi_current_controller #(
    parameter integer CMD_MAX = 10000,           // 전류 명령/측정 해상도(±CMD_MAX)
    parameter integer INV_CMDMAX_Q15 = (32767 << 15) / CMD_MAX // Q1.15 스케일 변환 상수
)(
    input  wire         clk,                     // 100MHz
    input  wire         reset_n,                 // Active-low
    input  wire         clk_20k_enable,          // 외부 20kHz enable(권장)
    input  wire signed [31:0] desired_current,   // 전류 명령(±CMD_MAX)
    input  wire signed [31:0] actual_current,    // 전류 측정값(ADC 스케일링 후)
    input  wire [15:0]  Kp_cur_axi,               // Q8.8
    input  wire [15:0]  Ki_cur_axi,               // Q8.8
    input  wire [15:0]  VMAX_Q15,                 // 최대 전압명령(Q1.15)
    output reg  [15:0]  vref_q15,                 // PWM용 전압명령(Q1.15)
    output reg           dir,                     // 1: 정방향, 0: 역방향
    output reg           sat_flag                 // 포화 여부(안티윈드업 지표)
);
    // 내부 변수
    reg signed [31:0] error_cur, integral;
    reg signed [47:0] p_term, i_term, u_sum;    // Q40.8
    reg signed [47:0] u_int;                    // Q40.8 → 정수 근사

    // 적분 한계(충분히 큰 값)
    parameter signed [31:0] INTEGRAL_LIMIT = 32'sd2000000000;

    // 1) 오차 계산
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) error_cur <= 32'sd0;
        else if (clk_20k_enable) error_cur <= desired_current - actual_current;
    end

    // 2) 적분(포화 시 감쇠 적용)
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) integral <= 32'sd0;
        else if (clk_20k_enable) begin
            if (sat_flag) begin
                // 포화 중에는 적분항을 서서히 감쇠 → windup 방지
                integral <= integral - (integral >>> 6);
            end else begin
                if ((integral + error_cur) >  INTEGRAL_LIMIT)       integral <=  INTEGRAL_LIMIT;
                else if ((integral + error_cur) < -INTEGRAL_LIMIT)  integral <= -INTEGRAL_LIMIT;
                else                                                integral <= integral + error_cur;
            end
        end
    end

    // 3) P항 / I항 계산
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) p_term <= 48'sd0;
        else if (clk_20k_enable) p_term <= $signed(Kp_cur_axi) * error_cur;
    end
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) i_term <= 48'sd0;
        else if (clk_20k_enable) i_term <= $signed(Ki_cur_axi) * integral;
    end

    // 4) PI 합산
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            u_sum <= 48'sd0; u_int <= 48'sd0;
            vref_q15 <= 16'd0; dir <= 1'b1; sat_flag <= 1'b0;
        end else if (clk_20k_enable) begin
            // PI 출력 합산
            u_sum <= p_term + i_term;     // Q40.8
            u_int <= u_sum >>> 8;         // 정수 근사

            // 방향 비트
            dir <= ~u_int[47];

            // 절대값 계산
            reg [47:0] mag;
            mag = u_int[47] ? (~u_int + 48'd1) : u_int;

            // 정수 → Q1.15 변환 (나눗셈 회피)
            reg [63:0] q15_scaled;
            q15_scaled = $signed(mag[31:0]) * $signed(INV_CMDMAX_Q15);
            reg [31:0] q15_val;
            q15_val = q15_scaled >> 15;

            // 범위 제한(풀스케일 검사)
            reg [15:0] q15_sat;
            q15_sat = (|q15_val[31:16]) ? 16'h7FFF : q15_val[15:0];

            // 최대 전압 포화
            if (q15_sat > VMAX_Q15) begin
                vref_q15 <= VMAX_Q15;
                sat_flag <= 1'b1;
            end else begin
                vref_q15 <= q15_sat;
                sat_flag <= 1'b0;
            end
        end
    end
endmodule
