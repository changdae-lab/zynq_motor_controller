`timescale 1ns / 1ps

module pid_controller (
    input wire clk,                      // 원래 클럭 (100kHz)
    input wire reset_n,                  // 비동기 리셋 (Active Low)
    input wire signed [31:0] desired_pos, // 목표 위치
    input wire signed [31:0] actual_pos,  // 실제 위치
    input wire [15:0] Kp_axi,             // 비례 게인
    input wire [15:0] Ki_axi,             // 적분 게인
    input wire [15:0] Kd_axi,             // 미분 게인
    output reg signed [15:0] control_signal // PID 제어 신호 출력
);

    // 내부 변수
    reg signed [31:0] error;              // 현재 오차
    reg signed [47:0] integral;           // 적분 값
    reg signed [31:0] derivative;         // 미분 값
    reg signed [31:0] prev_error;         // 이전 오차
    reg signed [31:0] pid_output_p;       // 비례 항 중간 값
    reg signed [47:0] pid_output_i;       // 적분 항 중간 값
    reg signed [31:0] pid_output_d;       // 미분 항 중간 값
    reg signed [47:0] pid_output;         // PID 출력 중간 값

    // 클럭 분주기 (100kHz -> 10kHz)
    reg [3:0] clk_div_counter; // 4비트 분주기 카운터
    wire clk_10k;              // 10kHz 클럭 신호

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            clk_div_counter <= 4'd0;
        end else begin
            if (clk_div_counter == 4'd9) begin // 10주기로 10kHz 신호 생성
                clk_div_counter <= 4'd0;
            end else begin
                clk_div_counter <= clk_div_counter + 1'b1;
            end
        end
    end

    assign clk_10k = (clk_div_counter == 4'd9); // 10kHz 클럭 신호 생성

    // 오차 계산 (이건 10kHz 클럭으로만 계산하도록)
    always @(posedge clk_10k or negedge reset_n) begin
        if (!reset_n) begin
            error <= 32'sd0;
            prev_error <= 32'sd0;
        end else begin
            error <= desired_pos - actual_pos;
            prev_error <= error;
        end
    end

    // 적분 계산 (이건 10kHz 클럭으로만 계산하도록)
    always @(posedge clk_10k or negedge reset_n) begin
        if (!reset_n) begin
            integral <= 48'sd0;
        end else begin
            // 출력이 제한 범위를 벗어나면 적분값을 절반으로 줄임
            if (control_signal > 16'sd9500 || control_signal < -16'sd9500) begin
                integral <= integral >>> 1; 
            end 
            // 오차가 일정 범위 내에 있으면 적분값을 유지
            else if (error < 32'sd1000 && error > -32'sd1000) begin
                // 적분값을 누적하지 않음
                integral <= integral; 
            end 
            // 오차가 일정 범위 외부에 있으면 적분값을 누적
            else begin
                integral <= integral + error; 
            end
        end
    end

    // 비례 항 계산 (Stage 1)
    always @(posedge clk_10k or negedge reset_n) begin
        if (!reset_n) begin
            pid_output_p <= 32'sd0;
        end else begin
            pid_output_p <= $signed(Kp_axi) * error;
        end
    end

    // 적분 항 계산 (Stage 2)
    always @(posedge clk_10k or negedge reset_n) begin
        if (!reset_n) begin
            pid_output_i <= 48'sd0;
        end else begin
            pid_output_i <= $signed(Ki_axi) * integral;
        end
    end

    // 미분 항 계산 (Stage 3)
    always @(posedge clk_10k or negedge reset_n) begin
        if (!reset_n) begin
            pid_output_d <= 32'sd0;
        end else begin
            pid_output_d <= $signed(Kd_axi) * (error - prev_error);
        end
    end

    reg signed [47:0] pid_output_mid;

    // PID 출력 계산 및 제한 (Stage 4)
    always @(posedge clk_10k or negedge reset_n) begin
        if (!reset_n) begin
            pid_output <= 48'sd0;
            control_signal <= 16'sd0;
        end else begin
            pid_output <= pid_output_p + pid_output_i + pid_output_d;
            pid_output_mid <= pid_output >>> 16;

            // 출력 신호 제한
            if (pid_output_mid > 16'sd10000) begin
                control_signal <= 16'sd10000;
            end else if (pid_output_mid < -16'sd10000) begin
                control_signal <= -16'sd10000;
            end else begin
                control_signal <= pid_output_mid[15:0];
            end
        end
    end

endmodule