`timescale 1ns / 1ps

module pi_velocity_controller (
    input wire clk,                      // 원래 클럭 (100mhz)
    input wire reset_n,                  // 비동기 리셋 (Active Low)
    input wire signed [31:0] desired_vel, // 목표 속도도
    input wire signed [31:0] actual_pos,  // 실제 위치
    input wire [15:0] Kp_axi,             // 비례 게인
    input wire [15:0] Ki_axi,             // 적분 게인
    output reg signed [31:0] actual_vel, // 실제 속도
    output reg signed [15:0] control_signal // PID 제어 신호 출력

);

    // 내부 변수
    reg signed [47:0] integral;           // 적분 값
    reg signed [31:0] derivative;         // 미분 값
    reg signed [31:0] prev_error;         // 이전 오차
    reg signed [47:0] pi_output_p;       // 비례 항 중간 값
    reg signed [47:0] pi_output_i;       // 적분 항 중간 값
    reg signed [47:0] pi_output;         // PID 출력 중간 값
    reg signed [15:0] pi_output_mid; // PID 출력 중간 값 (16비트로 변환)
 
    // 100MHz -> 20kHz 변환을 위한 파라미터
    parameter DIVIDER = 5000; // 100MHz / 20kHz

    reg [12:0] clk_div_counter; // 16비트 분주기 카운터
    reg clk_20k;           // 20kHz 클럭 신호

    // 20kHz 클럭 생성
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            clk_div_counter <= 13'd0;
            clk_20k <= 1'b0;
        end else begin
            if (clk_div_counter == DIVIDER - 1) begin // 5000주기로 20kHz 신호 생성
                clk_div_counter <= 13'd0;
                clk_20k <= ~clk_20k; // 20kHz 클럭 신호 생성
            end else begin
                clk_div_counter <= clk_div_counter + 1'b1;
            end
        end
    end
    
    // error_vel 계산을 위한 reg
    reg signed [31:0] prev_pos;    // 이전 위치
    reg signed [31:0] current_pos; // 현재 위치
    reg signed [31:0] error_vel;   // 속도 오차
    reg signed [31:0] delta_pos;   // 위치 차이
    reg signed [31:0] delta_pos_sum; // 위치 차이 누적
    reg [3:0] sample_count;        // 샘플 카운트

    // 오차 계산 
    always @(posedge clk_20k or negedge reset_n) begin
        if (!reset_n) begin
            actual_vel <= 32'sd0;
            prev_pos <= 32'sd0;
            current_pos <= 32'sd0;
            error_vel <= 32'sd0;
            delta_pos <= 32'sd0;
            delta_pos_sum <= 32'sd0;
            sample_count <= 0; // 샘플 카운트 초기화
        end else begin
            delta_pos <= actual_pos - prev_pos; // 위치 차이 계산
            prev_pos <= actual_pos;            // 이전 위치 업데이트
            delta_pos_sum <= delta_pos_sum + delta_pos; // 위치 차이 누적

            sample_count <= sample_count + 1; // 샘플 카운트 증가

            error_vel <= desired_vel - actual_vel; // 속도 오차 계산
            
            // 속도 오차 계산
            if (sample_count == 9) begin // 
                sample_count <= 0; // 샘플 카운트 초기화
                actual_vel <= delta_pos_sum; // 속도 계산
                delta_pos_sum <= 32'sd0; // 누적 위치 차이 초기화
            end 
        end
    end

    // 적분 계산
    always @(posedge clk_20k or negedge reset_n) begin
        if (!reset_n) begin
            integral <= 48'sd0;
        end else begin
            if (control_signal >= 16'sd3900 || control_signal <= -16'sd3900) begin
                integral <= integral; // 적분값을 유지 
            end else if (error_vel < 3 && error_vel > -3) begin
                integral <= integral; // 적분값을 유지
            end else begin
                integral <= integral + error_vel; 
            end
        end
    end


    // 비례 항 계산 (Stage 1)
    always @(posedge clk_20k or negedge reset_n) begin
        if (!reset_n) begin
            pi_output_p <= 48'sd0;
        end else begin
            pi_output_p <= Kp_axi * error_vel;
        end
    end

    // 적분 항 계산 (Stage 2)
    always @(posedge clk_20k or negedge reset_n) begin
        if (!reset_n) begin
            pi_output_i <= 48'sd0;
        end else begin
            pi_output_i <= Ki_axi * integral; // 적분 항 계산
        end
    end

    // // 미분 항 계산 (Stage 3)
    // always @(posedge clk_10k or negedge reset_n) begin
    //     if (!reset_n) begin
    //         pid_output_d <= 32'sd0;
    //     end else begin
    //         pid_output_d <= $signed(Kd_axi) * (error - prev_error);
    //     end
    // end

    // PID 출력 계산 및 제한 (Stage 4)
    always @(posedge clk_20k or negedge reset_n) begin
        if (!reset_n) begin
            pi_output <= 48'sd0;
            control_signal <= 16'sd0;
            pi_output_mid <= 16'sd0;
        end else begin
            pi_output <= pi_output_p + pi_output_i;
            // pid_output <= pid_output_p + pid_output_i + pid_output_d;
            pi_output_mid <= pi_output >>> 32; // 16비트로 변환
            // 출력 신호 제한
            if (pi_output_mid > 16'sd4000) begin
                control_signal <= 16'sd4000;
            end else if (pi_output_mid < -16'sd4000) begin
                control_signal <= -16'sd4000;
            end else begin
                control_signal <= pi_output_mid; // PID 출력 신호
            end
        end
    end

endmodule
