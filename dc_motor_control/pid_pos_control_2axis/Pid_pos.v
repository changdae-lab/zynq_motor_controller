`timescale 1ns / 1ps

module pi_velocity_controller (
    input wire clk,                      // 원래 클럭 (100mhz)
    input wire reset_n,                  // 비동기 리셋 (Active Low)
    input wire signed [31:0] desired_pos, // 목표 속도도
    input wire signed [31:0] actual_pos,  // 실제 위치
    input wire [15:0] Kp_axi,             // 비례 게인
    input wire [15:0] Ki_axi,             // 적분 게인
    input wire [15:0] Kd_axi,             // 미분 게인
    output reg signed [15:0] control_signal // PID 제어 신호 출력

);
 
    // 100MHz → 20kHz 분주기용 Enable 신호 생성
    parameter DIVIDER = 5000; // 100MHz / 20kHz

    reg [12:0] clk_div_counter;
    wire clk_20k_enable;

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            clk_div_counter <= 0;
        end else if (clk_div_counter == DIVIDER - 1) begin
            clk_div_counter <= 0;
        end else begin
            clk_div_counter <= clk_div_counter + 1;
        end
    end

    assign clk_20k_enable = (clk_div_counter == 0);
    
    // PID 제어 변수
    reg signed [31:0] error_pos;
    reg signed [31:0] prev_error;
    reg signed [31:0] derivative;
    reg signed [31:0] integral;
    reg signed [31:0] delta_error;

    reg signed [47:0] pid_output_p;
    reg signed [47:0] pid_output_i;
    reg signed [47:0] pid_output_d;
    reg signed [47:0] pid_output;
    reg signed [40:0] pid_output_mid;

    reg signed [31:0] actual_pos_ff; // 실제 위치
    reg signed [31:0] desired_pos_ff; // 목표 위치

    // 오차 계산 
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            error_pos <= 32'sd0;
            prev_error <= 32'sd0;

            delta_error <= 32'sd0;
            actual_pos_ff <= 32'sd0;
            
            desired_pos_ff <= 32'sd0;
        end else begin
            if (clk_20k_enable) begin
                actual_pos_ff <= actual_pos; // 실제 위치 업데이트
                desired_pos_ff <= desired_pos; // 목표 위치 업데이트

                error_pos <= desired_pos_ff - actual_pos_ff; // 위치 오차 계산
                prev_error <= error_pos; // 이전 오차 저장

                delta_error <= error_pos - prev_error; // 오차 변화량 계산
            end
        end
    end

    // 적분 계산 (anti-windup + overflow clamp)
    parameter signed [31:0] INTEGRAL_LIMIT = 32'sd2000000000; // 약 20억

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            integral <= 32'sd0;
        end else begin
            if (clk_20k_enable) begin
                if (control_signal >= 16'sd3950 || control_signal <= -16'sd3950) begin
                    integral <= integral;
                end else if (error_pos < desired_pos + 2 && error_pos > desired_pos - 2) begin
                    integral <= integral - (integral >>> 6); 
                end else begin
                    if ((integral + error_pos) > INTEGRAL_LIMIT)
                        integral <= INTEGRAL_LIMIT;
                    else if ((integral + error_pos) < -INTEGRAL_LIMIT)
                        integral <= -INTEGRAL_LIMIT;
                    else
                        integral <= integral + error_pos;
                end
            end
        end
    end


     // P 항 계산: Q8.8 × int32 = Q40.8 → 48비트
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            pid_output_p <= 48'sd0;
        end else begin
            if (clk_20k_enable) begin
                pid_output_p <= $signed(Kp_axi) * error_pos;
            end
        end
    end

    // I 항 계산: Q8.8 × int32 = Q40.8
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            pid_output_i <= 48'sd0;
        end else begin
            if (clk_20k_enable) begin
                pid_output_i <= $signed(Ki_axi) * integral;
            end
        end
    end

    // D 항 계산: Q8.8 × int32 = Q40.8
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            pid_output_d <= 48'sd0;
        end else begin
            if (clk_20k_enable) begin
                pid_output_d <= $signed(Kd_axi) * delta_error;
            end       
        end
    end

    // PID 출력 계산 및 saturation
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            pid_output <= 48'sd0;
            pid_output_mid <= 40'sd0;
            control_signal <= 16'sd0;
        end else begin
            if (clk_20k_enable) begin
                // PID 출력 계산
                pid_output <= pid_output_p + pid_output_i + pid_output_d;
                pid_output_mid <= pid_output >>> 8; // Q40.8 → int40 변환

                // saturation 처리
                if (pid_output_mid > 16'sd4000)
                    control_signal <= 16'sd4000;
                else if (pid_output_mid < -16'sd4000)
                    control_signal <= -16'sd4000;
                else
                    control_signal <= pid_output_mid; // 안전한 다운캐스팅
            end
        end
    end

endmodule