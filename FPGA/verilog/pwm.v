`timescale 1ns / 1ps

module pwm_generator_bidirectional (
    input wire clk,                    // 100MHz 클럭 입력
    input wire reset_n,                // 비동기 리셋 (Active Low)
    input wire signed [15:0] pid_control_signal, // PID 제어 입력 (-4000 ~ 4000)
    output reg dir1,                   // 방향 제어 1
    output reg dir2,                   // 방향 제어 2
    output reg pwm_out                 // PWM 출력
);

    // PWM 파라미터 (25kHz 기준)
    parameter [15:0] MAX_COUNT = 4000;             // 100MHz / 25kHz = 4000
    parameter integer DEADTIME_CYCLES = 3;         // 3 클럭 = 30ns

    // 내부 레지스터
    reg [15:0] counter;
    reg [15:0] duty_cycle;

    reg [1:0] direction_state;                     // 00: 정지, 01: CW, 10: CCW
    reg [1:0] next_direction_state;
    reg [2:0] deadtime_counter;                    // 최대 7 사이클 deadtime 지원

    reg dir1_raw, dir2_raw;                        // 내부 raw 방향 제어 신호

    // 방향 및 duty_cycle 계산 + deadtime 로직
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            direction_state <= 2'b00;
            next_direction_state <= 2'b00;
            deadtime_counter <= 0;
            duty_cycle <= 16'd0;
            dir1_raw <= 1'b0;
            dir2_raw <= 1'b0;
        end else begin
            // 방향 상태 계산
            if (pid_control_signal > 0) begin
                next_direction_state <= 2'b01; // CW
                duty_cycle <= pid_control_signal;
            end else if (pid_control_signal < 0) begin
                next_direction_state <= 2'b10; // CCW
                duty_cycle <= -pid_control_signal;
            end else begin
                next_direction_state <= 2'b00;
                duty_cycle <= 16'd0;
            end

            // 방향 변경 시 deadtime 시작
            if (next_direction_state != direction_state) begin
                deadtime_counter <= DEADTIME_CYCLES;
                dir1_raw <= 1'b0;
                dir2_raw <= 1'b0;
                direction_state <= next_direction_state;
            end else begin
                if (deadtime_counter != 0) begin
                    deadtime_counter <= deadtime_counter - 1;
                end else begin
                    // deadtime 후 방향 출력 활성화
                    case (direction_state)
                        2'b01: begin dir1_raw <= 1'b1; dir2_raw <= 1'b0; end // CW
                        2'b10: begin dir1_raw <= 1'b0; dir2_raw <= 1'b1; end // CCW
                        default: begin dir1_raw <= 1'b0; dir2_raw <= 1'b0; end
                    endcase
                end
            end
        end
    end

    // PWM 신호 생성
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            counter <= 16'd0;
            pwm_out <= 1'b0;
        end else begin
            // PWM 카운터 증가
            if (counter < MAX_COUNT - 1) begin
                counter <= counter + 1;
            end else begin
                counter <= 16'd0;
            end

            // PWM 출력
            if (counter < duty_cycle) begin
                pwm_out <= 1'b1;
            end else begin
                pwm_out <= 1'b0;
            end
        end
    end

    // 출력 버퍼 연결
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            dir1 <= 1'b0;
            dir2 <= 1'b0;
        end else begin
            dir1 <= dir1_raw;
            dir2 <= dir2_raw;
        end
    end


endmodule
