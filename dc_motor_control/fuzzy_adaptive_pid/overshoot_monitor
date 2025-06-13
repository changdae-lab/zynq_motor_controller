`timescale 1ns / 1ps

module overshoot_detection (
    input wire clk,
    input wire reset_n,
    input wire clk_100k_enable,
    input wire signed [31:0] error_pos,
    input wire signed [31:0] prev_error,
    input wire signed [31:0] actual_pos,
    input wire signed [31:0] desired_pos,
    output reg overshoot_detected
);

    // Parameters
    parameter signed [31:0] ERROR_THRESHOLD = 32'd100;
    parameter HOLD_CYCLES = 4'd10;

    // FSM States
    parameter IDLE = 2'd0,
              WAIT_CROSS = 2'd1,
              HOLD_SIGN = 2'd2,
              OVERSHOOT_CONFIRMED = 2'd3;
    reg [1:0] overshoot_state;

    // Internal Registers
    reg overshoot_sign;
    reg [3:0] overshoot_hold_count;
    reg [4:0] sign_history;

    // FSM Logic
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            overshoot_state <= IDLE;
            overshoot_detected <= 0;
            overshoot_sign <= 0;
            overshoot_hold_count <= 0;
            sign_history <= 0;
        end else if (clk_100k_enable) begin
            case (overshoot_state)
                IDLE: begin
                    overshoot_detected <= 0;
                    overshoot_hold_count <= 0;
                    sign_history <= 0;
                    if ((error_pos > ERROR_THRESHOLD) || (error_pos < -ERROR_THRESHOLD)) begin
                        overshoot_state <= WAIT_CROSS;
                    end
                end
                WAIT_CROSS: begin
                    if ((prev_error[31] != error_pos[31]) && (error_pos != 0)) begin
                        overshoot_sign <= error_pos[31];
                        overshoot_hold_count <= 0;
                        sign_history <= 0;
                        overshoot_state <= HOLD_SIGN;
                    end else begin
                        overshoot_state <= IDLE;
                    end
                end
                HOLD_SIGN: begin
                    sign_history <= {sign_history[3:0], (error_pos[31] == overshoot_sign) && (error_pos != 0)};
                    overshoot_hold_count <= overshoot_hold_count + 1;
                    if (overshoot_hold_count >= HOLD_CYCLES) begin
                        if ((sign_history[4] + sign_history[3] + sign_history[2] + sign_history[1] + sign_history[0]) >= 4) begin
                            overshoot_state <= OVERSHOOT_CONFIRMED;
                        end else begin
                            overshoot_state <= IDLE;
                        end
                    end
                end
                OVERSHOOT_CONFIRMED: begin
                    if ((actual_pos - desired_pos <= ERROR_THRESHOLD) &&
                        (desired_pos - actual_pos <= ERROR_THRESHOLD)) begin
                        overshoot_state <= IDLE;
                    end else begin
                        overshoot_detected <= 1;
                    end
                end
            endcase
        end
    end
endmodule
