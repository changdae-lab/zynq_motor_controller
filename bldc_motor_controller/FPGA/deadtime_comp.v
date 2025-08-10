// --------------------------------------------------------------------------
// Deadtime complementary driver with enable masking
// in_pwm=1 → High-side ON, in_pwm=0 → Low-side ON
// enable=0 → both OFF (동기/비동기 Off 옵션)
// --------------------------------------------------------------------------
module deadtime_comp_en #(
    parameter SYNC_OFF_OUTPUTS = 1 // 1: clk에 동기화하여 OFF, 0: 즉시 OFF
)(
    input  wire        clk,
    input  wire        reset_n,
    input  wire        enable,
    input  wire        in_pwm,
    input  wire [15:0] dead_cycles,
    output reg         pwm_h,
    output reg         pwm_l
);
    reg in_d;
    reg [15:0] dcnt;

    // 비동기 OFF 옵션 처리
    wire en_i = enable;

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            in_d  <= 1'b0;
            dcnt  <= 16'd0;
            pwm_h <= 1'b0;
            pwm_l <= 1'b0;
        end else begin
            if (!en_i) begin
                in_d  <= in_pwm;
                dcnt  <= 16'd0;
                pwm_h <= 1'b0;
                pwm_l <= 1'b0;
            end else begin
                if (in_pwm != in_d) begin
                    // 전환 시작: 데드타임 구간
                    in_d  <= in_pwm;
                    dcnt  <= dead_cycles;
                    pwm_h <= 1'b0;
                    pwm_l <= 1'b0;
                end else if (dcnt != 16'd0) begin
                    dcnt  <= dcnt - 16'd1;
                    pwm_h <= 1'b0;
                    pwm_l <= 1'b0;
                end else begin
                    // 정상 출력 (상보)
                    pwm_h <=  in_d;
                    pwm_l <= ~in_d;
                end
            end
        end
    end
endmodule
