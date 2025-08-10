`timescale 1ns/1ps
// ============================================================================
// 6step_pwm.v  —  BLDC 6-Step Complementary PWM with Deadtime
// Inputs : sector_in(0..5), vref_q15(Q1.15), deadtime_cycles, enable, fault_n
// Outputs: pah/pal, pbh/pbl, pch/pcl  (complementary gate signals)
// Clock  : 100 MHz default (param), PWM default 25 kHz (param)
// ============================================================================

module bldc_6step_pwm #(
    parameter integer CLK_HZ       = 100_000_000,   // 시스템 클럭
    parameter integer PWM_HZ       = 25_000,        // PWM 주파수
    parameter integer MIN_PULSE_EN = 1              // 1: 최소펄스 억제 사용
)(
    input  wire        clk,
    input  wire        reset_n,           // Active-Low
    // 제어 입력
    input  wire [2:0]  sector_in,         // qep_to_sector_6step 등에서 입력(0..5)
    input  wire [15:0] vref_q15,          // Q1.15 (0..32767)
    input  wire [15:0] deadtime_cycles,   // 데드타임 사이클 수(예: 100MHz에서 1us=100)
    input  wire        enable,            // 코어 Enable
    input  wire        fault_n,           // 0이면 즉시 Kill
    // 게이트 출력 (Phase A/B/C High/Low)
    output wire        pah, pal,
    output wire        pbh, pbl,
    output wire        pch, pcl,
    // 디버그(옵션)
    output reg  [15:0] dbg_cnt,           // PWM 카운터
    output reg  [15:0] dbg_duty
);
    // ------------------------------------------------------------------------
    // 1) 캐리어/카운터
    // ------------------------------------------------------------------------
    localparam integer PWM_MAX_COUNT = CLK_HZ / PWM_HZ; // 정수 나눗셈 가정
    localparam integer CNT_W = (PWM_MAX_COUNT <= 1) ? 1 : $clog2(PWM_MAX_COUNT);

    reg [CNT_W-1:0] cnt;
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) cnt <= {CNT_W{1'b0}};
        else begin
            if (cnt == PWM_MAX_COUNT-1) cnt <= {CNT_W{1'b0}};
            else                        cnt <= cnt + 1'b1;
        end
    end
    // 디버그 노출
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) dbg_cnt <= 16'd0;
        else          dbg_cnt <= cnt[15:0];
    end

    // ------------------------------------------------------------------------
    // 2) 듀티 변환 (Q1.15 → 카운터 스케일) + 최소펄스 억제
    // ------------------------------------------------------------------------
    // duty_cnt = vref_q15/32767 * PWM_MAX_COUNT
    wire [31:0] mul_v = vref_q15 * PWM_MAX_COUNT;
    wire [15:0] duty_raw = (mul_v[30:15] > (PWM_MAX_COUNT-1)) ? (PWM_MAX_COUNT-1)
                                                              : mul_v[30:15];

    // 최소펄스: deadtime*2 이하 듀티는 출력 억제(하/상 전환 안정)
    wire [15:0] min_edge = (deadtime_cycles << 1);
    wire        too_small = (MIN_PULSE_EN && (duty_raw <= min_edge));
    wire        pwm_on    = (~too_small) && (cnt < duty_raw);

    // 디버그 노출
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) dbg_duty <= 16'd0;
        else          dbg_duty <= duty_raw;
    end

    // ------------------------------------------------------------------------
    // 3) 섹터별 상 배치 (한 상 고정 H/L + 한 상 PWM)
    //    표(60° 단위):
    //      0: A(PWM), B(L),   C(Off)
    //      1: A(H),   B(Off), C(PWM)
    //      2: A(Off), B(PWM), C(L)
    //      3: A(PWM), B(H),   C(Off)
    //      4: A(L),   B(Off), C(PWM)
    //      5: A(Off), B(H),   C(PWM)
    // ------------------------------------------------------------------------
    reg aH,aL,aPWM, bH,bL,bPWM, cH,cL,cPWM;
    always @(*) begin
        aH=0;aL=0;aPWM=0;
        bH=0;bL=0;bPWM=0;
        cH=0;cL=0;cPWM=0;
        case (sector_in)
            3'd0: begin aPWM=1; bL=1;              end
            3'd1: begin aH=1;   cPWM=1;            end
            3'd2: begin bPWM=1; cL=1;              end
            3'd3: begin aPWM=1; bH=1;              end
            3'd4: begin aL=1;   cPWM=1;            end
            default: begin bH=1; cPWM=1;           end // 3'd5
        endcase
    end

    // 섹터 배치 + 듀티 적용 (상단=1 / 하단=0)
    wire a_in = aH ? 1'b1 : (aL ? 1'b0 : (aPWM ? pwm_on : 1'b0));
    wire b_in = bH ? 1'b1 : (bL ? 1'b0 : (bPWM ? pwm_on : 1'b0));
    wire c_in = cH ? 1'b1 : (cL ? 1'b0 : (cPWM ? pwm_on : 1'b0));

    // ------------------------------------------------------------------------
    // 4) Enable/FAULT 마스킹
    // ------------------------------------------------------------------------
    wire global_en = enable & fault_n;
    wire a_en = global_en & (aH | aL | aPWM);
    wire b_en = global_en & (bH | bL | bPWM);
    wire c_en = global_en & (cH | cL | cPWM);

    // ------------------------------------------------------------------------
    // 5) 상보 출력 + 데드타임
    // ------------------------------------------------------------------------
    deadtime_comp_en #(.SYNC_OFF_OUTPUTS(1)) DA (
        .clk(clk), .reset_n(reset_n), .enable(a_en),
        .in_pwm(a_in), .dead_cycles(deadtime_cycles),
        .pwm_h(pah), .pwm_l(pal)
    );
    deadtime_comp_en #(.SYNC_OFF_OUTPUTS(1)) DB (
        .clk(clk), .reset_n(reset_n), .enable(b_en),
        .in_pwm(b_in), .dead_cycles(deadtime_cycles),
        .pwm_h(pbh), .pwm_l(pbl)
    );
    deadtime_comp_en #(.SYNC_OFF_OUTPUTS(1)) DC (
        .clk(clk), .reset_n(reset_n), .enable(c_en),
        .in_pwm(c_in), .dead_cycles(deadtime_cycles),
        .pwm_h(pch), .pwm_l(pcl)
    );

endmodule
