`timescale 1ns/1ps
// QEP(쿼드러처) 인코더 기반 BLDC 6-step 섹터 산출기
module qep_to_sector_6step_min #(
  parameter integer CPR         = 1024,  // 인코더 CPR (A상 사이클 수)
  parameter integer POLE_PAIRS  = 4,     // 모터 극쌍 수
  parameter integer CLK_HZ      = 100_000_000, // 시스템 클럭(Hz)
  parameter integer STEP_HZ     = 25_000       // 섹터 계산/갱신 주파수(보통 PWM 주파수)
)(
  input  wire        clk,
  input  wire        reset_n,
  input  wire signed [31:0] pos_cnt,   // 쿼드×4 누적 카운트(네 quadrature_encoder의 actual_position)
  output reg  [2:0]  sector            // 0..5
);
  // -------------------------------
  // 파생 상수
  // -------------------------------
  localparam integer CPR4  = CPR * 4;
  localparam integer EREV  = CPR4 * POLE_PAIRS;           // 전기 1회전 카운트
  localparam integer DIV   = CLK_HZ / STEP_HZ;            // 분주 (정수 나눗셈 가정)

  // 경계값(EREV * k/6) – 정수 산술 (합성 시 상수)
  localparam integer TH1 = (EREV * 1) / 6;
  localparam integer TH2 = (EREV * 2) / 6;
  localparam integer TH3 = (EREV * 3) / 6;
  localparam integer TH4 = (EREV * 4) / 6;
  localparam integer TH5 = (EREV * 5) / 6;

  // -------------------------------
  // 25 kHz enable 생성
  // -------------------------------
  localparam integer DIV_W = (DIV <= 1) ? 1 : $clog2(DIV);
  reg [DIV_W-1:0] div_cnt;
  wire ce_step = (div_cnt == 0);

  always @(posedge clk or negedge reset_n) begin
    if (!reset_n) div_cnt <= 0;
    else          div_cnt <= (div_cnt == DIV-1) ? {DIV_W{1'b0}} : (div_cnt + 1'b1);
  end

  // -------------------------------
  // 상태 레지스터
  // -------------------------------
  reg signed [31:0] pos_prev;
  reg signed [31:0] elec_acc;   // 0..EREV-1

  // -------------------------------
  // Δpos 계산 및 전기 카운트 누적
  // -------------------------------
  // 조합 경로
  wire signed [31:0] dpos      = pos_cnt - pos_prev;
  wire signed [47:0] incr_wide = $signed(dpos) * $signed(POLE_PAIRS[15:0]); // 32x16 → 48b
  wire signed [31:0] incr      = incr_wide[31:0];

  // 시퀀셜 업데이트
  always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
      // 리셋 시 현재 위치를 기준으로: 전기각 0°로 간주
      pos_prev <= pos_cnt;
      elec_acc <= 32'sd0;
    end else if (ce_step) begin
      pos_prev <= pos_cnt;

      // 누적
      elec_acc <= elec_acc + incr;

      // wrap (고속이면 두 번 반복해도 됨)
      if      (elec_acc >=  EREV) elec_acc <= elec_acc - EREV;
      else if (elec_acc <   0   ) elec_acc <= elec_acc + EREV;
    end
  end

  // -------------------------------
  // 섹터 결정 (나눗셈 없이 비교)
  // elec_acc * 6 vs EREV*k 비교 대신, 미리 계산한 경계와 비교
  // -------------------------------
  always @(posedge clk or negedge reset_n) begin
    if (!reset_n) sector <= 3'd0;
    else if (ce_step) begin
      // elec_acc ∈ [0, EREV)
      if      (elec_acc < TH1) sector <= 3'd0;
      else if (elec_acc < TH2) sector <= 3'd1;
      else if (elec_acc < TH3) sector <= 3'd2;
      else if (elec_acc < TH4) sector <= 3'd3;
      else if (elec_acc < TH5) sector <= 3'd4;
      else                     sector <= 3'd5;
    end
  end

endmodule
