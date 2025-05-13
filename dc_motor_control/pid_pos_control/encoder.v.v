module quadrature_encoder (
    input wire clk,                     // 클럭
    input wire reset_n,                        // 리셋 신호 (active high)
    input wire A,                              // A 신호
    input wire B,                              // B 신호
    input wire Index,                          // Index 신호
    output reg signed [31:0] actual_position,  // 실제 위치 카운트
    output reg error_flag                      // 에러 플래그
);

    // 내부 신호
    reg [2:0] A_prev;
    reg [2:0] B_prev;
    reg signed [31:0] position_count = 32'sd0;   // 위치 카운트
    reg signed [31:0] prev_index_position = 32'sd0; // 이전 Index 위치
    reg Index_sync, Index_sync_d;               // Index 신호 동기화 레지스터
    parameter signed PULSES_PER_REVOLUTION = 4096;     // 한 바퀴당 펄스 수
    parameter signed MAX_POSITION_ERROR = 100;         // 최대 허용 오차

    // A와 B 신호의 이전 상태 저장 (Shift Register)
    always @(posedge clk) begin
        A_prev <= {A_prev[1:0], A};
        B_prev <= {B_prev[1:0], B};
    end

    // Index 신호 동기화 (비동기 신호를 동기화)
    always @(posedge clk_150MHz or negedge reset_n) begin
        if (!reset_n) begin
            Index_sync <= 0;
            Index_sync_d <= 0;
        end else begin
            Index_sync <= Index;
            Index_sync_d <= Index_sync;
        end
    end

    // 위치 카운트 계산
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            position_count <= 32'sd0;
            prev_index_position <= 32'sd0;
            actual_position <= 32'sd0;
            error_flag <= 1'b0;
        end else begin
            // 위치 카운트 업데이트 (A/B 신호의 변화를 감지하여 계산)
            case ({A_prev[2], B_prev[2], A_prev[1], B_prev[1]})
                4'b0010, 4'b1011, 4'b1101, 4'b0100: position_count <= $signed(position_count) + 1; // 시계방향 증가
                4'b0001, 4'b0111, 4'b1110, 4'b1000: position_count <= $signed(position_count) - 1; // 반시계방향 감소
                default: position_count <= $signed(position_count); // 변화 없음
            endcase

            // Index 신호를 이용한 위치 보상
            if (Index_sync_d) begin // Index 신호의 동기화된 신호 처리
                if ($signed($signed(position_count) - $signed(prev_index_position)) > (PULSES_PER_REVOLUTION - MAX_POSITION_ERROR)) begin
                    // Index 신호 이후 위치가 예상 범위를 초과
                    position_count <= $signed(prev_index_position) + PULSES_PER_REVOLUTION;
                    error_flag <= 1'b1;
                end else if ($signed($signed(position_count) - $signed(prev_index_position)) < (-PULSES_PER_REVOLUTION + MAX_POSITION_ERROR)) begin
                    // Index 신호 이후 위치가 예상 범위를 초과 (반대 방향)
                    position_count <= $signed(prev_index_position) - PULSES_PER_REVOLUTION;
                    error_flag <= 1'b1;
                end else begin
                    // 정상 범위 내
                    error_flag <= 1'b0;
                end
                prev_index_position <= $signed(position_count); // Index 위치 업데이트
            end

            // 실제 위치 업데이트
            actual_position <= $signed(position_count);
        end
    end

endmodule
