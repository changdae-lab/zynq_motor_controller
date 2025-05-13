module quadrature_encoder (
    input wire clk,                            // 시스템 클럭 (e.g. 100 MHz)
    input wire reset_n,                        // Active low reset
    input wire A,                              // 비동기 A 신호
    input wire B,                              // 비동기 B 신호
    input wire Index,                          // 비동기 Index 신호
    output reg signed [31:0] actual_position  // 실제 위치 카운트
);     

    // Majority Voting 필터링
    reg [4:0] A_filter, B_filter, Index_filter;
    reg A_filtered, B_filtered, Index_filtered;
    reg [3:0] A_sum, B_sum, Index_sum;

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            A_filter <= 5'b0;
            B_filter <= 5'b0;
            Index_filter <= 5'b0;
            A_filtered <= 0;
            B_filtered <= 0;
            Index_filtered <= 0;
        end else begin
            // 4비트 Shift Register를 통한 필터링
            A_filter <= {A_filter[3:0], A};
            B_filter <= {B_filter[3:0], B};
            Index_filter <= {Index_filter[3:0], Index};

            // 합계 계산 (majority voting용)
            A_sum <= A_filter[0] + A_filter[1] + A_filter[2] + A_filter[3] + A_filter[4];
            B_sum <= B_filter[0] + B_filter[1] + B_filter[2] + B_filter[3] + B_filter[4];
            Index_sum <= Index_filter[0] + Index_filter[1] + Index_filter[2] + Index_filter[3] + Index_filter[4];


            if (A_sum >= 3'd4) begin     
                A_filtered <= 1'b1;
            end else if (A_sum <= 3'd1) begin
                A_filtered <= 1'b0;
            end else begin
                A_filtered <= A_filtered; // 이전 상태 유지
            end

            if (B_sum >= 3'd4) begin
                B_filtered <= 1'b1;
            end else if (B_sum <= 1) begin
                B_filtered <= 1'b0;
            end else begin
                B_filtered <= B_filtered; // 이전 상태 유지
            end

            if (Index_sum >= 3'd4) begin
                Index_filtered <= 1'b1;
            end else if (Index_sum <= 3'd1) begin
                Index_filtered <= 1'b0;
            end else begin
                Index_filtered <= Index_filtered; // 이전 상태 유지
            end
        end    
    end

    // 동기화를 위한 3단 플립플롭 레지스터 선언
    reg [2:0] A_sync;
    reg [2:0] B_sync;
    reg [2:0] Index_sync;

    // A, B, Index 신호 3단 동기화
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            A_sync <= 3'b000;
            B_sync <= 3'b000;
            Index_sync <= 3'b000;
        end else begin
            A_sync <= {A_sync[1:0], A_filtered};
            B_sync <= {B_sync[1:0], B_filtered};
            Index_sync <= {Index_sync[1:0], Index_filtered};
        end
    end

    // 위치 카운트 및 에러 플래그
    reg signed [31:0] position_count;         
    reg signed [31:0] prev_index_position;    
    parameter signed PULSES_PER_REVOLUTION = 4096;     
    parameter signed MAX_POSITION_ERROR = 100;
    reg error_flag; // 에러 플래그    



    // 위치 카운트 계산 (동기화된 A, B 신호 사용)
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            position_count <= 32'sd0;
            prev_index_position <= 32'sd0;
            actual_position <= 32'sd0;
            error_flag <= 1'b0;
        end else begin
            // A, B 신호의 에지 검출을 통한 방향 결정
            case ({A_sync[2], B_sync[2], A_sync[1], B_sync[1]})
                4'b0010, 4'b1011, 4'b1101, 4'b0100: position_count <= position_count + 1; // CW 방향 증가
                4'b0001, 4'b0111, 4'b1110, 4'b1000: position_count <= position_count - 1; // CCW 방향 감소
                default: position_count <= position_count; // 변화 없음
            endcase

            // Index 신호의 rising edge 검출
            if (Index_sync[2] && !Index_sync[1]) begin
                if ((position_count - prev_index_position) > (PULSES_PER_REVOLUTION - MAX_POSITION_ERROR)) begin
                    // 정방향 초과
                    position_count <= prev_index_position + PULSES_PER_REVOLUTION;
                    error_flag <= 1'b1;
                end else if ((position_count - prev_index_position) < (-PULSES_PER_REVOLUTION + MAX_POSITION_ERROR)) begin
                    // 역방향 초과
                    position_count <= prev_index_position - PULSES_PER_REVOLUTION;
                    error_flag <= 1'b1;
                end else begin
                    // 정상 범위 내
                    error_flag <= 1'b0;
                end
                prev_index_position <= position_count; // Index 위치 업데이트
            end

            // 실제 위치 업데이트
            actual_position <= position_count;
        end
    end

endmodule
