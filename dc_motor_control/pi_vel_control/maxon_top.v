`timescale 1 ns / 1 ps

module maxon_top (
    // AXI Interface
    input wire s00_axi_aclk,
    input wire s00_axi_aresetn,
    input wire [3:0] s00_axi_awaddr,
    input wire [2:0] s00_axi_awprot,
    input wire s00_axi_awvalid,
    output wire s00_axi_awready,
    input wire [31:0] s00_axi_wdata,
    input wire [3:0] s00_axi_wstrb,
    input wire s00_axi_wvalid,
    output wire s00_axi_wready,
    output wire [1:0] s00_axi_bresp,
    output wire s00_axi_bvalid,
    input wire s00_axi_bready,
    input wire [3:0] s00_axi_araddr,
    input wire [2:0] s00_axi_arprot,
    input wire s00_axi_arvalid,
    output wire s00_axi_arready,
    output wire [31:0] s00_axi_rdata,
    output wire [1:0] s00_axi_rresp,
    output wire s00_axi_rvalid,
    input wire s00_axi_rready,

    // Motor and Encoder Signals
    input wire clk,                  // 클럭
    input wire reset_n,              // 리셋 신호 (Active Low)
    input wire encoder_a,            // 엔코더 A 채널
    input wire encoder_b,            // 엔코더 B 채널
    input wire encoder_index,        // 엔코더 Index 신호
    output wire dir1,                // 방향 제어 1
    output wire dir2,                // 방향 제어 2
    output wire pwm_out,             // PWM 출력

    // 디버깅 LED 출력
    output reg [3:0] led             // LED 디버깅 출력
);

    // 내부 신호 정의
    wire [15:0] kp_init;               // Kp 초기 값
    wire [15:0] ki_init;               // Ki 초기 값
    wire [15:0] kd_init;               // Kd 초기 값
    wire signed [31:0] desired_vel;    // 목표 속도
    wire signed [31:0] actual_vel;     // 실제 속도도
    wire signed [31:0] actual_pos;    // 실제 위치
    wire signed [15:0] internal_control_signal; // 내부 제어 신호

    // AXI 슬레이브 모듈 인스턴스화
    (* dont_touch = "true" *)
    myip_v1_0 #(
        .C_S00_AXI_DATA_WIDTH(32),
        .C_S00_AXI_ADDR_WIDTH(4)
    ) u_myip_v1_0 (
        .kp_init(kp_init),
        .ki_init(ki_init),
        .desired_vel(desired_vel),
        .actual_vel(actual_vel),
        .actual_pos(actual_pos), // 실제 위치

        .s00_axi_aclk(s00_axi_aclk),
        .s00_axi_aresetn(s00_axi_aresetn),
        .s00_axi_awaddr(s00_axi_awaddr),
        .s00_axi_awprot(s00_axi_awprot),
        .s00_axi_awvalid(s00_axi_awvalid),
        .s00_axi_awready(s00_axi_awready),
        .s00_axi_wdata(s00_axi_wdata),
        .s00_axi_wstrb(s00_axi_wstrb),
        .s00_axi_wvalid(s00_axi_wvalid),
        .s00_axi_wready(s00_axi_wready),
        .s00_axi_bresp(s00_axi_bresp),
        .s00_axi_bvalid(s00_axi_bvalid),
        .s00_axi_bready(s00_axi_bready),
        .s00_axi_araddr(s00_axi_araddr),
        .s00_axi_arprot(s00_axi_arprot),
        .s00_axi_arvalid(s00_axi_arvalid),
        .s00_axi_arready(s00_axi_arready),
        .s00_axi_rdata(s00_axi_rdata),
        .s00_axi_rresp(s00_axi_rresp),
        .s00_axi_rvalid(s00_axi_rvalid),
        .s00_axi_rready(s00_axi_rready)
    );

    // 모터 제어 모듈 인스턴스화
    (* dont_touch = "true" *)
    motor_top u_motor_top (
        .clk(clk),                     // 100 MHz 클럭
        .reset_n(reset_n),             // 리셋 신호
        .encoder_a(encoder_a),
        .encoder_b(encoder_b),
        .encoder_index(encoder_index),
        .Kp_axi(kp_init),              // AXI로부터 전달받은 Kp 값
        .Ki_axi(ki_init),              // AXI로부터 전달받은 Ki 값
        .desired_vel(desired_vel),     // AXI로부터 전달받은 목표 속도도
        .actual_vel(actual_vel),       // 실제 속도 출력
        .actual_position(actual_pos),  // 실제 위치 출력
        .dir1(dir1),                   // 방향 제어 1
        .dir2(dir2),                   // 방향 제어 2
        .pi_control_signal(internal_control_signal), // 디버깅: 제어 신호
        .pwm_out(pwm_out)              // PWM 출력
    );

    // LED 디버깅 출력 연결
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            led <= 4'b0000; // 리셋 시 LED 초기화
        end else begin
            led[0] <= (internal_control_signal != 0); // 내부 제어 신호가 0이 아니면 LED[0] ON
            led[1] <= dir1 && !dir2;                 // 방향 신호에 따라 LED[1] ON/OFF
            led[2] <= (desired_vel != 0) && (kp_init != 0); // 목표 위치와 Kp 초기값 확인
            led[3] <= pwm_out;                       // PWM 출력에 따라 LED[3] ON/OFF
        end
    end

endmodule
