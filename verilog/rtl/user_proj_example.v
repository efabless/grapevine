module RX_INTF #(
    parameter NOC_WID = 16
) (
    input                   clk,
    input                   rst,

    input [NOC_WID-1:0]     rx,
    input [7:0]             rx_bits,
    input                   rx_toggle,

    // NoC ports
    output wire             rx_req,
    output wire [1:0]       rx_d,
    input                   rx_ack
);

    localparam [2:0]
        IDLE = 3'b000,
        REQ_HI = 3'b001,
        DATA_HI = 3'b010,
        DATA_LO = 3'b011,
        WAIT_ACK = 3'b100;

    reg [2:0]           state;
    reg [NOC_WID-1:0]   rx_sr;
    reg [7:0]           bits_left;
    reg                 toggle_last;


    assign rx_req = (state != IDLE) ? 1 : 0;
    assign rx_d[0] = (state == DATA_HI && rx_sr[NOC_WID-1] == 0) ? 1 : 0;
    assign rx_d[1] = (state == DATA_HI && rx_sr[NOC_WID-1] == 1) ? 1 : 0;

    always @(posedge clk) begin
        if (rst) begin
            state <= IDLE;
            rx_sr <= 0;
            bits_left <= 0;
            toggle_last <= 0;
        end else begin
            case (state)
                IDLE: begin
                    if (rx_toggle != toggle_last) begin
                        state <= REQ_HI;
                        rx_sr <= rx;
                        bits_left <= rx_bits;
                        toggle_last <= rx_toggle;
                    end
                end
                REQ_HI: begin
                    if (rx_ack) begin
                        state <= DATA_HI;
                    end
                end
                DATA_HI: begin
                    if (~rx_ack) begin
                        state <= DATA_LO;
                    end
                end
                DATA_LO: begin
                    state <= WAIT_ACK;
                    rx_sr <= (rx_sr << 1);
                    bits_left <= bits_left - 1;
                end
                WAIT_ACK: begin
                    if (rx_ack) begin
                        if (bits_left == 0) begin
                            state <= IDLE;
                        end else begin
                            state <= DATA_HI;
                        end
                    end
                end
            endcase
        end
    end

endmodule


// module tb;

//     reg         clk, rst;
//     reg [15:0]  rx;
//     reg [7:0]   rx_bits;
//     reg         rx_toggle;

//     wire        rx_req;
//     wire [1:0]  rx_d;
//     wire        rx_ack;

//     RX_INTF i_RX_INTF (
//         .clk      (clk      ),
//         .rst      (rst      ),
//         .rx       (rx       ),
//         .rx_bits  (rx_bits  ),
//         .rx_toggle(rx_toggle),
//         .rx_req   (rx_req   ),
//         .rx_d     (rx_d     ),
//         .rx_ack   (rx_ack   )
//     );

//     always #5 clk = ~clk;

//     localparam [1:0]
//         IDLE = 2'b00,
//         ACK_HI = 2'b01,
//         ACK_LO = 2'b10;

//     reg [1:0] state;

//     wire rx_valid;

//     assign rx_valid = rx_d[0] | rx_d[1];
//     assign rx_ack = (state == ACK_HI) ? 1 : 0;

//     always @(posedge clk) begin
//         case (state)
//             IDLE: begin
//                 if (rx_req) begin
//                     state <= ACK_HI;
//                 end
//             end
//             ACK_HI: begin
//                 if (rx_valid) begin
//                     state <= ACK_LO;
//                 end else if (~rx_req) begin
//                     state <= IDLE;
//                 end
//             end
//             ACK_LO: begin
//                 if (~rx_valid) begin
//                     state <= ACK_HI;
//                 end
//             end
//         endcase
//     end

//     initial begin
//         $dumpfile("dump.vcd");
//         $dumpvars(0, tb);

//         clk = 0;
//         rst = 1;

//         state = IDLE;

//         rx = 0;
//         rx_bits = 0;
//         rx_toggle = 0;
//         #21;

//         rst = 0;
//         rx = 16'b1100_1010_0000_0000;
//         rx_bits = 8;
//         rx_toggle = ~rx_toggle;
//         #400;

//         #200;
//         $finish;
//     end
// endmodule
module TX_INTF #(
    parameter NOC_WID = 16
) (
    input                       clk,
    input                       rst,

    output reg [NOC_WID-1:0]    tx,

    // NoC ports
    input [2*NOC_WID-1:0]       tx_d,
    output wire                 tx_ack
);

    localparam [1:0]
        IDLE = 2'b00,
        DATA_LOAD = 2'b01,
        ACK_HI = 2'b10;

    reg [2:0]           state;

    wire [NOC_WID-1:0]  tx_valid_bitwise;
    wire                tx_valid;
    wire                tx_empty;

    genvar i;
    generate
        for (i = 0; i < NOC_WID; i = i+1) begin
            assign tx_valid_bitwise[i] = (tx_d[2*i] | tx_d[2*i+1]);
        end
    endgenerate

    assign tx_valid = &tx_valid_bitwise;
    assign tx_empty = ~(|tx_valid_bitwise);

    assign tx_ack = (state == ACK_HI) ? 1 : 0;

    integer j;
    always @(posedge clk) begin
        if (rst) begin
            state <= IDLE;
            tx <= 0;
        end else begin
            case (state)
                IDLE: begin
                    if (tx_valid) begin
                        state <= DATA_LOAD;
                        for (j = 0; j < NOC_WID; j = j+1) begin
                            tx[j] <= tx_d[2*j+1];
                        end
                    end
                end
                DATA_LOAD: begin
                    state <= ACK_HI;
                end
                ACK_HI: begin
                    if (tx_empty) begin
                        state <= IDLE;
                    end
                end

            endcase

        end
    end
endmodule


// module tb;

//     reg         clk, rst;
//     wire [3:0]  tx;
//     reg [7:0]   tx_d;
//     wire        tx_ack;

//     TX_INTF #(
//         .NOC_WID(4)
//     ) i_TX_INTF (
//         .clk(clk),
//         .rst(rst),
//         .tx(tx),
//         .tx_d(tx_d),
//         .tx_ack(tx_ack)
//     );

//     always #5 clk = ~clk;

//     initial begin
//         $dumpfile("dump.vcd");
//         $dumpvars(0, tb);

//         clk = 0;
//         rst = 1;

//         tx_d = 0;
//         #21;

//         rst = 0;
//         tx_d = 8'b10100101;
//         #400;
//         tx_d = 0;

//         #200;
//         $finish;

//     end


// endmodule
module WB_INTF #(
    parameter WB_WID = 32,
    parameter NOC_WID = 16
) (

    input                       en,             // mux enabled

    input                       wb_clk_i,
    input                       wb_rst_i,
    input                       wbs_stb_i,
    input                       wbs_cyc_i,
    input                       wbs_we_i,
    input [(WB_WID/8)-1:0]      wbs_sel_i,
    input [WB_WID-1:0]          wbs_dat_i,
    input [WB_WID-1:0]          wbs_adr_i,
    output reg                  wbs_ack_o,
    output reg [WB_WID-1:0]     wbs_dat_o,

    // To repeaters
    output reg [WB_WID-1:0]     repeat_dat,
    output reg [WB_WID-1:0]     repeat_adr,

    // To NoC wrappers
    output reg [NOC_WID-1:0]    noc_rx,
    output reg [7:0]            noc_rx_bits,
    output reg                  noc_rx_toggle,
    input [NOC_WID-1:0]         noc_tx,
    output reg                  noc_tx_toggle
);

    localparam DEFAULT_ADDR = 32'hFFFFFFFF;     // Out of bounds for all PEs

    localparam NOC_RX_ADDR = 32'hFFFF0000;
    localparam NOC_TX_ADDR = 32'hFFFF0004;

    always @(posedge wb_clk_i) begin
        if (wb_rst_i | ~en) begin
            wbs_dat_o <= 0;
            repeat_dat <= 0;
            repeat_adr <= DEFAULT_ADDR;
            noc_rx <= 0;
            noc_rx_bits <= 0;
            noc_rx_toggle <= 0;
            noc_tx_toggle <= 0;
        end else if (wbs_stb_i && wbs_cyc_i) begin
            repeat_dat <= 0;
            repeat_adr <= DEFAULT_ADDR;
            noc_rx <= 0;
            wbs_dat_o <= 0;

            if (wbs_we_i && ~wbs_ack_o) begin                   // Write: stolen from Charles
                repeat_dat <= wbs_dat_i;
                repeat_adr <= wbs_adr_i;

                if (wbs_adr_i == NOC_RX_ADDR) begin
                    noc_rx <= wbs_dat_i[NOC_WID-1:0];
                    noc_rx_bits <= wbs_dat_i[NOC_WID+7:NOC_WID];
                    noc_rx_toggle <= ~noc_rx_toggle;
                end

            end else if (~wbs_we_i && ~wbs_ack_o) begin         // Read: stolen from Charles
                if (wbs_adr_i == NOC_TX_ADDR) begin
                    wbs_dat_o <= noc_tx;
                    noc_tx_toggle <= ~noc_tx_toggle;
                end
            end
        end
    end

    always @(posedge wb_clk_i) begin
        if (wb_rst_i | ~en) begin
            wbs_ack_o <= 0;
        end else begin
            wbs_ack_o <= (wbs_stb_i && wbs_cyc_i);              // We can process immediately: stolen from Charles
        end
    end
endmodule
module bus_repeater #(
    parameter WB_WID = 32,
    parameter NOC_WID = 16,
    parameter REGIONAL_ADDR_WID = 9,
    parameter FANOUT = 32,
    parameter ADDR_LO = 32'h0,
    parameter ADDR_HI = 32'b001000000000        // non-inclusive
) (

    input                               clk,
    input                               rst,
    input [WB_WID-1:0]                  in_dat,
    input [WB_WID-1:0]                  in_adr,

    output reg [NOC_WID-1:0]            out_dat,
    output reg [REGIONAL_ADDR_WID-1:0]  out_adr
);

    always @(posedge clk) begin
        if (rst) begin
            out_dat <= 0;
            out_adr <= 1;       // Note: address 0 actually maps to a PE
        end else begin
            if (in_adr >= ADDR_LO && in_adr < ADDR_HI) begin
                out_dat <= in_dat[NOC_WID-1:0];
                out_adr <= in_adr-ADDR_LO;
            end else begin
                out_dat <= 0;
                out_adr <= 1;
            end
        end
    end

endmodule


// module tb;

//     reg clk, rst;
//     reg [31:0] in_dat, in_adr;
//     wire [15:0] out_dat;
//     wire [8:0] out_adr;

//     bus_repeater i_bus_repeater (
//         .clk    (clk    ),
//         .rst    (rst    ),
//         .in_dat (in_dat ),
//         .in_adr (in_adr ),
//         .out_dat(out_dat),
//         .out_adr(out_adr)
//     );

//     always #5 clk = ~clk;

//     initial begin
//         $dumpfile("dump.vcd");
//         $dumpvars(0, tb);

//         clk = 0;
//         rst = 1;

//         in_dat = 0;
//         in_adr = 0;

//         #21;
//         rst = 0;
//         in_dat = 1;
//         in_adr = 0;

//         #10;
//         in_dat = 2;
//         in_adr = 32'b001000000000;

//         #200;
//         $finish;
//     end

// endmodule
module leaf #(
    parameter NOC_WID = 16,
    parameter INIT_MSG = 32'b01010101010101010101010101010101
)(
    input       clk,
    input       rst,

    input                       rx_pr,
    input [1:0]                 rx_pd,
    output wire                 rx_pa,

    output wire [2*NOC_WID-1:0] tx_pd,
    input                       tx_pa
);


    localparam [1:0]
        RX_IDLE = 2'b00,
        RX_ACK_HI = 2'b01,
        RX_ACK_LO = 2'b10;

    reg [1:0] rx_state;

    wire rx_valid;

    assign rx_pa = (rx_state == RX_ACK_HI) ? 1 : 0;
    assign rx_valid = rx_pd[0] || rx_pd[1];

    always @(posedge clk) begin
        if (rst) begin
            rx_state <= RX_IDLE;
        end else begin
            case (rx_state)
                RX_IDLE: begin
                    if (rx_pr) begin
                        rx_state <= RX_ACK_HI;
                    end
                end
                RX_ACK_HI: begin
                    if (~rx_pr) begin
                        rx_state <= RX_IDLE;
                    end else if (rx_valid) begin
                        rx_state <= RX_ACK_LO;
                    end
                end
                RX_ACK_LO: begin
                    if (~rx_valid) begin
                        rx_state <= RX_ACK_HI;
                    end
                end
            endcase
        end
    end

    localparam [1:0]
        TX_IDLE = 2'b00,
        TX_SEND = 2'b01,
        TX_NOSEND = 2'b10;

    reg [1:0] tx_state;

    assign tx_pd = (tx_state == TX_SEND) ? INIT_MSG : 0;

    always @(posedge clk) begin
        if (rst) begin
            tx_state <= TX_IDLE;
        end else begin
            case (tx_state)
                TX_IDLE: begin
                    if (rx_state == RX_ACK_HI) begin
                        tx_state <= TX_SEND;
                    end
                end
                TX_SEND: begin
                    if (tx_pa) begin
                        tx_state <= TX_NOSEND;
                    end
                end
                TX_NOSEND: begin
                    if (rx_state == RX_IDLE) begin
                        tx_state <= TX_IDLE;
                    end
                end
            endcase
        end
    end


endmodule
module Top (
    input               wb_clk_i,
    input               wb_rst_i,
    input               wbs_stb_i,
    input               wbs_cyc_i,
    input               wbs_we_i,
    input [3:0]         wbs_sel_i,
    input [31:0]        wbs_dat_i,
    input [31:0]        wbs_adr_i,
    output wire         wbs_ack_o,
    output wire [31:0]  wbs_dat_o
);

    wire [31:0]     br_dat, br_adr;
    wire [15:0]     rx, tx;
    wire [7:0]      rx_bits;
    wire            rx_toggle, tx_toggle;
    WB_INTF #(
        .WB_WID(32),
        .NOC_WID(16)
    ) i_WB_INTF (
        .en           (1'b1),
        .wb_clk_i     (wb_clk_i),
        .wb_rst_i     (wb_rst_i),
        .wbs_stb_i    (wbs_stb_i),
        .wbs_cyc_i    (wbs_cyc_i),
        .wbs_we_i     (wbs_we_i),
        .wbs_sel_i    (wbs_sel_i),
        .wbs_dat_i    (wbs_dat_i),
        .wbs_adr_i    (wbs_adr_i),
        .wbs_ack_o    (wbs_ack_o),
        .wbs_dat_o    (wbs_dat_o),
        .repeat_dat   (br_dat),
        .repeat_adr   (br_adr),
        .noc_rx       (rx),
        .noc_rx_bits  (rx_bits),
        .noc_rx_toggle(rx_toggle),
        .noc_tx       (tx),
        .noc_tx_toggle(tx_toggle)
    );


    wire            top_rx_req, top_rx_ack;
    wire [1:0]      top_rx_d;
    RX_INTF #(
        .NOC_WID(16)
    ) i_RX_INTF (
        .clk      (wb_clk_i),
        .rst      (wb_rst_i),
        .rx       (rx),
        .rx_bits  (rx_bits),
        .rx_toggle(rx_toggle),
        .rx_req   (top_rx_req),
        .rx_d     (top_rx_d),
        .rx_ack   (top_rx_ack)
    );

    wire [31:0]     top_tx_d;
    wire            top_tx_ack;
    TX_INTF #(
        .NOC_WID(16)
    ) i_TX_INTF (
        .clk   (wb_clk_i),
        .rst   (wb_rst_i),
        .tx    (tx),
        .tx_d  (top_tx_d),
        .tx_ack(top_tx_ack)
    );

    wire [15:0]  br0_dat;
    wire [10:0]  br0_adr;
    bus_repeater #(
        .WB_WID           (32),
        .NOC_WID          (16),
        .REGIONAL_ADDR_WID(11),
        .FANOUT           (32),
        .ADDR_LO          (32'h30000000),
        .ADDR_HI          (32'h30000048)
    ) br0 (
        .clk(wb_clk_i),     .rst(wb_rst_i),
        .in_dat(br_dat),    .in_adr(br_adr),
        .out_dat(br0_dat),  .out_adr(br0_adr)
    );

    wire [15:0]  br1_dat;
    wire [10:0]  br1_adr;
    bus_repeater #(
        .WB_WID           (32),
        .NOC_WID          (16),
        .REGIONAL_ADDR_WID(11),
        .FANOUT           (32),
        .ADDR_LO          (32'h30000048),
        .ADDR_HI          (32'h3000006c)
    ) br1 (
        .clk(wb_clk_i),     .rst(wb_rst_i),
        .in_dat(br_dat),    .in_adr(br_adr),
        .out_dat(br1_dat),  .out_adr(br1_adr)
    );

    wire [15:0]  br2_dat;
    wire [10:0]  br2_adr;
    bus_repeater #(
        .WB_WID           (32),
        .NOC_WID          (16),
        .REGIONAL_ADDR_WID(11),
        .FANOUT           (32),
        .ADDR_LO          (32'h3000006c),
        .ADDR_HI          (32'h3000009c)
    ) br2 (
        .clk(wb_clk_i),     .rst(wb_rst_i),
        .in_dat(br_dat),    .in_adr(br_adr),
        .out_dat(br2_dat),  .out_adr(br2_adr)
    );

    wire [15:0]  br3_dat;
    wire [10:0]  br3_adr;
    bus_repeater #(
        .WB_WID           (32),
        .NOC_WID          (16),
        .REGIONAL_ADDR_WID(11),
        .FANOUT           (32),
        .ADDR_LO          (32'h3000009c),
        .ADDR_HI          (32'h300000c0)
    ) br3 (
        .clk(wb_clk_i),     .rst(wb_rst_i),
        .in_dat(br_dat),    .in_adr(br_adr),
        .out_dat(br3_dat),  .out_adr(br3_adr)
    );

    wire [15:0]  br4_dat;
    wire [10:0]  br4_adr;
    bus_repeater #(
        .WB_WID           (32),
        .NOC_WID          (16),
        .REGIONAL_ADDR_WID(11),
        .FANOUT           (32),
        .ADDR_LO          (32'h300000c0),
        .ADDR_HI          (32'h300000fc)
    ) br4 (
        .clk(wb_clk_i),     .rst(wb_rst_i),
        .in_dat(br_dat),    .in_adr(br_adr),
        .out_dat(br4_dat),  .out_adr(br4_adr)
    );

    wire [15:0]  br5_dat;
    wire [10:0]  br5_adr;
    bus_repeater #(
        .WB_WID           (32),
        .NOC_WID          (16),
        .REGIONAL_ADDR_WID(11),
        .FANOUT           (32),
        .ADDR_LO          (32'h300000fc),
        .ADDR_HI          (32'h30000120)
    ) br5 (
        .clk(wb_clk_i),     .rst(wb_rst_i),
        .in_dat(br_dat),    .in_adr(br_adr),
        .out_dat(br5_dat),  .out_adr(br5_adr)
    );

    wire [15:0]  br6_dat;
    wire [10:0]  br6_adr;
    bus_repeater #(
        .WB_WID           (32),
        .NOC_WID          (16),
        .REGIONAL_ADDR_WID(11),
        .FANOUT           (32),
        .ADDR_LO          (32'h30000120),
        .ADDR_HI          (32'h30000150)
    ) br6 (
        .clk(wb_clk_i),     .rst(wb_rst_i),
        .in_dat(br_dat),    .in_adr(br_adr),
        .out_dat(br6_dat),  .out_adr(br6_adr)
    );

    wire [15:0]  br7_dat;
    wire [10:0]  br7_adr;
    bus_repeater #(
        .WB_WID           (32),
        .NOC_WID          (16),
        .REGIONAL_ADDR_WID(11),
        .FANOUT           (32),
        .ADDR_LO          (32'h30000150),
        .ADDR_HI          (32'h30000174)
    ) br7 (
        .clk(wb_clk_i),     .rst(wb_rst_i),
        .in_dat(br_dat),    .in_adr(br_adr),
        .out_dat(br7_dat),  .out_adr(br7_adr)
    );


    wire            r_rxr, r_rxa, r_txa;
    wire [1:0]      r_rxd;
    wire [31:0]     r_txd;
    wire            rl_rxr, rl_rxa, rl_txa;
    wire [1:0]      rl_rxd;
    wire [31:0]     rl_txd;
    wire            rll_rxr, rll_rxa, rll_txa;
    wire [1:0]      rll_rxd;
    wire [31:0]     rll_txd;
    wire            rlll_rxr, rlll_rxa, rlll_txa;
    wire [1:0]      rlll_rxd;
    wire [31:0]     rlll_txd;
    wire            rllll_rxr, rllll_rxa, rllll_txa;
    wire [1:0]      rllll_rxd;
    wire [31:0]     rllll_txd;
    wire            rlllll_rxr, rlllll_rxa, rlllll_txa;
    wire [1:0]      rlllll_rxd;
    wire [31:0]     rlllll_txd;
    wire            rllllr_rxr, rllllr_rxa, rllllr_txa;
    wire [1:0]      rllllr_rxd;
    wire [31:0]     rllllr_txd;
    wire            rlllr_rxr, rlllr_rxa, rlllr_txa;
    wire [1:0]      rlllr_rxd;
    wire [31:0]     rlllr_txd;
    wire            rlllrl_rxr, rlllrl_rxa, rlllrl_txa;
    wire [1:0]      rlllrl_rxd;
    wire [31:0]     rlllrl_txd;
    wire            rlllrr_rxr, rlllrr_rxa, rlllrr_txa;
    wire [1:0]      rlllrr_rxd;
    wire [31:0]     rlllrr_txd;
    wire            rllr_rxr, rllr_rxa, rllr_txa;
    wire [1:0]      rllr_rxd;
    wire [31:0]     rllr_txd;
    wire            rllrl_rxr, rllrl_rxa, rllrl_txa;
    wire [1:0]      rllrl_rxd;
    wire [31:0]     rllrl_txd;
    wire            rllrll_rxr, rllrll_rxa, rllrll_txa;
    wire [1:0]      rllrll_rxd;
    wire [31:0]     rllrll_txd;
    wire            rllrlr_rxr, rllrlr_rxa, rllrlr_txa;
    wire [1:0]      rllrlr_rxd;
    wire [31:0]     rllrlr_txd;
    wire            rllrr_rxr, rllrr_rxa, rllrr_txa;
    wire [1:0]      rllrr_rxd;
    wire [31:0]     rllrr_txd;
    wire            rllrrl_rxr, rllrrl_rxa, rllrrl_txa;
    wire [1:0]      rllrrl_rxd;
    wire [31:0]     rllrrl_txd;
    wire            rllrrr_rxr, rllrrr_rxa, rllrrr_txa;
    wire [1:0]      rllrrr_rxd;
    wire [31:0]     rllrrr_txd;
    wire            rlr_rxr, rlr_rxa, rlr_txa;
    wire [1:0]      rlr_rxd;
    wire [31:0]     rlr_txd;
    wire            rlrl_rxr, rlrl_rxa, rlrl_txa;
    wire [1:0]      rlrl_rxd;
    wire [31:0]     rlrl_txd;
    wire            rlrll_rxr, rlrll_rxa, rlrll_txa;
    wire [1:0]      rlrll_rxd;
    wire [31:0]     rlrll_txd;
    wire            rlrlll_rxr, rlrlll_rxa, rlrlll_txa;
    wire [1:0]      rlrlll_rxd;
    wire [31:0]     rlrlll_txd;
    wire            rlrllr_rxr, rlrllr_rxa, rlrllr_txa;
    wire [1:0]      rlrllr_rxd;
    wire [31:0]     rlrllr_txd;
    wire            rlrlr_rxr, rlrlr_rxa, rlrlr_txa;
    wire [1:0]      rlrlr_rxd;
    wire [31:0]     rlrlr_txd;
    wire            rlrlrl_rxr, rlrlrl_rxa, rlrlrl_txa;
    wire [1:0]      rlrlrl_rxd;
    wire [31:0]     rlrlrl_txd;
    wire            rlrlrr_rxr, rlrlrr_rxa, rlrlrr_txa;
    wire [1:0]      rlrlrr_rxd;
    wire [31:0]     rlrlrr_txd;
    wire            rlrr_rxr, rlrr_rxa, rlrr_txa;
    wire [1:0]      rlrr_rxd;
    wire [31:0]     rlrr_txd;
    wire            rlrrl_rxr, rlrrl_rxa, rlrrl_txa;
    wire [1:0]      rlrrl_rxd;
    wire [31:0]     rlrrl_txd;
    wire            rlrrll_rxr, rlrrll_rxa, rlrrll_txa;
    wire [1:0]      rlrrll_rxd;
    wire [31:0]     rlrrll_txd;
    wire            rlrrlr_rxr, rlrrlr_rxa, rlrrlr_txa;
    wire [1:0]      rlrrlr_rxd;
    wire [31:0]     rlrrlr_txd;
    wire            rlrrr_rxr, rlrrr_rxa, rlrrr_txa;
    wire [1:0]      rlrrr_rxd;
    wire [31:0]     rlrrr_txd;
    wire            rlrrrl_rxr, rlrrrl_rxa, rlrrrl_txa;
    wire [1:0]      rlrrrl_rxd;
    wire [31:0]     rlrrrl_txd;
    wire            rlrrrr_rxr, rlrrrr_rxa, rlrrrr_txa;
    wire [1:0]      rlrrrr_rxd;
    wire [31:0]     rlrrrr_txd;
    wire            rr_rxr, rr_rxa, rr_txa;
    wire [1:0]      rr_rxd;
    wire [31:0]     rr_txd;
    wire            rrl_rxr, rrl_rxa, rrl_txa;
    wire [1:0]      rrl_rxd;
    wire [31:0]     rrl_txd;
    wire            rrll_rxr, rrll_rxa, rrll_txa;
    wire [1:0]      rrll_rxd;
    wire [31:0]     rrll_txd;
    wire            rrlll_rxr, rrlll_rxa, rrlll_txa;
    wire [1:0]      rrlll_rxd;
    wire [31:0]     rrlll_txd;
    wire            rrllll_rxr, rrllll_rxa, rrllll_txa;
    wire [1:0]      rrllll_rxd;
    wire [31:0]     rrllll_txd;
    wire            rrlllr_rxr, rrlllr_rxa, rrlllr_txa;
    wire [1:0]      rrlllr_rxd;
    wire [31:0]     rrlllr_txd;
    wire            rrllr_rxr, rrllr_rxa, rrllr_txa;
    wire [1:0]      rrllr_rxd;
    wire [31:0]     rrllr_txd;
    wire            rrllrl_rxr, rrllrl_rxa, rrllrl_txa;
    wire [1:0]      rrllrl_rxd;
    wire [31:0]     rrllrl_txd;
    wire            rrllrr_rxr, rrllrr_rxa, rrllrr_txa;
    wire [1:0]      rrllrr_rxd;
    wire [31:0]     rrllrr_txd;
    wire            rrlr_rxr, rrlr_rxa, rrlr_txa;
    wire [1:0]      rrlr_rxd;
    wire [31:0]     rrlr_txd;
    wire            rrlrl_rxr, rrlrl_rxa, rrlrl_txa;
    wire [1:0]      rrlrl_rxd;
    wire [31:0]     rrlrl_txd;
    wire            rrlrll_rxr, rrlrll_rxa, rrlrll_txa;
    wire [1:0]      rrlrll_rxd;
    wire [31:0]     rrlrll_txd;
    wire            rrlrlr_rxr, rrlrlr_rxa, rrlrlr_txa;
    wire [1:0]      rrlrlr_rxd;
    wire [31:0]     rrlrlr_txd;
    wire            rrlrr_rxr, rrlrr_rxa, rrlrr_txa;
    wire [1:0]      rrlrr_rxd;
    wire [31:0]     rrlrr_txd;
    wire            rrlrrl_rxr, rrlrrl_rxa, rrlrrl_txa;
    wire [1:0]      rrlrrl_rxd;
    wire [31:0]     rrlrrl_txd;
    wire            rrlrrr_rxr, rrlrrr_rxa, rrlrrr_txa;
    wire [1:0]      rrlrrr_rxd;
    wire [31:0]     rrlrrr_txd;
    wire            rrr_rxr, rrr_rxa, rrr_txa;
    wire [1:0]      rrr_rxd;
    wire [31:0]     rrr_txd;
    wire            rrrl_rxr, rrrl_rxa, rrrl_txa;
    wire [1:0]      rrrl_rxd;
    wire [31:0]     rrrl_txd;
    wire            rrrll_rxr, rrrll_rxa, rrrll_txa;
    wire [1:0]      rrrll_rxd;
    wire [31:0]     rrrll_txd;
    wire            rrrlll_rxr, rrrlll_rxa, rrrlll_txa;
    wire [1:0]      rrrlll_rxd;
    wire [31:0]     rrrlll_txd;
    wire            rrrllr_rxr, rrrllr_rxa, rrrllr_txa;
    wire [1:0]      rrrllr_rxd;
    wire [31:0]     rrrllr_txd;
    wire            rrrlr_rxr, rrrlr_rxa, rrrlr_txa;
    wire [1:0]      rrrlr_rxd;
    wire [31:0]     rrrlr_txd;
    wire            rrrlrl_rxr, rrrlrl_rxa, rrrlrl_txa;
    wire [1:0]      rrrlrl_rxd;
    wire [31:0]     rrrlrl_txd;
    wire            rrrlrr_rxr, rrrlrr_rxa, rrrlrr_txa;
    wire [1:0]      rrrlrr_rxd;
    wire [31:0]     rrrlrr_txd;
    wire            rrrr_rxr, rrrr_rxa, rrrr_txa;
    wire [1:0]      rrrr_rxd;
    wire [31:0]     rrrr_txd;
    wire            rrrrl_rxr, rrrrl_rxa, rrrrl_txa;
    wire [1:0]      rrrrl_rxd;
    wire [31:0]     rrrrl_txd;
    wire            rrrrll_rxr, rrrrll_rxa, rrrrll_txa;
    wire [1:0]      rrrrll_rxd;
    wire [31:0]     rrrrll_txd;
    wire            rrrrlr_rxr, rrrrlr_rxa, rrrrlr_txa;
    wire [1:0]      rrrrlr_rxd;
    wire [31:0]     rrrrlr_txd;
    wire            rrrrr_rxr, rrrrr_rxa, rrrrr_txa;
    wire [1:0]      rrrrr_rxd;
    wire [31:0]     rrrrr_txd;
    wire            rrrrrl_rxr, rrrrrl_rxa, rrrrrl_txa;
    wire [1:0]      rrrrrl_rxd;
    wire [31:0]     rrrrrl_txd;
    wire            rrrrrr_rxr, rrrrrr_rxa, rrrrrr_txa;
    wire [1:0]      rrrrrr_rxd;
    wire [31:0]     rrrrrr_txd;


    assign r_rxr = top_rx_req;
    assign r_rxd = top_rx_d;
    assign top_rx_ack = r_rxa;
    assign top_tx_d = r_txd;
    assign r_txa = top_tx_ack;
    PE_right r (
        .clk(wb_clk_i), .rst(wb_rst_i),
        .cfg_dat(br0_dat), .cfg_adr(br0_adr), .slv_addr(11'h0),
        .rx_pr(r_rxr), .rx_pd(r_rxd), .rx_pa(r_rxa),
        .rx_c0r(rl_rxr), .rx_c0d(rl_rxd), .rx_c0a(rl_rxa),
        .rx_c1r(rr_rxr), .rx_c1d(rr_rxd), .rx_c1a(rr_rxa),
        .tx_c0d(rl_txd), .tx_c0a(rl_txa),
        .tx_c1d(rr_txd), .tx_c1a(rr_txa),
        .tx_pd(r_txd), .tx_pa(r_txa)
    );
    PE_down rl (
        .clk(wb_clk_i), .rst(wb_rst_i),
        .cfg_dat(br0_dat), .cfg_adr(br0_adr), .slv_addr(11'hc),
        .rx_pr(rl_rxr), .rx_pd(rl_rxd), .rx_pa(rl_rxa),
        .rx_c0r(rll_rxr), .rx_c0d(rll_rxd), .rx_c0a(rll_rxa),
        .rx_c1r(rlr_rxr), .rx_c1d(rlr_rxd), .rx_c1a(rlr_rxa),
        .tx_c0d(rll_txd), .tx_c0a(rll_txa),
        .tx_c1d(rlr_txd), .tx_c1a(rlr_txa),
        .tx_pd(rl_txd), .tx_pa(rl_txa)
    );
    PE_right rll (
        .clk(wb_clk_i), .rst(wb_rst_i),
        .cfg_dat(br0_dat), .cfg_adr(br0_adr), .slv_addr(11'h18),
        .rx_pr(rll_rxr), .rx_pd(rll_rxd), .rx_pa(rll_rxa),
        .rx_c0r(rlll_rxr), .rx_c0d(rlll_rxd), .rx_c0a(rlll_rxa),
        .rx_c1r(rllr_rxr), .rx_c1d(rllr_rxd), .rx_c1a(rllr_rxa),
        .tx_c0d(rlll_txd), .tx_c0a(rlll_txa),
        .tx_c1d(rllr_txd), .tx_c1a(rllr_txa),
        .tx_pd(rll_txd), .tx_pa(rll_txa)
    );
    PE_down rlll (
        .clk(wb_clk_i), .rst(wb_rst_i),
        .cfg_dat(br0_dat), .cfg_adr(br0_adr), .slv_addr(11'h24),
        .rx_pr(rlll_rxr), .rx_pd(rlll_rxd), .rx_pa(rlll_rxa),
        .rx_c0r(rllll_rxr), .rx_c0d(rllll_rxd), .rx_c0a(rllll_rxa),
        .rx_c1r(rlllr_rxr), .rx_c1d(rlllr_rxd), .rx_c1a(rlllr_rxa),
        .tx_c0d(rllll_txd), .tx_c0a(rllll_txa),
        .tx_c1d(rlllr_txd), .tx_c1a(rlllr_txa),
        .tx_pd(rlll_txd), .tx_pa(rlll_txa)
    );
    PE_right rllll (
        .clk(wb_clk_i), .rst(wb_rst_i),
        .cfg_dat(br0_dat), .cfg_adr(br0_adr), .slv_addr(11'h30),
        .rx_pr(rllll_rxr), .rx_pd(rllll_rxd), .rx_pa(rllll_rxa),
        .rx_c0r(rlllll_rxr), .rx_c0d(rlllll_rxd), .rx_c0a(rlllll_rxa),
        .rx_c1r(rllllr_rxr), .rx_c1d(rllllr_rxd), .rx_c1a(rllllr_rxa),
        .tx_c0d(rlllll_txd), .tx_c0a(rlllll_txa),
        .tx_c1d(rllllr_txd), .tx_c1a(rllllr_txa),
        .tx_pd(rllll_txd), .tx_pa(rllll_txa)
    );
    leaf #(
        .NOC_WID(16)
    ) leaf_rlllll (
        .clk  (wb_clk_i),
        .rst  (wb_rst_i),
        .rx_pr(rlllll_rxr),
        .rx_pd(rlllll_rxd),
        .rx_pa(rlllll_rxa),
        .tx_pd(rlllll_txd),
        .tx_pa(rlllll_txa)
    );
    leaf #(
        .NOC_WID(16)
    ) leaf_rllllr (
        .clk  (wb_clk_i),
        .rst  (wb_rst_i),
        .rx_pr(rllllr_rxr),
        .rx_pd(rllllr_rxd),
        .rx_pa(rllllr_rxa),
        .tx_pd(rllllr_txd),
        .tx_pa(rllllr_txa)
    );
    PE_left rlllr (
        .clk(wb_clk_i), .rst(wb_rst_i),
        .cfg_dat(br0_dat), .cfg_adr(br0_adr), .slv_addr(11'h3c),
        .rx_pr(rlllr_rxr), .rx_pd(rlllr_rxd), .rx_pa(rlllr_rxa),
        .rx_c0r(rlllrl_rxr), .rx_c0d(rlllrl_rxd), .rx_c0a(rlllrl_rxa),
        .rx_c1r(rlllrr_rxr), .rx_c1d(rlllrr_rxd), .rx_c1a(rlllrr_rxa),
        .tx_c0d(rlllrl_txd), .tx_c0a(rlllrl_txa),
        .tx_c1d(rlllrr_txd), .tx_c1a(rlllrr_txa),
        .tx_pd(rlllr_txd), .tx_pa(rlllr_txa)
    );
    leaf #(
        .NOC_WID(16)
    ) leaf_rlllrl (
        .clk  (wb_clk_i),
        .rst  (wb_rst_i),
        .rx_pr(rlllrl_rxr),
        .rx_pd(rlllrl_rxd),
        .rx_pa(rlllrl_rxa),
        .tx_pd(rlllrl_txd),
        .tx_pa(rlllrl_txa)
    );
    leaf #(
        .NOC_WID(16)
    ) leaf_rlllrr (
        .clk  (wb_clk_i),
        .rst  (wb_rst_i),
        .rx_pr(rlllrr_rxr),
        .rx_pd(rlllrr_rxd),
        .rx_pa(rlllrr_rxa),
        .tx_pd(rlllrr_txd),
        .tx_pa(rlllrr_txa)
    );
    PE_up rllr (
        .clk(wb_clk_i), .rst(wb_rst_i),
        .cfg_dat(br1_dat), .cfg_adr(br1_adr), .slv_addr(11'h0),
        .rx_pr(rllr_rxr), .rx_pd(rllr_rxd), .rx_pa(rllr_rxa),
        .rx_c0r(rllrl_rxr), .rx_c0d(rllrl_rxd), .rx_c0a(rllrl_rxa),
        .rx_c1r(rllrr_rxr), .rx_c1d(rllrr_rxd), .rx_c1a(rllrr_rxa),
        .tx_c0d(rllrl_txd), .tx_c0a(rllrl_txa),
        .tx_c1d(rllrr_txd), .tx_c1a(rllrr_txa),
        .tx_pd(rllr_txd), .tx_pa(rllr_txa)
    );
    PE_right rllrl (
        .clk(wb_clk_i), .rst(wb_rst_i),
        .cfg_dat(br1_dat), .cfg_adr(br1_adr), .slv_addr(11'hc),
        .rx_pr(rllrl_rxr), .rx_pd(rllrl_rxd), .rx_pa(rllrl_rxa),
        .rx_c0r(rllrll_rxr), .rx_c0d(rllrll_rxd), .rx_c0a(rllrll_rxa),
        .rx_c1r(rllrlr_rxr), .rx_c1d(rllrlr_rxd), .rx_c1a(rllrlr_rxa),
        .tx_c0d(rllrll_txd), .tx_c0a(rllrll_txa),
        .tx_c1d(rllrlr_txd), .tx_c1a(rllrlr_txa),
        .tx_pd(rllrl_txd), .tx_pa(rllrl_txa)
    );
    leaf #(
        .NOC_WID(16)
    ) leaf_rllrll (
        .clk  (wb_clk_i),
        .rst  (wb_rst_i),
        .rx_pr(rllrll_rxr),
        .rx_pd(rllrll_rxd),
        .rx_pa(rllrll_rxa),
        .tx_pd(rllrll_txd),
        .tx_pa(rllrll_txa)
    );
    leaf #(
        .NOC_WID(16)
    ) leaf_rllrlr (
        .clk  (wb_clk_i),
        .rst  (wb_rst_i),
        .rx_pr(rllrlr_rxr),
        .rx_pd(rllrlr_rxd),
        .rx_pa(rllrlr_rxa),
        .tx_pd(rllrlr_txd),
        .tx_pa(rllrlr_txa)
    );
    PE_left rllrr (
        .clk(wb_clk_i), .rst(wb_rst_i),
        .cfg_dat(br1_dat), .cfg_adr(br1_adr), .slv_addr(11'h18),
        .rx_pr(rllrr_rxr), .rx_pd(rllrr_rxd), .rx_pa(rllrr_rxa),
        .rx_c0r(rllrrl_rxr), .rx_c0d(rllrrl_rxd), .rx_c0a(rllrrl_rxa),
        .rx_c1r(rllrrr_rxr), .rx_c1d(rllrrr_rxd), .rx_c1a(rllrrr_rxa),
        .tx_c0d(rllrrl_txd), .tx_c0a(rllrrl_txa),
        .tx_c1d(rllrrr_txd), .tx_c1a(rllrrr_txa),
        .tx_pd(rllrr_txd), .tx_pa(rllrr_txa)
    );
    leaf #(
        .NOC_WID(16)
    ) leaf_rllrrl (
        .clk  (wb_clk_i),
        .rst  (wb_rst_i),
        .rx_pr(rllrrl_rxr),
        .rx_pd(rllrrl_rxd),
        .rx_pa(rllrrl_rxa),
        .tx_pd(rllrrl_txd),
        .tx_pa(rllrrl_txa)
    );
    leaf #(
        .NOC_WID(16)
    ) leaf_rllrrr (
        .clk  (wb_clk_i),
        .rst  (wb_rst_i),
        .rx_pr(rllrrr_rxr),
        .rx_pd(rllrrr_rxd),
        .rx_pa(rllrrr_rxa),
        .tx_pd(rllrrr_txd),
        .tx_pa(rllrrr_txa)
    );
    PE_left rlr (
        .clk(wb_clk_i), .rst(wb_rst_i),
        .cfg_dat(br2_dat), .cfg_adr(br2_adr), .slv_addr(11'h0),
        .rx_pr(rlr_rxr), .rx_pd(rlr_rxd), .rx_pa(rlr_rxa),
        .rx_c0r(rlrl_rxr), .rx_c0d(rlrl_rxd), .rx_c0a(rlrl_rxa),
        .rx_c1r(rlrr_rxr), .rx_c1d(rlrr_rxd), .rx_c1a(rlrr_rxa),
        .tx_c0d(rlrl_txd), .tx_c0a(rlrl_txa),
        .tx_c1d(rlrr_txd), .tx_c1a(rlrr_txa),
        .tx_pd(rlr_txd), .tx_pa(rlr_txa)
    );
    PE_down rlrl (
        .clk(wb_clk_i), .rst(wb_rst_i),
        .cfg_dat(br2_dat), .cfg_adr(br2_adr), .slv_addr(11'hc),
        .rx_pr(rlrl_rxr), .rx_pd(rlrl_rxd), .rx_pa(rlrl_rxa),
        .rx_c0r(rlrll_rxr), .rx_c0d(rlrll_rxd), .rx_c0a(rlrll_rxa),
        .rx_c1r(rlrlr_rxr), .rx_c1d(rlrlr_rxd), .rx_c1a(rlrlr_rxa),
        .tx_c0d(rlrll_txd), .tx_c0a(rlrll_txa),
        .tx_c1d(rlrlr_txd), .tx_c1a(rlrlr_txa),
        .tx_pd(rlrl_txd), .tx_pa(rlrl_txa)
    );
    PE_right rlrll (
        .clk(wb_clk_i), .rst(wb_rst_i),
        .cfg_dat(br2_dat), .cfg_adr(br2_adr), .slv_addr(11'h18),
        .rx_pr(rlrll_rxr), .rx_pd(rlrll_rxd), .rx_pa(rlrll_rxa),
        .rx_c0r(rlrlll_rxr), .rx_c0d(rlrlll_rxd), .rx_c0a(rlrlll_rxa),
        .rx_c1r(rlrllr_rxr), .rx_c1d(rlrllr_rxd), .rx_c1a(rlrllr_rxa),
        .tx_c0d(rlrlll_txd), .tx_c0a(rlrlll_txa),
        .tx_c1d(rlrllr_txd), .tx_c1a(rlrllr_txa),
        .tx_pd(rlrll_txd), .tx_pa(rlrll_txa)
    );
    leaf #(
        .NOC_WID(16)
    ) leaf_rlrlll (
        .clk  (wb_clk_i),
        .rst  (wb_rst_i),
        .rx_pr(rlrlll_rxr),
        .rx_pd(rlrlll_rxd),
        .rx_pa(rlrlll_rxa),
        .tx_pd(rlrlll_txd),
        .tx_pa(rlrlll_txa)
    );
    leaf #(
        .NOC_WID(16)
    ) leaf_rlrllr (
        .clk  (wb_clk_i),
        .rst  (wb_rst_i),
        .rx_pr(rlrllr_rxr),
        .rx_pd(rlrllr_rxd),
        .rx_pa(rlrllr_rxa),
        .tx_pd(rlrllr_txd),
        .tx_pa(rlrllr_txa)
    );
    PE_left rlrlr (
        .clk(wb_clk_i), .rst(wb_rst_i),
        .cfg_dat(br2_dat), .cfg_adr(br2_adr), .slv_addr(11'h24),
        .rx_pr(rlrlr_rxr), .rx_pd(rlrlr_rxd), .rx_pa(rlrlr_rxa),
        .rx_c0r(rlrlrl_rxr), .rx_c0d(rlrlrl_rxd), .rx_c0a(rlrlrl_rxa),
        .rx_c1r(rlrlrr_rxr), .rx_c1d(rlrlrr_rxd), .rx_c1a(rlrlrr_rxa),
        .tx_c0d(rlrlrl_txd), .tx_c0a(rlrlrl_txa),
        .tx_c1d(rlrlrr_txd), .tx_c1a(rlrlrr_txa),
        .tx_pd(rlrlr_txd), .tx_pa(rlrlr_txa)
    );
    leaf #(
        .NOC_WID(16)
    ) leaf_rlrlrl (
        .clk  (wb_clk_i),
        .rst  (wb_rst_i),
        .rx_pr(rlrlrl_rxr),
        .rx_pd(rlrlrl_rxd),
        .rx_pa(rlrlrl_rxa),
        .tx_pd(rlrlrl_txd),
        .tx_pa(rlrlrl_txa)
    );
    leaf #(
        .NOC_WID(16)
    ) leaf_rlrlrr (
        .clk  (wb_clk_i),
        .rst  (wb_rst_i),
        .rx_pr(rlrlrr_rxr),
        .rx_pd(rlrlrr_rxd),
        .rx_pa(rlrlrr_rxa),
        .tx_pd(rlrlrr_txd),
        .tx_pa(rlrlrr_txa)
    );
    PE_up rlrr (
        .clk(wb_clk_i), .rst(wb_rst_i),
        .cfg_dat(br3_dat), .cfg_adr(br3_adr), .slv_addr(11'h0),
        .rx_pr(rlrr_rxr), .rx_pd(rlrr_rxd), .rx_pa(rlrr_rxa),
        .rx_c0r(rlrrl_rxr), .rx_c0d(rlrrl_rxd), .rx_c0a(rlrrl_rxa),
        .rx_c1r(rlrrr_rxr), .rx_c1d(rlrrr_rxd), .rx_c1a(rlrrr_rxa),
        .tx_c0d(rlrrl_txd), .tx_c0a(rlrrl_txa),
        .tx_c1d(rlrrr_txd), .tx_c1a(rlrrr_txa),
        .tx_pd(rlrr_txd), .tx_pa(rlrr_txa)
    );
    PE_right rlrrl (
        .clk(wb_clk_i), .rst(wb_rst_i),
        .cfg_dat(br3_dat), .cfg_adr(br3_adr), .slv_addr(11'hc),
        .rx_pr(rlrrl_rxr), .rx_pd(rlrrl_rxd), .rx_pa(rlrrl_rxa),
        .rx_c0r(rlrrll_rxr), .rx_c0d(rlrrll_rxd), .rx_c0a(rlrrll_rxa),
        .rx_c1r(rlrrlr_rxr), .rx_c1d(rlrrlr_rxd), .rx_c1a(rlrrlr_rxa),
        .tx_c0d(rlrrll_txd), .tx_c0a(rlrrll_txa),
        .tx_c1d(rlrrlr_txd), .tx_c1a(rlrrlr_txa),
        .tx_pd(rlrrl_txd), .tx_pa(rlrrl_txa)
    );
    leaf #(
        .NOC_WID(16)
    ) leaf_rlrrll (
        .clk  (wb_clk_i),
        .rst  (wb_rst_i),
        .rx_pr(rlrrll_rxr),
        .rx_pd(rlrrll_rxd),
        .rx_pa(rlrrll_rxa),
        .tx_pd(rlrrll_txd),
        .tx_pa(rlrrll_txa)
    );
    leaf #(
        .NOC_WID(16)
    ) leaf_rlrrlr (
        .clk  (wb_clk_i),
        .rst  (wb_rst_i),
        .rx_pr(rlrrlr_rxr),
        .rx_pd(rlrrlr_rxd),
        .rx_pa(rlrrlr_rxa),
        .tx_pd(rlrrlr_txd),
        .tx_pa(rlrrlr_txa)
    );
    PE_left rlrrr (
        .clk(wb_clk_i), .rst(wb_rst_i),
        .cfg_dat(br3_dat), .cfg_adr(br3_adr), .slv_addr(11'h18),
        .rx_pr(rlrrr_rxr), .rx_pd(rlrrr_rxd), .rx_pa(rlrrr_rxa),
        .rx_c0r(rlrrrl_rxr), .rx_c0d(rlrrrl_rxd), .rx_c0a(rlrrrl_rxa),
        .rx_c1r(rlrrrr_rxr), .rx_c1d(rlrrrr_rxd), .rx_c1a(rlrrrr_rxa),
        .tx_c0d(rlrrrl_txd), .tx_c0a(rlrrrl_txa),
        .tx_c1d(rlrrrr_txd), .tx_c1a(rlrrrr_txa),
        .tx_pd(rlrrr_txd), .tx_pa(rlrrr_txa)
    );
    leaf #(
        .NOC_WID(16)
    ) leaf_rlrrrl (
        .clk  (wb_clk_i),
        .rst  (wb_rst_i),
        .rx_pr(rlrrrl_rxr),
        .rx_pd(rlrrrl_rxd),
        .rx_pa(rlrrrl_rxa),
        .tx_pd(rlrrrl_txd),
        .tx_pa(rlrrrl_txa)
    );
    leaf #(
        .NOC_WID(16)
    ) leaf_rlrrrr (
        .clk  (wb_clk_i),
        .rst  (wb_rst_i),
        .rx_pr(rlrrrr_rxr),
        .rx_pd(rlrrrr_rxd),
        .rx_pa(rlrrrr_rxa),
        .tx_pd(rlrrrr_txd),
        .tx_pa(rlrrrr_txa)
    );
    PE_up rr (
        .clk(wb_clk_i), .rst(wb_rst_i),
        .cfg_dat(br4_dat), .cfg_adr(br4_adr), .slv_addr(11'h0),
        .rx_pr(rr_rxr), .rx_pd(rr_rxd), .rx_pa(rr_rxa),
        .rx_c0r(rrl_rxr), .rx_c0d(rrl_rxd), .rx_c0a(rrl_rxa),
        .rx_c1r(rrr_rxr), .rx_c1d(rrr_rxd), .rx_c1a(rrr_rxa),
        .tx_c0d(rrl_txd), .tx_c0a(rrl_txa),
        .tx_c1d(rrr_txd), .tx_c1a(rrr_txa),
        .tx_pd(rr_txd), .tx_pa(rr_txa)
    );
    PE_right rrl (
        .clk(wb_clk_i), .rst(wb_rst_i),
        .cfg_dat(br4_dat), .cfg_adr(br4_adr), .slv_addr(11'hc),
        .rx_pr(rrl_rxr), .rx_pd(rrl_rxd), .rx_pa(rrl_rxa),
        .rx_c0r(rrll_rxr), .rx_c0d(rrll_rxd), .rx_c0a(rrll_rxa),
        .rx_c1r(rrlr_rxr), .rx_c1d(rrlr_rxd), .rx_c1a(rrlr_rxa),
        .tx_c0d(rrll_txd), .tx_c0a(rrll_txa),
        .tx_c1d(rrlr_txd), .tx_c1a(rrlr_txa),
        .tx_pd(rrl_txd), .tx_pa(rrl_txa)
    );
    PE_down rrll (
        .clk(wb_clk_i), .rst(wb_rst_i),
        .cfg_dat(br4_dat), .cfg_adr(br4_adr), .slv_addr(11'h18),
        .rx_pr(rrll_rxr), .rx_pd(rrll_rxd), .rx_pa(rrll_rxa),
        .rx_c0r(rrlll_rxr), .rx_c0d(rrlll_rxd), .rx_c0a(rrlll_rxa),
        .rx_c1r(rrllr_rxr), .rx_c1d(rrllr_rxd), .rx_c1a(rrllr_rxa),
        .tx_c0d(rrlll_txd), .tx_c0a(rrlll_txa),
        .tx_c1d(rrllr_txd), .tx_c1a(rrllr_txa),
        .tx_pd(rrll_txd), .tx_pa(rrll_txa)
    );
    PE_right rrlll (
        .clk(wb_clk_i), .rst(wb_rst_i),
        .cfg_dat(br4_dat), .cfg_adr(br4_adr), .slv_addr(11'h24),
        .rx_pr(rrlll_rxr), .rx_pd(rrlll_rxd), .rx_pa(rrlll_rxa),
        .rx_c0r(rrllll_rxr), .rx_c0d(rrllll_rxd), .rx_c0a(rrllll_rxa),
        .rx_c1r(rrlllr_rxr), .rx_c1d(rrlllr_rxd), .rx_c1a(rrlllr_rxa),
        .tx_c0d(rrllll_txd), .tx_c0a(rrllll_txa),
        .tx_c1d(rrlllr_txd), .tx_c1a(rrlllr_txa),
        .tx_pd(rrlll_txd), .tx_pa(rrlll_txa)
    );
    leaf #(
        .NOC_WID(16)
    ) leaf_rrllll (
        .clk  (wb_clk_i),
        .rst  (wb_rst_i),
        .rx_pr(rrllll_rxr),
        .rx_pd(rrllll_rxd),
        .rx_pa(rrllll_rxa),
        .tx_pd(rrllll_txd),
        .tx_pa(rrllll_txa)
    );
    leaf #(
        .NOC_WID(16)
    ) leaf_rrlllr (
        .clk  (wb_clk_i),
        .rst  (wb_rst_i),
        .rx_pr(rrlllr_rxr),
        .rx_pd(rrlllr_rxd),
        .rx_pa(rrlllr_rxa),
        .tx_pd(rrlllr_txd),
        .tx_pa(rrlllr_txa)
    );
    PE_left rrllr (
        .clk(wb_clk_i), .rst(wb_rst_i),
        .cfg_dat(br4_dat), .cfg_adr(br4_adr), .slv_addr(11'h30),
        .rx_pr(rrllr_rxr), .rx_pd(rrllr_rxd), .rx_pa(rrllr_rxa),
        .rx_c0r(rrllrl_rxr), .rx_c0d(rrllrl_rxd), .rx_c0a(rrllrl_rxa),
        .rx_c1r(rrllrr_rxr), .rx_c1d(rrllrr_rxd), .rx_c1a(rrllrr_rxa),
        .tx_c0d(rrllrl_txd), .tx_c0a(rrllrl_txa),
        .tx_c1d(rrllrr_txd), .tx_c1a(rrllrr_txa),
        .tx_pd(rrllr_txd), .tx_pa(rrllr_txa)
    );
    leaf #(
        .NOC_WID(16)
    ) leaf_rrllrl (
        .clk  (wb_clk_i),
        .rst  (wb_rst_i),
        .rx_pr(rrllrl_rxr),
        .rx_pd(rrllrl_rxd),
        .rx_pa(rrllrl_rxa),
        .tx_pd(rrllrl_txd),
        .tx_pa(rrllrl_txa)
    );
    leaf #(
        .NOC_WID(16)
    ) leaf_rrllrr (
        .clk  (wb_clk_i),
        .rst  (wb_rst_i),
        .rx_pr(rrllrr_rxr),
        .rx_pd(rrllrr_rxd),
        .rx_pa(rrllrr_rxa),
        .tx_pd(rrllrr_txd),
        .tx_pa(rrllrr_txa)
    );
    PE_up rrlr (
        .clk(wb_clk_i), .rst(wb_rst_i),
        .cfg_dat(br5_dat), .cfg_adr(br5_adr), .slv_addr(11'h0),
        .rx_pr(rrlr_rxr), .rx_pd(rrlr_rxd), .rx_pa(rrlr_rxa),
        .rx_c0r(rrlrl_rxr), .rx_c0d(rrlrl_rxd), .rx_c0a(rrlrl_rxa),
        .rx_c1r(rrlrr_rxr), .rx_c1d(rrlrr_rxd), .rx_c1a(rrlrr_rxa),
        .tx_c0d(rrlrl_txd), .tx_c0a(rrlrl_txa),
        .tx_c1d(rrlrr_txd), .tx_c1a(rrlrr_txa),
        .tx_pd(rrlr_txd), .tx_pa(rrlr_txa)
    );
    PE_right rrlrl (
        .clk(wb_clk_i), .rst(wb_rst_i),
        .cfg_dat(br5_dat), .cfg_adr(br5_adr), .slv_addr(11'hc),
        .rx_pr(rrlrl_rxr), .rx_pd(rrlrl_rxd), .rx_pa(rrlrl_rxa),
        .rx_c0r(rrlrll_rxr), .rx_c0d(rrlrll_rxd), .rx_c0a(rrlrll_rxa),
        .rx_c1r(rrlrlr_rxr), .rx_c1d(rrlrlr_rxd), .rx_c1a(rrlrlr_rxa),
        .tx_c0d(rrlrll_txd), .tx_c0a(rrlrll_txa),
        .tx_c1d(rrlrlr_txd), .tx_c1a(rrlrlr_txa),
        .tx_pd(rrlrl_txd), .tx_pa(rrlrl_txa)
    );
    leaf #(
        .NOC_WID(16)
    ) leaf_rrlrll (
        .clk  (wb_clk_i),
        .rst  (wb_rst_i),
        .rx_pr(rrlrll_rxr),
        .rx_pd(rrlrll_rxd),
        .rx_pa(rrlrll_rxa),
        .tx_pd(rrlrll_txd),
        .tx_pa(rrlrll_txa)
    );
    leaf #(
        .NOC_WID(16)
    ) leaf_rrlrlr (
        .clk  (wb_clk_i),
        .rst  (wb_rst_i),
        .rx_pr(rrlrlr_rxr),
        .rx_pd(rrlrlr_rxd),
        .rx_pa(rrlrlr_rxa),
        .tx_pd(rrlrlr_txd),
        .tx_pa(rrlrlr_txa)
    );
    PE_left rrlrr (
        .clk(wb_clk_i), .rst(wb_rst_i),
        .cfg_dat(br5_dat), .cfg_adr(br5_adr), .slv_addr(11'h18),
        .rx_pr(rrlrr_rxr), .rx_pd(rrlrr_rxd), .rx_pa(rrlrr_rxa),
        .rx_c0r(rrlrrl_rxr), .rx_c0d(rrlrrl_rxd), .rx_c0a(rrlrrl_rxa),
        .rx_c1r(rrlrrr_rxr), .rx_c1d(rrlrrr_rxd), .rx_c1a(rrlrrr_rxa),
        .tx_c0d(rrlrrl_txd), .tx_c0a(rrlrrl_txa),
        .tx_c1d(rrlrrr_txd), .tx_c1a(rrlrrr_txa),
        .tx_pd(rrlrr_txd), .tx_pa(rrlrr_txa)
    );
    leaf #(
        .NOC_WID(16)
    ) leaf_rrlrrl (
        .clk  (wb_clk_i),
        .rst  (wb_rst_i),
        .rx_pr(rrlrrl_rxr),
        .rx_pd(rrlrrl_rxd),
        .rx_pa(rrlrrl_rxa),
        .tx_pd(rrlrrl_txd),
        .tx_pa(rrlrrl_txa)
    );
    leaf #(
        .NOC_WID(16)
    ) leaf_rrlrrr (
        .clk  (wb_clk_i),
        .rst  (wb_rst_i),
        .rx_pr(rrlrrr_rxr),
        .rx_pd(rrlrrr_rxd),
        .rx_pa(rrlrrr_rxa),
        .tx_pd(rrlrrr_txd),
        .tx_pa(rrlrrr_txa)
    );
    PE_left rrr (
        .clk(wb_clk_i), .rst(wb_rst_i),
        .cfg_dat(br6_dat), .cfg_adr(br6_adr), .slv_addr(11'h0),
        .rx_pr(rrr_rxr), .rx_pd(rrr_rxd), .rx_pa(rrr_rxa),
        .rx_c0r(rrrl_rxr), .rx_c0d(rrrl_rxd), .rx_c0a(rrrl_rxa),
        .rx_c1r(rrrr_rxr), .rx_c1d(rrrr_rxd), .rx_c1a(rrrr_rxa),
        .tx_c0d(rrrl_txd), .tx_c0a(rrrl_txa),
        .tx_c1d(rrrr_txd), .tx_c1a(rrrr_txa),
        .tx_pd(rrr_txd), .tx_pa(rrr_txa)
    );
    PE_down rrrl (
        .clk(wb_clk_i), .rst(wb_rst_i),
        .cfg_dat(br6_dat), .cfg_adr(br6_adr), .slv_addr(11'hc),
        .rx_pr(rrrl_rxr), .rx_pd(rrrl_rxd), .rx_pa(rrrl_rxa),
        .rx_c0r(rrrll_rxr), .rx_c0d(rrrll_rxd), .rx_c0a(rrrll_rxa),
        .rx_c1r(rrrlr_rxr), .rx_c1d(rrrlr_rxd), .rx_c1a(rrrlr_rxa),
        .tx_c0d(rrrll_txd), .tx_c0a(rrrll_txa),
        .tx_c1d(rrrlr_txd), .tx_c1a(rrrlr_txa),
        .tx_pd(rrrl_txd), .tx_pa(rrrl_txa)
    );
    PE_right rrrll (
        .clk(wb_clk_i), .rst(wb_rst_i),
        .cfg_dat(br6_dat), .cfg_adr(br6_adr), .slv_addr(11'h18),
        .rx_pr(rrrll_rxr), .rx_pd(rrrll_rxd), .rx_pa(rrrll_rxa),
        .rx_c0r(rrrlll_rxr), .rx_c0d(rrrlll_rxd), .rx_c0a(rrrlll_rxa),
        .rx_c1r(rrrllr_rxr), .rx_c1d(rrrllr_rxd), .rx_c1a(rrrllr_rxa),
        .tx_c0d(rrrlll_txd), .tx_c0a(rrrlll_txa),
        .tx_c1d(rrrllr_txd), .tx_c1a(rrrllr_txa),
        .tx_pd(rrrll_txd), .tx_pa(rrrll_txa)
    );
    leaf #(
        .NOC_WID(16)
    ) leaf_rrrlll (
        .clk  (wb_clk_i),
        .rst  (wb_rst_i),
        .rx_pr(rrrlll_rxr),
        .rx_pd(rrrlll_rxd),
        .rx_pa(rrrlll_rxa),
        .tx_pd(rrrlll_txd),
        .tx_pa(rrrlll_txa)
    );
    leaf #(
        .NOC_WID(16)
    ) leaf_rrrllr (
        .clk  (wb_clk_i),
        .rst  (wb_rst_i),
        .rx_pr(rrrllr_rxr),
        .rx_pd(rrrllr_rxd),
        .rx_pa(rrrllr_rxa),
        .tx_pd(rrrllr_txd),
        .tx_pa(rrrllr_txa)
    );
    PE_left rrrlr (
        .clk(wb_clk_i), .rst(wb_rst_i),
        .cfg_dat(br6_dat), .cfg_adr(br6_adr), .slv_addr(11'h24),
        .rx_pr(rrrlr_rxr), .rx_pd(rrrlr_rxd), .rx_pa(rrrlr_rxa),
        .rx_c0r(rrrlrl_rxr), .rx_c0d(rrrlrl_rxd), .rx_c0a(rrrlrl_rxa),
        .rx_c1r(rrrlrr_rxr), .rx_c1d(rrrlrr_rxd), .rx_c1a(rrrlrr_rxa),
        .tx_c0d(rrrlrl_txd), .tx_c0a(rrrlrl_txa),
        .tx_c1d(rrrlrr_txd), .tx_c1a(rrrlrr_txa),
        .tx_pd(rrrlr_txd), .tx_pa(rrrlr_txa)
    );
    leaf #(
        .NOC_WID(16)
    ) leaf_rrrlrl (
        .clk  (wb_clk_i),
        .rst  (wb_rst_i),
        .rx_pr(rrrlrl_rxr),
        .rx_pd(rrrlrl_rxd),
        .rx_pa(rrrlrl_rxa),
        .tx_pd(rrrlrl_txd),
        .tx_pa(rrrlrl_txa)
    );
    leaf #(
        .NOC_WID(16)
    ) leaf_rrrlrr (
        .clk  (wb_clk_i),
        .rst  (wb_rst_i),
        .rx_pr(rrrlrr_rxr),
        .rx_pd(rrrlrr_rxd),
        .rx_pa(rrrlrr_rxa),
        .tx_pd(rrrlrr_txd),
        .tx_pa(rrrlrr_txa)
    );
    PE_up rrrr (
        .clk(wb_clk_i), .rst(wb_rst_i),
        .cfg_dat(br7_dat), .cfg_adr(br7_adr), .slv_addr(11'h0),
        .rx_pr(rrrr_rxr), .rx_pd(rrrr_rxd), .rx_pa(rrrr_rxa),
        .rx_c0r(rrrrl_rxr), .rx_c0d(rrrrl_rxd), .rx_c0a(rrrrl_rxa),
        .rx_c1r(rrrrr_rxr), .rx_c1d(rrrrr_rxd), .rx_c1a(rrrrr_rxa),
        .tx_c0d(rrrrl_txd), .tx_c0a(rrrrl_txa),
        .tx_c1d(rrrrr_txd), .tx_c1a(rrrrr_txa),
        .tx_pd(rrrr_txd), .tx_pa(rrrr_txa)
    );
    PE_right rrrrl (
        .clk(wb_clk_i), .rst(wb_rst_i),
        .cfg_dat(br7_dat), .cfg_adr(br7_adr), .slv_addr(11'hc),
        .rx_pr(rrrrl_rxr), .rx_pd(rrrrl_rxd), .rx_pa(rrrrl_rxa),
        .rx_c0r(rrrrll_rxr), .rx_c0d(rrrrll_rxd), .rx_c0a(rrrrll_rxa),
        .rx_c1r(rrrrlr_rxr), .rx_c1d(rrrrlr_rxd), .rx_c1a(rrrrlr_rxa),
        .tx_c0d(rrrrll_txd), .tx_c0a(rrrrll_txa),
        .tx_c1d(rrrrlr_txd), .tx_c1a(rrrrlr_txa),
        .tx_pd(rrrrl_txd), .tx_pa(rrrrl_txa)
    );
    leaf #(
        .NOC_WID(16)
    ) leaf_rrrrll (
        .clk  (wb_clk_i),
        .rst  (wb_rst_i),
        .rx_pr(rrrrll_rxr),
        .rx_pd(rrrrll_rxd),
        .rx_pa(rrrrll_rxa),
        .tx_pd(rrrrll_txd),
        .tx_pa(rrrrll_txa)
    );
    leaf #(
        .NOC_WID(16)
    ) leaf_rrrrlr (
        .clk  (wb_clk_i),
        .rst  (wb_rst_i),
        .rx_pr(rrrrlr_rxr),
        .rx_pd(rrrrlr_rxd),
        .rx_pa(rrrrlr_rxa),
        .tx_pd(rrrrlr_txd),
        .tx_pa(rrrrlr_txa)
    );
    PE_left rrrrr (
        .clk(wb_clk_i), .rst(wb_rst_i),
        .cfg_dat(br7_dat), .cfg_adr(br7_adr), .slv_addr(11'h18),
        .rx_pr(rrrrr_rxr), .rx_pd(rrrrr_rxd), .rx_pa(rrrrr_rxa),
        .rx_c0r(rrrrrl_rxr), .rx_c0d(rrrrrl_rxd), .rx_c0a(rrrrrl_rxa),
        .rx_c1r(rrrrrr_rxr), .rx_c1d(rrrrrr_rxd), .rx_c1a(rrrrrr_rxa),
        .tx_c0d(rrrrrl_txd), .tx_c0a(rrrrrl_txa),
        .tx_c1d(rrrrrr_txd), .tx_c1a(rrrrrr_txa),
        .tx_pd(rrrrr_txd), .tx_pa(rrrrr_txa)
    );
    leaf #(
        .NOC_WID(16)
    ) leaf_rrrrrl (
        .clk  (wb_clk_i),
        .rst  (wb_rst_i),
        .rx_pr(rrrrrl_rxr),
        .rx_pd(rrrrrl_rxd),
        .rx_pa(rrrrrl_rxa),
        .tx_pd(rrrrrl_txd),
        .tx_pa(rrrrrl_txa)
    );
    leaf #(
        .NOC_WID(16)
    ) leaf_rrrrrr (
        .clk  (wb_clk_i),
        .rst  (wb_rst_i),
        .rx_pr(rrrrrr_rxr),
        .rx_pd(rrrrrr_rxd),
        .rx_pa(rrrrrr_rxa),
        .tx_pd(rrrrrr_txd),
        .tx_pa(rrrrrr_txa)
    );
endmodule
`default_nettype none
`define MPRJ_IO_PADS 38

module user_proj_example #(
    parameter BITS = 32
) (
`ifdef USE_POWER_PINS
    inout vccd1,    // User area 1 1.8V supply
    inout vssd1,    // User area 1 digital ground
`endif

    // Wishbone Slave ports (WB MI A)
    input wire wb_clk_i,
    input wire wb_rst_i,
    input wire wbs_stb_i,
    input wire wbs_cyc_i,
    input wire wbs_we_i,
    input wire [3:0] wbs_sel_i,
    input wire [31:0] wbs_dat_i,
    input wire [31:0] wbs_adr_i,
    output wire wbs_ack_o,
    output wire [31:0] wbs_dat_o,

    // Logic Analyzer Signals
    input  wire [127:0] la_data_in,
    output wire [127:0] la_data_out,
    input  wire [127:0] la_oenb,

    // IOs
    input  wire [`MPRJ_IO_PADS-1:0] io_in,
    output wire [`MPRJ_IO_PADS-1:0] io_out,
    output wire [`MPRJ_IO_PADS-1:0] io_oeb,

    // Analog (direct connection to GPIO pad---use with caution)
    // Note that analog I/O is not available on the 7 lowest-numbered
    // GPIO pads, and so the analog_io indexing is offset from the
    // GPIO indexing by 7 (also upper 2 GPIOs do not have analog_io).
    inout wire [`MPRJ_IO_PADS-10:0] analog_io,

    // Independent clock (on independent integer divider)
    input wire user_clock2,

    // User maskable interrupt signals
    output wire [2:0] user_irq
);

  Top i_Top (
    .wb_clk_i (wb_clk_i ),
    .wb_rst_i (wb_rst_i ),
    .wbs_stb_i(wbs_stb_i),
    .wbs_cyc_i(wbs_cyc_i),
    .wbs_we_i (wbs_we_i ),
    .wbs_sel_i(wbs_sel_i),
    .wbs_dat_i(wbs_dat_i),
    .wbs_adr_i(wbs_adr_i),
    .wbs_ack_o(wbs_ack_o),
    .wbs_dat_o(wbs_dat_o)
  );

endmodule

`default_nettype wire
