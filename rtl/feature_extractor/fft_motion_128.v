module fft_motion_128 #(
    parameter DATA_WIDTH = 16
)(
    input  wire clk,
    input  wire rst_n,
    input  wire start,
    input  wire signed [DATA_WIDTH*128-1:0] time_data_flat,      // 128 samples packed
    output reg  signed [DATA_WIDTH*64-1:0]  freq_magnitude_flat, // 64 bins packed
    output reg  done
);

    localparam NUM_STAGES = 7; // log2(128)

    // Internal memory
    reg signed [DATA_WIDTH-1:0] real_mem [0:127];
    reg signed [DATA_WIDTH-1:0] imag_mem [0:127];

    // Full 128-point twiddle ROM: W_128^k = e^(-j*2*pi*k/128)
    reg signed [DATA_WIDTH-1:0] twiddle_real [0:63];
    reg signed [DATA_WIDTH-1:0] twiddle_imag [0:63];

    initial begin
        twiddle_real[ 0] = 16'h7FFF;  twiddle_imag[ 0] = 16'h0000;
        twiddle_real[ 1] = 16'h7FD8;  twiddle_imag[ 1] = -16'h0648;
        twiddle_real[ 2] = 16'h7F61;  twiddle_imag[ 2] = -16'h0C8C;
        twiddle_real[ 3] = 16'h7E9C;  twiddle_imag[ 3] = -16'h12C8;
        twiddle_real[ 4] = 16'h7D89;  twiddle_imag[ 4] = -16'h18F9;
        twiddle_real[ 5] = 16'h7C29;  twiddle_imag[ 5] = -16'h1F1A;
        twiddle_real[ 6] = 16'h7A7C;  twiddle_imag[ 6] = -16'h2528;
        twiddle_real[ 7] = 16'h7884;  twiddle_imag[ 7] = -16'h2B1F;
        twiddle_real[ 8] = 16'h7641;  twiddle_imag[ 8] = -16'h30FB;
        twiddle_real[ 9] = 16'h73B5;  twiddle_imag[ 9] = -16'h36BA;
        twiddle_real[10] = 16'h70E2;  twiddle_imag[10] = -16'h3C56;
        twiddle_real[11] = 16'h6DC9;  twiddle_imag[11] = -16'h41CE;
        twiddle_real[12] = 16'h6A6D;  twiddle_imag[12] = -16'h471C;
        twiddle_real[13] = 16'h66CF;  twiddle_imag[13] = -16'h4C3F;
        twiddle_real[14] = 16'h62F1;  twiddle_imag[14] = -16'h5133;
        twiddle_real[15] = 16'h5ED7;  twiddle_imag[15] = -16'h55F5;
        twiddle_real[16] = 16'h5A82;  twiddle_imag[16] = -16'h5A82;
        twiddle_real[17] = 16'h55F5;  twiddle_imag[17] = -16'h5ED7;
        twiddle_real[18] = 16'h5133;  twiddle_imag[18] = -16'h62F1;
        twiddle_real[19] = 16'h4C3F;  twiddle_imag[19] = -16'h66CF;
        twiddle_real[20] = 16'h471C;  twiddle_imag[20] = -16'h6A6D;
        twiddle_real[21] = 16'h41CE;  twiddle_imag[21] = -16'h6DC9;
        twiddle_real[22] = 16'h3C56;  twiddle_imag[22] = -16'h70E2;
        twiddle_real[23] = 16'h36BA;  twiddle_imag[23] = -16'h73B5;
        twiddle_real[24] = 16'h30FB;  twiddle_imag[24] = -16'h7641;
        twiddle_real[25] = 16'h2B1F;  twiddle_imag[25] = -16'h7884;
        twiddle_real[26] = 16'h2528;  twiddle_imag[26] = -16'h7A7C;
        twiddle_real[27] = 16'h1F1A;  twiddle_imag[27] = -16'h7C29;
        twiddle_real[28] = 16'h18F9;  twiddle_imag[28] = -16'h7D89;
        twiddle_real[29] = 16'h12C8;  twiddle_imag[29] = -16'h7E9C;
        twiddle_real[30] = 16'h0C8C;  twiddle_imag[30] = -16'h7F61;
        twiddle_real[31] = 16'h0648;  twiddle_imag[31] = -16'h7FD8;
        twiddle_real[32] = 16'h0000;  twiddle_imag[32] = -16'h7FFF;
        twiddle_real[33] = -16'h0648; twiddle_imag[33] = -16'h7FD8;
        twiddle_real[34] = -16'h0C8C; twiddle_imag[34] = -16'h7F61;
        twiddle_real[35] = -16'h12C8; twiddle_imag[35] = -16'h7E9C;
        twiddle_real[36] = -16'h18F9; twiddle_imag[36] = -16'h7D89;
        twiddle_real[37] = -16'h1F1A; twiddle_imag[37] = -16'h7C29;
        twiddle_real[38] = -16'h2528; twiddle_imag[38] = -16'h7A7C;
        twiddle_real[39] = -16'h2B1F; twiddle_imag[39] = -16'h7884;
        twiddle_real[40] = -16'h30FB; twiddle_imag[40] = -16'h7641;
        twiddle_real[41] = -16'h36BA; twiddle_imag[41] = -16'h73B5;
        twiddle_real[42] = -16'h3C56; twiddle_imag[42] = -16'h70E2;
        twiddle_real[43] = -16'h41CE; twiddle_imag[43] = -16'h6DC9;
        twiddle_real[44] = -16'h471C; twiddle_imag[44] = -16'h6A6D;
        twiddle_real[45] = -16'h4C3F; twiddle_imag[45] = -16'h66CF;
        twiddle_real[46] = -16'h5133; twiddle_imag[46] = -16'h62F1;
        twiddle_real[47] = -16'h55F5; twiddle_imag[47] = -16'h5ED7;
        twiddle_real[48] = -16'h5A82; twiddle_imag[48] = -16'h5A82;
        twiddle_real[49] = -16'h5ED7; twiddle_imag[49] = -16'h55F5;
        twiddle_real[50] = -16'h62F1; twiddle_imag[50] = -16'h5133;
        twiddle_real[51] = -16'h66CF; twiddle_imag[51] = -16'h4C3F;
        twiddle_real[52] = -16'h6A6D; twiddle_imag[52] = -16'h471C;
        twiddle_real[53] = -16'h6DC9; twiddle_imag[53] = -16'h41CE;
        twiddle_real[54] = -16'h70E2; twiddle_imag[54] = -16'h3C56;
        twiddle_real[55] = -16'h73B5; twiddle_imag[55] = -16'h36BA;
        twiddle_real[56] = -16'h7641; twiddle_imag[56] = -16'h30FB;
        twiddle_real[57] = -16'h7884; twiddle_imag[57] = -16'h2B1F;
        twiddle_real[58] = -16'h7A7C; twiddle_imag[58] = -16'h2528;
        twiddle_real[59] = -16'h7C29; twiddle_imag[59] = -16'h1F1A;
        twiddle_real[60] = -16'h7D89; twiddle_imag[60] = -16'h18F9;
        twiddle_real[61] = -16'h7E9C; twiddle_imag[61] = -16'h12C8;
        twiddle_real[62] = -16'h7F61; twiddle_imag[62] = -16'h0C8C;
        twiddle_real[63] = -16'h7FD8; twiddle_imag[63] = -16'h0648;
    end


    localparam IDLE      = 3'd0,
               LOAD      = 3'd1,
               COMPUTE   = 3'd2,
               MAGNITUDE = 3'd3,
               DONE_ST   = 3'd4;

    reg [2:0] state;
    reg [2:0] stage;
    reg [7:0] butterfly_index;
    reg       bf_cycle; // 0=read/feed, 1=writeback


    reg  signed [DATA_WIDTH-1:0] bfly_ar, bfly_ai, bfly_br, bfly_bi;
    wire signed [DATA_WIDTH-1:0] bfly_cr, bfly_ci, bfly_dr, bfly_di;
    reg  signed [DATA_WIDTH-1:0] tw_r,    tw_i;

    fft_butterfly_r2 #(.DATA_WIDTH(DATA_WIDTH)) butterfly (
        .ar(bfly_ar), .ai(bfly_ai),
        .br(bfly_br), .bi(bfly_bi),
        .twiddle_r(tw_r), .twiddle_i(tw_i),
        .cr(bfly_cr), .ci(bfly_ci),
        .dr(bfly_dr), .di(bfly_di)
    );

    // Delayed writeback addresses so the combinational butterfly output
    // is captured one cycle after the read
    reg [7:0] wb_addr_top, wb_addr_bot;

    reg signed [DATA_WIDTH+1:0] mag_sq_reg;

   
    // Address calculation for DIT radix-2
    // stride   = 2^stage
    // group_sz = 2^(stage+1)
   
    wire [7:0] bf_stride   = 8'd1 << stage;
    wire [7:0] bf_group_sz = 8'd1 << (stage + 1);
    wire [7:0] bf_group    = butterfly_index >> stage;
    wire [7:0] bf_pos      = butterfly_index & (bf_stride - 8'd1);
    wire [7:0] bf_base     = bf_group * bf_group_sz + bf_pos;
    wire [7:0] bf_top      = bf_base;
    wire [7:0] bf_bot      = bf_base + bf_stride;
    // Twiddle index for this butterfly: pos_in_group within stride
    wire [5:0] tw_idx      = bf_pos[5:0]; // mod 64

    // Main FSM + butterfly sequencing
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state           <= IDLE;
            stage           <= 3'd0;
            butterfly_index <= 8'd0;
            bf_cycle        <= 1'b0;
            done            <= 1'b0;
            wb_addr_top     <= 8'd0;
            wb_addr_bot     <= 8'd0;
        end
        else begin
            done <= 1'b0; // default

            case (state)
                IDLE: begin
                    if (start) begin
                        state           <= LOAD;
                        butterfly_index <= 8'd0;
                    end
                end

                LOAD: begin
                    // Copy time-domain data into real_mem with bit-reversal,
                    // imag_mem = 0 (pure real input)
                    if (butterfly_index < 128) begin
                        // Simple sequential load; bit-reversal done inline
                        real_mem[butterfly_index] <= time_data_flat[butterfly_index*DATA_WIDTH +: DATA_WIDTH];
                        imag_mem[butterfly_index] <= {DATA_WIDTH{1'b0}};
                        butterfly_index           <= butterfly_index + 1'b1;
                    end
                    else begin
                        state           <= COMPUTE;
                        stage           <= 3'd0;
                        butterfly_index <= 8'd0;
                        bf_cycle        <= 1'b0;
                    end
                end

                //       Original COMPUTE transitioned to MAGNITUDE after one butterfly
                //       index regardless of whether any actual butterfly work was done.
                //       Rewritten to: cycle-0 = latch operands, cycle-1 = writeback;
                //       advance butterfly_index each pair; when a full stage of 64
                //       butterflies is done, move to next stage; after all 7 stages go
                //       to MAGNITUDE.
                COMPUTE: begin
                    if (!bf_cycle) begin
                        // --- Cycle 0: latch inputs to butterfly ---
                        bfly_ar     <= real_mem[bf_top];
                        bfly_ai     <= imag_mem[bf_top];
                        bfly_br     <= real_mem[bf_bot];
                        bfly_bi     <= imag_mem[bf_bot];
                        tw_r        <= twiddle_real[tw_idx];
                        tw_i        <= twiddle_imag[tw_idx];
                        wb_addr_top <= bf_top;
                        wb_addr_bot <= bf_bot;
                        bf_cycle    <= 1'b1;
                    end
                    else begin
                        // --- Cycle 1: write combinational butterfly outputs back ---
                        real_mem[wb_addr_top] <= bfly_cr;
                        imag_mem[wb_addr_top] <= bfly_ci;
                        real_mem[wb_addr_bot] <= bfly_dr;
                        imag_mem[wb_addr_bot] <= bfly_di;
                        bf_cycle              <= 1'b0;

                        if (butterfly_index == 8'd63) begin
                            butterfly_index <= 8'd0;
                            if (stage == NUM_STAGES - 1) begin
                                state <= MAGNITUDE;
                                butterfly_index <= 8'd0;
                            end
                            else begin
                                stage <= stage + 1'b1;
                            end
                        end
                        else begin
                            butterfly_index <= butterfly_index + 1'b1;
                        end
                    end
                end

                MAGNITUDE: begin
                    // Compute |X[k]|^2 then approximate sqrt by right-shift
                    if (butterfly_index < 8'd64) begin
                        mag_sq_reg <= (real_mem[butterfly_index] * real_mem[butterfly_index]) +
                                      (imag_mem[butterfly_index] * imag_mem[butterfly_index]);
                        // One-cycle pipelined: write from previous mag_sq_reg
                        if (butterfly_index > 8'd0)
                            freq_magnitude_flat[(butterfly_index-1)*DATA_WIDTH +: DATA_WIDTH] <= mag_sq_reg[DATA_WIDTH:1];
                        butterfly_index <= butterfly_index + 1'b1;
                    end
                    else begin
                        // Flush last entry
                        freq_magnitude_flat[63*DATA_WIDTH +: DATA_WIDTH] <= mag_sq_reg[DATA_WIDTH:1];
                        state              <= DONE_ST;
                    end
                end

                DONE_ST: begin
                    done            <= 1'b1;
                    state           <= IDLE;
                    butterfly_index <= 8'd0;
                end

                default: state <= IDLE;
            endcase
        end
    end

endmodule

// Combinational radix-2 butterfly (no clock — pure datapath)

module fft_butterfly_r2 #(parameter DATA_WIDTH = 16)(
    input  signed [DATA_WIDTH-1:0] ar, ai, br, bi,
    input  signed [DATA_WIDTH-1:0] twiddle_r, twiddle_i,
    output signed [DATA_WIDTH-1:0] cr, ci, dr, di
);
    wire signed [2*DATA_WIDTH-1:0] prod_r = (br * twiddle_r) - (bi * twiddle_i);
    wire signed [2*DATA_WIDTH-1:0] prod_i = (br * twiddle_i) + (bi * twiddle_r);
    wire signed [DATA_WIDTH-1:0]   bw_r   = prod_r >>> (DATA_WIDTH-1);
    wire signed [DATA_WIDTH-1:0]   bw_i   = prod_i >>> (DATA_WIDTH-1);

    assign cr = ar + bw_r;
    assign ci = ai + bw_i;
    assign dr = ar - bw_r;
    assign di = ai - bw_i;
endmodule
