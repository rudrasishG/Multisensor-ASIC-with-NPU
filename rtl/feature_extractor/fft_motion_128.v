module fft_motion_128 #(
    parameter DATA_WIDTH = 16
)(
    input wire clk,
    input wire rst_n,
    input wire start,
    input wire signed [DATA_WIDTH-1:0] time_data [0:127],
    output reg signed [DATA_WIDTH-1:0] freq_magnitude [0:63],
    output reg done
);

    // FFT stages: log2(128) = 7 stages
    localparam NUM_STAGES = 7;
    
    // Internal memory for FFT butterfly computations
    reg signed [DATA_WIDTH-1:0] real_mem [0:127];
    reg signed [DATA_WIDTH-1:0] imag_mem [0:127];
    
    // Twiddle factor ROM (smaller than audio FFT)
    reg signed [DATA_WIDTH-1:0] twiddle_real [0:63];
    reg signed [DATA_WIDTH-1:0] twiddle_imag [0:63];
    
    initial begin
        // Twiddle factors W_128^k = e^(-j*2*pi*k/128), Q1.15 fixed-point
        // real[k] = round(cos(2*pi*k/128) * 32767)
        // imag[k] = round(-sin(2*pi*k/128) * 32767)
        twiddle_real[ 0] = 16'h7FFF;  twiddle_imag[ 0] = 16'h0000;  // W^0  (1 + 0j)
        twiddle_real[ 1] = 16'h7FD8;  twiddle_imag[ 1] = -16'h0648;  // W^1
        twiddle_real[ 2] = 16'h7F61;  twiddle_imag[ 2] = -16'h0C8C;  // W^2
        twiddle_real[ 3] = 16'h7E9C;  twiddle_imag[ 3] = -16'h12C8;  // W^3
        twiddle_real[ 4] = 16'h7D89;  twiddle_imag[ 4] = -16'h18F9;  // W^4
        twiddle_real[ 5] = 16'h7C29;  twiddle_imag[ 5] = -16'h1F1A;  // W^5
        twiddle_real[ 6] = 16'h7A7C;  twiddle_imag[ 6] = -16'h2528;  // W^6
        twiddle_real[ 7] = 16'h7884;  twiddle_imag[ 7] = -16'h2B1F;  // W^7
        twiddle_real[ 8] = 16'h7641;  twiddle_imag[ 8] = -16'h30FB;  // W^8
        twiddle_real[ 9] = 16'h73B5;  twiddle_imag[ 9] = -16'h36BA;  // W^9
        twiddle_real[10] = 16'h70E2;  twiddle_imag[10] = -16'h3C56;  // W^10
        twiddle_real[11] = 16'h6DC9;  twiddle_imag[11] = -16'h41CE;  // W^11
        twiddle_real[12] = 16'h6A6D;  twiddle_imag[12] = -16'h471C;  // W^12
        twiddle_real[13] = 16'h66CF;  twiddle_imag[13] = -16'h4C3F;  // W^13
        twiddle_real[14] = 16'h62F1;  twiddle_imag[14] = -16'h5133;  // W^14
        twiddle_real[15] = 16'h5ED7;  twiddle_imag[15] = -16'h55F5;  // W^15
        twiddle_real[16] = 16'h5A82;  twiddle_imag[16] = -16'h5A82;  // W^16 (45°)
        twiddle_real[17] = 16'h55F5;  twiddle_imag[17] = -16'h5ED7;  // W^17
        twiddle_real[18] = 16'h5133;  twiddle_imag[18] = -16'h62F1;  // W^18
        twiddle_real[19] = 16'h4C3F;  twiddle_imag[19] = -16'h66CF;  // W^19
        twiddle_real[20] = 16'h471C;  twiddle_imag[20] = -16'h6A6D;  // W^20
        twiddle_real[21] = 16'h41CE;  twiddle_imag[21] = -16'h6DC9;  // W^21
        twiddle_real[22] = 16'h3C56;  twiddle_imag[22] = -16'h70E2;  // W^22
        twiddle_real[23] = 16'h36BA;  twiddle_imag[23] = -16'h73B5;  // W^23
        twiddle_real[24] = 16'h30FB;  twiddle_imag[24] = -16'h7641;  // W^24
        twiddle_real[25] = 16'h2B1F;  twiddle_imag[25] = -16'h7884;  // W^25
        twiddle_real[26] = 16'h2528;  twiddle_imag[26] = -16'h7A7C;  // W^26
        twiddle_real[27] = 16'h1F1A;  twiddle_imag[27] = -16'h7C29;  // W^27
        twiddle_real[28] = 16'h18F9;  twiddle_imag[28] = -16'h7D89;  // W^28
        twiddle_real[29] = 16'h12C8;  twiddle_imag[29] = -16'h7E9C;  // W^29
        twiddle_real[30] = 16'h0C8C;  twiddle_imag[30] = -16'h7F61;  // W^30
        twiddle_real[31] = 16'h0648;  twiddle_imag[31] = -16'h7FD8;  // W^31
        twiddle_real[32] = 16'h0000;  twiddle_imag[32] = -16'h7FFF;  // W^32 (90°, 0 - j)
        twiddle_real[33] = -16'h0648;  twiddle_imag[33] = -16'h7FD8;  // W^33
        twiddle_real[34] = -16'h0C8C;  twiddle_imag[34] = -16'h7F61;  // W^34
        twiddle_real[35] = -16'h12C8;  twiddle_imag[35] = -16'h7E9C;  // W^35
        twiddle_real[36] = -16'h18F9;  twiddle_imag[36] = -16'h7D89;  // W^36
        twiddle_real[37] = -16'h1F1A;  twiddle_imag[37] = -16'h7C29;  // W^37
        twiddle_real[38] = -16'h2528;  twiddle_imag[38] = -16'h7A7C;  // W^38
        twiddle_real[39] = -16'h2B1F;  twiddle_imag[39] = -16'h7884;  // W^39
        twiddle_real[40] = -16'h30FB;  twiddle_imag[40] = -16'h7641;  // W^40
        twiddle_real[41] = -16'h36BA;  twiddle_imag[41] = -16'h73B5;  // W^41
        twiddle_real[42] = -16'h3C56;  twiddle_imag[42] = -16'h70E2;  // W^42
        twiddle_real[43] = -16'h41CE;  twiddle_imag[43] = -16'h6DC9;  // W^43
        twiddle_real[44] = -16'h471C;  twiddle_imag[44] = -16'h6A6D;  // W^44
        twiddle_real[45] = -16'h4C3F;  twiddle_imag[45] = -16'h66CF;  // W^45
        twiddle_real[46] = -16'h5133;  twiddle_imag[46] = -16'h62F1;  // W^46
        twiddle_real[47] = -16'h55F5;  twiddle_imag[47] = -16'h5ED7;  // W^47
        twiddle_real[48] = -16'h5A82;  twiddle_imag[48] = -16'h5A82;  // W^48 (135°)
        twiddle_real[49] = -16'h5ED7;  twiddle_imag[49] = -16'h55F5;  // W^49
        twiddle_real[50] = -16'h62F1;  twiddle_imag[50] = -16'h5133;  // W^50
        twiddle_real[51] = -16'h66CF;  twiddle_imag[51] = -16'h4C3F;  // W^51
        twiddle_real[52] = -16'h6A6D;  twiddle_imag[52] = -16'h471C;  // W^52
        twiddle_real[53] = -16'h6DC9;  twiddle_imag[53] = -16'h41CE;  // W^53
        twiddle_real[54] = -16'h70E2;  twiddle_imag[54] = -16'h3C56;  // W^54
        twiddle_real[55] = -16'h73B5;  twiddle_imag[55] = -16'h36BA;  // W^55
        twiddle_real[56] = -16'h7641;  twiddle_imag[56] = -16'h30FB;  // W^56
        twiddle_real[57] = -16'h7884;  twiddle_imag[57] = -16'h2B1F;  // W^57
        twiddle_real[58] = -16'h7A7C;  twiddle_imag[58] = -16'h2528;  // W^58
        twiddle_real[59] = -16'h7C29;  twiddle_imag[59] = -16'h1F1A;  // W^59
        twiddle_real[60] = -16'h7D89;  twiddle_imag[60] = -16'h18F9;  // W^60
        twiddle_real[61] = -16'h7E9C;  twiddle_imag[61] = -16'h12C8;  // W^61
        twiddle_real[62] = -16'h7F61;  twiddle_imag[62] = -16'h0C8C;  // W^62
        twiddle_real[63] = -16'h7FD8;  twiddle_imag[63] = -16'h0648;  // W^63
    end
    
    // FSM states
    reg [2:0] stage;
    reg [7:0] butterfly_index;
    reg [2:0] state;
    
    localparam IDLE = 3'd0;
    localparam LOAD = 3'd1;
    localparam COMPUTE = 3'd2;
    localparam MAGNITUDE = 3'd3;
    localparam DONE = 3'd4;
    
    // Butterfly computation wires
    wire signed [DATA_WIDTH-1:0] bfly_ar, bfly_ai, bfly_br, bfly_bi;
    wire signed [DATA_WIDTH-1:0] bfly_cr, bfly_ci, bfly_dr, bfly_di;
    reg signed [DATA_WIDTH-1:0] tw_r, tw_i;
    
    // Radix-2 butterfly
    fft_butterfly_r2 #(.DATA_WIDTH(DATA_WIDTH)) butterfly (
        .ar(bfly_ar), .ai(bfly_ai),
        .br(bfly_br), .bi(bfly_bi),
        .twiddle_r(tw_r), .twiddle_i(tw_i),
        .cr(bfly_cr), .ci(bfly_ci),
        .dr(bfly_dr), .di(bfly_di)
    );
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
            stage <= 3'd0;
            done <= 1'b0;
        end
        else begin
            case (state)
                IDLE: begin
                    if (start) begin
                        state <= LOAD;
                        stage <= 3'd0;
                        butterfly_index <= 8'd0;
                    end
                end
                
                LOAD: begin
                    // Load time-domain data, initialize imaginary to zero
                    if (butterfly_index < 128) begin
                        real_mem[butterfly_index] <= time_data[butterfly_index];
                        imag_mem[butterfly_index] <= {DATA_WIDTH{1'b0}};
                        butterfly_index <= butterfly_index + 1'b1;
                    end
                    else begin
                        state <= COMPUTE;
                        stage <= 3'd0;
                        butterfly_index <= 8'd0;
                    end
                end
                
                COMPUTE: begin
                    if (stage < NUM_STAGES) begin
                        state <= MAGNITUDE;
                    end
                end
                
                MAGNITUDE: begin
                    // Compute magnitude of first 64 bins
                    if (butterfly_index < 64) begin
                        wire signed [DATA_WIDTH+1:0] mag_sq = 
                            (real_mem[butterfly_index] * real_mem[butterfly_index]) +
                            (imag_mem[butterfly_index] * imag_mem[butterfly_index]);
                        // Approximate sqrt by shift (simplified)
                        freq_magnitude[butterfly_index] <= mag_sq[DATA_WIDTH:1];
                        butterfly_index <= butterfly_index + 1'b1;
                    end
                    else begin
                        state <= DONE;
                    end
                end
                
                DONE: begin
                    done <= 1'b1;
                    state <= IDLE;
                end
            endcase
        end
    end

endmodule

// Radix-2 butterfly unit
module fft_butterfly_r2 #(parameter DATA_WIDTH = 16)(
    input signed [DATA_WIDTH-1:0] ar, ai, br, bi,
    input signed [DATA_WIDTH-1:0] twiddle_r, twiddle_i,
    output signed [DATA_WIDTH-1:0] cr, ci, dr, di
);
    // Complex multiply: B * W
    wire signed [DATA_WIDTH+DATA_WIDTH-1:0] prod_r = 
        (br * twiddle_r) - (bi * twiddle_i);
    wire signed [DATA_WIDTH+DATA_WIDTH-1:0] prod_i = 
        (br * twiddle_i) + (bi * twiddle_r);
    
    wire signed [DATA_WIDTH-1:0] bw_r = prod_r >>> (DATA_WIDTH-1);
    wire signed [DATA_WIDTH-1:0] bw_i = prod_i >>> (DATA_WIDTH-1);
    
    // Butterfly: C = A + BW, D = A - BW
    assign cr = ar + bw_r;
    assign ci = ai + bw_i;
    assign dr = ar - bw_r;
    assign di = ai - bw_i;
endmodule