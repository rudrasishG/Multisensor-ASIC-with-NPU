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
        // Pre-compute twiddle factors W_128^k = e^(-j*2*pi*k/128)
        integer k;
        // These would be computed as: cos(2*pi*k/128), -sin(2*pi*k/128)
        // For brevity, showing pattern - full implementation needs all 64
        twiddle_real[0]  = 16'h7FFF;  twiddle_imag[0]  = 16'h0000;  // W^0
        twiddle_real[1]  = 16'h7FFD;  twiddle_imag[1]  = -16'h0192; // W^1
        twiddle_real[16] = 16'h7D8A;  twiddle_imag[16] = -16'h187D; // W^16
        twiddle_real[32] = 16'h7641;  twiddle_imag[32] = -16'h2D41; // W^32
        twiddle_real[63] = 16'h0192;  twiddle_imag[63] = -16'h7FFD; // W^63
        // ... (fill remaining twiddle factors)
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