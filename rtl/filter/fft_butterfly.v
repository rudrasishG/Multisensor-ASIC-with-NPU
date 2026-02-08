/* Implements the basic radix-2 butterfly operation:
  A' = A + W*B
  B' = A - W*B
 Where W is the complex twiddle factor (cos - j*sin)
 This is the fundamental building block of the FFT algorithm.

 The butterfly performs complex multiplication and addition/subtraction.
 Complex multiplication: (a + jb) * (c + jd) = (ac - bd) + j(ad + bc)*/


module fft_butterfly #(
    parameter DATA_WIDTH = 16
)(
    input wire clk,
    input wire rst_n,
    
    // Input A (top branch)
    input wire signed [DATA_WIDTH-1:0] in_a_real,
    input wire signed [DATA_WIDTH-1:0] in_a_imag,
    
    // Input B (bottom branch)
    input wire signed [DATA_WIDTH-1:0] in_b_real,
    input wire signed [DATA_WIDTH-1:0] in_b_imag,
    
    // Twiddle factor W = cos(theta) - j*sin(theta)
    input wire signed [DATA_WIDTH-1:0] twiddle_real,
    input wire signed [DATA_WIDTH-1:0] twiddle_imag,
    
    // Outputs
    output reg signed [DATA_WIDTH-1:0] out_a_real,
    output reg signed [DATA_WIDTH-1:0] out_a_imag,
    output reg signed [DATA_WIDTH-1:0] out_b_real,
    output reg signed [DATA_WIDTH-1:0] out_b_imag
);


    // Extended width for multiplication results
    wire signed [2*DATA_WIDTH-1:0] mult_wr_br;  // Wr * Br
    wire signed [2*DATA_WIDTH-1:0] mult_wi_bi;  // Wi * Bi
    wire signed [2*DATA_WIDTH-1:0] mult_wr_bi;  // Wr * Bi
    wire signed [2*DATA_WIDTH-1:0] mult_wi_br;  // Wi * Br
    
    // Perform multiplications
    assign mult_wr_br = twiddle_real * in_b_real;
    assign mult_wi_bi = twiddle_imag * in_b_imag;
    assign mult_wr_bi = twiddle_real * in_b_imag;
    assign mult_wi_br = twiddle_imag * in_b_real;
    
    // Scale results back to DATA_WIDTH
    // For Q15 fixed-point: take bits [30:15] to properly scale
    wire signed [DATA_WIDTH-1:0] wb_real;
    wire signed [DATA_WIDTH-1:0] wb_imag;
    
    assign wb_real = (mult_wr_br - mult_wi_bi) >>> (DATA_WIDTH - 1);
    assign wb_imag = (mult_wr_bi + mult_wi_br) >>> (DATA_WIDTH - 1);
    
    
    // Extended width to handle potential overflow
    wire signed [DATA_WIDTH:0] sum_real;
    wire signed [DATA_WIDTH:0] sum_imag;
    wire signed [DATA_WIDTH:0] diff_real;
    wire signed [DATA_WIDTH:0] diff_imag;
    
    assign sum_real  = in_a_real + wb_real;
    assign sum_imag  = in_a_imag + wb_imag;
    assign diff_real = in_a_real - wb_real;
    assign diff_imag = in_a_imag - wb_imag;
    

    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            out_a_real <= {DATA_WIDTH{1'b0}};
            out_a_imag <= {DATA_WIDTH{1'b0}};
            out_b_real <= {DATA_WIDTH{1'b0}};
            out_b_imag <= {DATA_WIDTH{1'b0}};
        end
        else begin
            // Scale by 1/2 to prevent overflow accumulation
            // This is the standard scaling approach for FFT
            out_a_real <= sum_real[DATA_WIDTH:1];   // Arithmetic right shift
            out_a_imag <= sum_imag[DATA_WIDTH:1];
            out_b_real <= diff_real[DATA_WIDTH:1];
            out_b_imag <= diff_imag[DATA_WIDTH:1];
        end
    end

endmodule
