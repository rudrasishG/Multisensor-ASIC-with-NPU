module dct_coefficient_rom #(
    parameter NUM_MFCCS     = 13,
    parameter NUM_MEL_BANDS = 26,
    parameter DATA_WIDTH    = 16
)(
    input  wire [4:0]                    coefficient_index,  // k (0-12)
    input  wire [4:0]                    band_index,         // n (0-25)
    output reg  signed [DATA_WIDTH-1:0]  dct_value
);

    // Cosine table: cos(x) for x = 0..pi in steps of pi/128, Q15 format
    // FIX1: array was [0:127] but initial block wrote to index 128 → OOB.
    //       Extended to [0:128] to hold the cos(pi)=0 anchor point.
    reg signed [DATA_WIDTH-1:0] cos_table [0:128];
    integer i;
    integer lower;
    integer upper;

    initial begin
        // Zero the whole table first so the interpolation loop can detect unfilled entries
        for (i = 0; i <= 128; i = i + 1)
            cos_table[i] = 16'sd0;

        // Known anchor values (Q15)
        cos_table[0]   = 16'h7FFF;  
        cos_table[1]   = 16'h7FFD;
        cos_table[2]   = 16'h7FF6;  
        cos_table[4]   = 16'h7FD9;  
        cos_table[8]   = 16'h7F62;  
        cos_table[16]  = 16'h7D8A;  
        cos_table[32]  = 16'h7641;  
        cos_table[64]  = 16'h5A82;  
        cos_table[128] = 16'h0000;  

        // Linear interpolation for entries still at zero
        for (i = 1; i < 128; i = i + 1) begin
            if (cos_table[i] == 16'sd0) begin
                lower = i - 1;
                while (lower > 0 && cos_table[lower] == 16'sd0) lower = lower - 1;
                upper = i + 1;
                while (upper <= 128 && cos_table[upper] == 16'sd0) upper = upper + 1;
                if (upper <= 128)
                    cos_table[i] = cos_table[lower] +
                                   ((cos_table[upper] - cos_table[lower]) *
                                    (i - lower)) / (upper - lower);
            end
        end
    end

    reg [15:0] angle_scaled;
    reg [6:0]  cos_idx;

    always @(*) begin
        // DCT-II angle: pi * k * (n + 0.5) / NUM_MEL_BANDS
        // Map to cos_table index: angle/pi * 128 = k*(2n+1)*64/26
        angle_scaled = (coefficient_index * ((band_index << 1) + 1) * 64) / NUM_MEL_BANDS;

        // Wrap with cosine symmetry
        if (angle_scaled > 128) begin
            cos_idx = angle_scaled[6:0];          // modulo 128 (drop bit 7+)
            if (angle_scaled < 256)
                dct_value = -cos_table[cos_idx];  
            else
                dct_value =  cos_table[cos_idx];  
        end
        else begin
            dct_value = cos_table[angle_scaled[7:0]]; 
        end

        // DCT normalization factors (Q15)
        // k=0: sqrt(1/N) ≈ 0.196 → 6430
        // k>0: sqrt(2/N) ≈ 0.277 → 9093
        if (coefficient_index == 5'd0)
            dct_value = (dct_value * 16'sd6430) >>> 15;
        else
            dct_value = (dct_value * 16'sd9093) >>> 15;
    end

endmodule
