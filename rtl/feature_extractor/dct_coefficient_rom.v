module dct_coefficient_rom #(
    parameter NUM_MFCCS = 13,
    parameter NUM_MEL_BANDS = 26,
    parameter DATA_WIDTH = 16
)(
    input wire [4:0] coefficient_index,  // k (0-12)
    input wire [4:0] band_index,         // n (0-25)
    output reg signed [DATA_WIDTH-1:0] dct_value
);

    // DCT matrix: 13 x 26
    // Each element = cos(pi * k * (n + 0.5) / 26) in Q15 format
    
    // We'll compute this combinationally using a simplified approach
    // In real implementation,we pre-compute all 338 values 
    // Pre-computed cosine table for common angles
    reg signed [DATA_WIDTH-1:0] cos_table [0:127];
    
    initial begin
        // cos(x) values for x = 0 to pi in steps of pi/128
        // Stored in Q15 format (multiply by 32767)
        cos_table[0]   = 16'h7FFF;  // cos(0) = 1.0
        cos_table[1]   = 16'h7FFD;  // cos(pi/128)
        cos_table[2]   = 16'h7FF6;  // cos(2pi/128)
        cos_table[4]   = 16'h7FD9;  // cos(4pi/128)
        cos_table[8]   = 16'h7F62;  // cos(8pi/128)
        cos_table[16]  = 16'h7D8A;  // cos(16pi/128) = cos(pi/8)
        cos_table[32]  = 16'h7641;  // cos(32pi/128) = cos(pi/4)
        cos_table[64]  = 16'h5A82;  // cos(64pi/128) = cos(pi/2)
        cos_table[128] = 16'h0000;  // cos(128pi/128) = cos(pi) = 0
        
        // Fill intermediate values with linear interpolation
        integer i;
        for (i = 3; i < 128; i = i + 1) begin
            if (cos_table[i] == 16'd0) begin
                // Find bracketing known values
                integer lower, upper;
                lower = i - 1;
                while (lower > 0 && cos_table[lower] == 16'd0) lower = lower - 1;
                upper = i + 1;
                while (upper < 128 && cos_table[upper] == 16'd0) upper = upper + 1;
                
                if (upper < 128) begin
                    // Linear interpolation
                    cos_table[i] = cos_table[lower] + 
                                   ((cos_table[upper] - cos_table[lower]) * 
                                    (i - lower)) / (upper - lower);
                end
            end
        end
    end
    
    // Compute DCT coefficient on-the-fly
    // angle = pi * k * (n + 0.5) / 26
    // Map to cos_table index: (angle / pi) * 128
    
    reg [15:0] angle_scaled;
    reg [6:0] cos_index;
    
    always @(*) begin
        // Compute angle index: k * (n + 0.5) * 128 / 26
        // Simplify: k * (2n + 1) * 64 / 26
        angle_scaled = coefficient_index * ((band_index << 1) + 1);
        angle_scaled = (angle_scaled * 64) / 26;
        
        // Wrap to [0, 127] range
        if (angle_scaled > 127) begin
            // Use cosine symmetry: cos(pi + x) = -cos(x)
            cos_index = angle_scaled % 128;
            if (angle_scaled >= 128 && angle_scaled < 256)
                dct_value = -cos_table[cos_index];
            else
                dct_value = cos_table[cos_index % 128];
        end
        else begin
            dct_value = cos_table[angle_scaled[6:0]];
        end
        
        // Apply DCT normalization factor
        // First coefficient (k=0) has factor sqrt(1/N)
        // Others have factor sqrt(2/N)
        if (coefficient_index == 0) begin
            // Multiply by sqrt(1/26) ≈ 0.196 in Q15 format
            dct_value = (dct_value * 16'd6430) >>> 15;
        end
        else begin
            // Multiply by sqrt(2/26) ≈ 0.277 in Q15 format
            dct_value = (dct_value * 16'd9093) >>> 15;
        end
    end

endmodule