//==============================================================================
// Module: sqrt_lut
// Description: Fast square root using lookup table with interpolation
//
// Computes sqrt(x) using 256-entry LUT with linear interpolation
// Input: 16-bit unsigned value
// Output: sqrt(x) in same format
//
// Method: Separate exponent and mantissa, use LUT for mantissa
//==============================================================================

module sqrt_lut #(
    parameter DATA_WIDTH = 16
)(
    input wire [DATA_WIDTH-1:0] value_in,
    output reg [DATA_WIDTH-1:0] sqrt_out
);

    // Handle zero
    wire is_zero = (value_in == 16'd0);
    
    // Find position of leading one (MSB)
    reg [4:0] leading_one_pos;
    integer i;
    
    always @(*) begin
        leading_one_pos = 5'd0;
        for (i = DATA_WIDTH-1; i >= 0; i = i - 1) begin
            if (value_in[i] && leading_one_pos == 5'd0) begin
                leading_one_pos = i;
            end
        end
    end
    
    // Extract mantissa (normalize to [1, 2) range)
    wire [7:0] mantissa = (value_in << (15 - leading_one_pos)) >> 8;
    
    // LUT for sqrt(1 + x/256) where x in [0, 255]
    reg [15:0] sqrt_lut [0:255];
    
    initial begin
        // Pre-computed sqrt(1 + i/256) * 256 for integer output
        sqrt_lut[0]   = 16'd256;   // sqrt(1.000) = 1.000
        sqrt_lut[1]   = 16'd257;   // sqrt(1.004) = 1.002
        sqrt_lut[2]   = 16'd258;   // sqrt(1.008) = 1.004
        sqrt_lut[4]   = 16'd260;   // sqrt(1.016) = 1.008
        sqrt_lut[8]   = 16'd263;   // sqrt(1.031) = 1.016
        sqrt_lut[16]  = 16'd270;   // sqrt(1.062) = 1.031
        sqrt_lut[32]  = 16'd283;   // sqrt(1.125) = 1.061
        sqrt_lut[64]  = 16'd307;   // sqrt(1.250) = 1.118
        sqrt_lut[96]  = 16'd328;   // sqrt(1.375) = 1.173
        sqrt_lut[128] = 16'd347;   // sqrt(1.500) = 1.225
        sqrt_lut[160] = 16'd364;   // sqrt(1.625) = 1.275
        sqrt_lut[192] = 16'd380;   // sqrt(1.750) = 1.323
        sqrt_lut[224] = 16'd395;   // sqrt(1.875) = 1.369
        sqrt_lut[255] = 16'd409;   // sqrt(1.996) = 1.413
        
        // Fill with linear interpolation
        for (i = 1; i < 256; i = i + 1) begin
            if (sqrt_lut[i] == 16'd0) begin
                integer lower, upper;
                lower = i - 1;
                while (lower > 0 && sqrt_lut[lower] == 16'd0) lower = lower - 1;
                upper = i + 1;
                while (upper < 256 && sqrt_lut[upper] == 16'd0) upper = upper + 1;
                
                if (upper < 256) begin
                    sqrt_lut[i] = sqrt_lut[lower] + 
                                  ((sqrt_lut[upper] - sqrt_lut[lower]) * 
                                   (i - lower)) / (upper - lower);
                end
            end
        end
    end
    
    // Combine exponent scaling with mantissa LUT result
    // sqrt(2^n * m) = 2^(n/2) * sqrt(m)
    
    reg [15:0] mantissa_sqrt;
    reg [4:0] exponent_half;
    reg exponent_odd;
    
    always @(*) begin
        if (is_zero) begin
            sqrt_out = 16'd0;
        end
        else begin
            mantissa_sqrt = sqrt_lut[mantissa];
            exponent_half = leading_one_pos >> 1;  // Divide by 2
            exponent_odd = leading_one_pos[0];     // Check if odd
            
            if (exponent_odd) begin
                // Odd exponent: multiply mantissa result by sqrt(2) ≈ 1.414
                mantissa_sqrt = (mantissa_sqrt * 16'd362) >> 8; // 362/256 ≈ 1.414
            end
            
            // Scale by 2^(exponent/2)
            if (exponent_half < 8) begin
                sqrt_out = (mantissa_sqrt << exponent_half) >> 8;
            end
            else begin
                sqrt_out = mantissa_sqrt << (exponent_half - 8);
            end
        end
    end

endmodule