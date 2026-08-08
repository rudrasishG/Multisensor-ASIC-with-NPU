module sqrt_lut #(
    parameter DATA_WIDTH = 16
)(
    input  wire [DATA_WIDTH-1:0] value_in,
    output reg  [DATA_WIDTH-1:0] sqrt_out
);

    wire is_zero = (value_in == 16'd0);

    // Find MSB position of input
    reg [4:0] leading_one_pos;
    integer i;       
    integer lower;   
    integer upper;   

    always @(*) begin
        leading_one_pos = 5'd0;
        for (i = DATA_WIDTH-1; i >= 0; i = i - 1) begin
            if (value_in[i] && leading_one_pos == 5'd0)
                leading_one_pos = i;
        end
    end

    // Normalise mantissa to [1,2): shift leading-one to bit 15, take bits [14:7]
    wire [7:0] mantissa = (value_in << (15 - leading_one_pos)) >> 8;

    // LUT: sqrt(1 + i/256) * 256 for i = 0..255
    reg [15:0] sqrt_lut_rom [0:255]; // FIX3: renamed from sqrt_lut to sqrt_lut_rom to avoid name clash with module

    initial begin
        // Anchor points
        sqrt_lut_rom[0]   = 16'd256;
        sqrt_lut_rom[1]   = 16'd257;
        sqrt_lut_rom[2]   = 16'd258;
        sqrt_lut_rom[4]   = 16'd260;
        sqrt_lut_rom[8]   = 16'd263;
        sqrt_lut_rom[16]  = 16'd270;
        sqrt_lut_rom[32]  = 16'd283;
        sqrt_lut_rom[64]  = 16'd307;
        sqrt_lut_rom[96]  = 16'd328;
        sqrt_lut_rom[128] = 16'd347;
        sqrt_lut_rom[160] = 16'd364;
        sqrt_lut_rom[192] = 16'd380;
        sqrt_lut_rom[224] = 16'd395;
        sqrt_lut_rom[255] = 16'd409;

        // Zero-fill then interpolate
        for (i = 0; i < 256; i = i + 1) begin
            if (sqrt_lut_rom[i] == 16'd0) begin
                lower = i - 1;
                while (lower > 0 && sqrt_lut_rom[lower] == 16'd0) lower = lower - 1;
                upper = i + 1;
                while (upper < 256 && sqrt_lut_rom[upper] == 16'd0) upper = upper + 1;
                if (upper < 256)
                    sqrt_lut_rom[i] = sqrt_lut_rom[lower] +
                                      ((sqrt_lut_rom[upper] - sqrt_lut_rom[lower]) *
                                       (i - lower)) / (upper - lower);
            end
        end
        // Fill entry 3 which is missed by the anchor loop above
        if (sqrt_lut_rom[3] == 16'd0)
            sqrt_lut_rom[3] = sqrt_lut_rom[2] + ((sqrt_lut_rom[4] - sqrt_lut_rom[2]) * 1) / 2;
    end

    reg [15:0] mantissa_sqrt;
    reg [4:0]  exponent_half;
    reg        exponent_odd;

    always @(*) begin
        if (is_zero) begin
            sqrt_out = 16'd0;
        end
        else begin
            mantissa_sqrt = sqrt_lut_rom[mantissa];
            exponent_half = leading_one_pos >> 1;
            exponent_odd  = leading_one_pos[0];

            // Odd exponent: multiply by sqrt(2) ≈ 362/256
            if (exponent_odd)
                mantissa_sqrt = (mantissa_sqrt * 16'd362) >> 8;

            // Scale by 2^(exponent/2): shift output
            if (exponent_half < 8)
                sqrt_out = (mantissa_sqrt << exponent_half) >> 8;
            else
                sqrt_out = mantissa_sqrt << (exponent_half - 8);
        end
    end

endmodule
