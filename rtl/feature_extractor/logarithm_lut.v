module logarithm_lut #(
    parameter DATA_WIDTH = 16
)(
    input wire [DATA_WIDTH-1:0] value_in,
    output reg signed [DATA_WIDTH-1:0] log_out
);

    //  edge case: log(0) is undefined
    wire is_zero = (value_in == 16'd0)
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
    
    // Extract fractional part (8 bits after leading one)
    wire [7:0] fraction = (value_in << (15 - leading_one_pos)) >> 8;
    
    // LUT for fractional part of log2(1 + x) where x in [0, 1)
    // log2(1 + x/256) for x = 0 to 255
    reg [DATA_WIDTH-1:0] log_frac_lut [0:255];
    
    initial begin
        // Pre-computed log2(1 + i/256) * 32768 for i = 0..255
        log_frac_lut[0]   = 16'd0;     // log2(1.000) = 0.000
        log_frac_lut[1]   = 16'd57;    // log2(1.004) = 0.00174
        log_frac_lut[2]   = 16'd114;   
        log_frac_lut[3]   = 16'd170;   
        log_frac_lut[4]   = 16'd226;   
        log_frac_lut[5]   = 16'd282;   
        log_frac_lut[6]   = 16'd338;   
        log_frac_lut[7]   = 16'd394;   
        log_frac_lut[8]   = 16'd449;   
        log_frac_lut[9]   = 16'd505;   
        log_frac_lut[10]  = 16'd560;   
        log_frac_lut[16]  = 16'd890;   
        log_frac_lut[32]  = 16'd1760;  
        log_frac_lut[64]  = 16'd3460;  
        log_frac_lut[96]  = 16'd5070;  
        log_frac_lut[128] = 16'd6600;  
        log_frac_lut[160] = 16'd8050;  
        log_frac_lut[192] = 16'd9430;  
        log_frac_lut[224] = 16'd10740; 
        log_frac_lut[255] = 16'd11980; 
        
        // Fill in remaining entries with linear interpolation
        for (i = 11; i < 256; i = i + 1) begin
            if (i != 16 && i != 32 && i != 64 && i != 96 && 
                i != 128 && i != 160 && i != 192 && i != 224 && i != 255) begin
                // Linear interpolation between known points
                if (i < 16)
                    log_frac_lut[i] = (i * 890) / 16;
                else if (i < 32)
                    log_frac_lut[i] = 890 + ((i - 16) * (1760 - 890)) / 16;
                else if (i < 64)
                    log_frac_lut[i] = 1760 + ((i - 32) * (3460 - 1760)) / 32;
                else if (i < 96)
                    log_frac_lut[i] = 3460 + ((i - 64) * (5070 - 3460)) / 32;
                else if (i < 128)
                    log_frac_lut[i] = 5070 + ((i - 96) * (6600 - 5070)) / 32;
                else if (i < 160)
                    log_frac_lut[i] = 6600 + ((i - 128) * (8050 - 6600)) / 32;
                else if (i < 192)
                    log_frac_lut[i] = 8050 + ((i - 160) * (9430 - 8050)) / 32;
                else if (i < 224)
                    log_frac_lut[i] = 9430 + ((i - 192) * (10740 - 9430)) / 32;
                else
                    log_frac_lut[i] = 10740 + ((i - 224) * (11980 - 10740)) / 31;
            end
        end
    end
    
    // Combine integer and fractional parts
    // log2(value) = leading_one_pos + log2(1 + fraction/256)
    
    always @(*) begin
        if (is_zero) begin
            log_out = 16'h8000; // Return minimum value (most negative in Q15)
        end
        else begin
            // Integer part scaled to Q15: leading_one_pos * 32768
            // Fractional part from LUT (already in Q15)
            log_out = (leading_one_pos << 15) + log_frac_lut[fraction];
        end
    end

endmodule