module mel_filterbank_rom #(
    parameter FFT_SIZE = 512,
    parameter NUM_MEL_BANDS = 26,
    parameter SAMPLE_RATE = 16000,
    parameter DATA_WIDTH = 16
)(
    input wire [4:0] mel_band,      // Which mel band (0-25)
    input wire [8:0] fft_bin,       // Which FFT bin (0-255)
    output reg signed [DATA_WIDTH-1:0] coefficient
);

    // Mel frequency boundaries for 26 bands from 0 Hz to 8000 Hz
    // Pre-computed using: mel = 2595 * log10(1 + freq/700)
    
    // Convert FFT bin to frequency: freq = (bin * SAMPLE_RATE) / FFT_SIZE
    wire [15:0] bin_freq = (fft_bin * SAMPLE_RATE) / FFT_SIZE;
    
    // Mel band center frequencies (in Hz)
    reg [15:0] mel_centers [0:NUM_MEL_BANDS-1];
    
    initial begin
        // Pre-computed mel center frequencies
        mel_centers[0]  = 16'd0;     // 0 Hz
        mel_centers[1]  = 16'd93;    // 93 Hz
        mel_centers[2]  = 16'd197;   // 197 Hz
        mel_centers[3]  = 16'd312;   // 312 Hz
        mel_centers[4]  = 16'd437;   // 437 Hz
        mel_centers[5]  = 16'd573;   // 573 Hz
        mel_centers[6]  = 16'd721;   // 721 Hz
        mel_centers[7]  = 16'd881;   // 881 Hz
        mel_centers[8]  = 16'd1054;  // 1054 Hz
        mel_centers[9]  = 16'd1241;  // 1241 Hz
        mel_centers[10] = 16'd1444;  // 1444 Hz
        mel_centers[11] = 16'd1663;  // 1663 Hz
        mel_centers[12] = 16'd1900;  // 1900 Hz
        mel_centers[13] = 16'd2157;  // 2157 Hz
        mel_centers[14] = 16'd2435;  // 2435 Hz
        mel_centers[15] = 16'd2737;  // 2737 Hz
        mel_centers[16] = 16'd3064;  // 3064 Hz
        mel_centers[17] = 16'd3420;  // 3420 Hz
        mel_centers[18] = 16'd3805;  // 3805 Hz
        mel_centers[19] = 16'd4224;  // 4224 Hz
        mel_centers[20] = 16'd4678;  // 4678 Hz
        mel_centers[21] = 16'd5172;  // 5172 Hz
        mel_centers[22] = 16'd5708;  // 5708 Hz
        mel_centers[23] = 16'd6290;  // 6290 Hz
        mel_centers[24] = 16'd6923;  // 6923 Hz
        mel_centers[25] = 16'd7609;  // 7609 Hz
    end
    
    // Compute triangular filter coefficient
    // Triangular filter has peak at center, slopes to zero at adjacent centers
    
    reg [15:0] mel_left, mel_center, mel_right;
    reg signed [15:0] distance_from_center;
    reg signed [15:0] bandwidth;
    
    always @(*) begin
        if (mel_band == 0) begin
            mel_left = 16'd0;
            mel_center = mel_centers[0];
            mel_right = mel_centers[1];
        end
        else if (mel_band == NUM_MEL_BANDS - 1) begin
            mel_left = mel_centers[NUM_MEL_BANDS - 2];
            mel_center = mel_centers[NUM_MEL_BANDS - 1];
            mel_right = 16'd8000; // Nyquist frequency
        end
        else begin
            mel_left = mel_centers[mel_band - 1];
            mel_center = mel_centers[mel_band];
            mel_right = mel_centers[mel_band + 1];
        end
        
        // Triangular window computation
        if (bin_freq < mel_left || bin_freq > mel_right) begin
            // Outside filter support - coefficient is zero
            coefficient = 16'h0000;
        end
        else if (bin_freq <= mel_center) begin
            // Left slope: rises from 0 at mel_left to 1.0 at mel_center
            bandwidth = mel_center - mel_left;
            distance_from_center = bin_freq - mel_left;
            // Coefficient = (distance / bandwidth) in Q15 format
            coefficient = (distance_from_center << 15) / bandwidth;
        end
        else begin
            // Right slope: falls from 1.0 at mel_center to 0 at mel_right
            bandwidth = mel_right - mel_center;
            distance_from_center = mel_right - bin_freq;
            // Coefficient = (distance / bandwidth) in Q15 format
            coefficient = (distance_from_center << 15) / bandwidth;
        end
    end

endmodule