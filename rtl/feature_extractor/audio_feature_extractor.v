module audio_feature_extractor #(
    parameter FFT_SIZE      = 512,
    parameter DATA_WIDTH    = 16,
    parameter NUM_MEL_BANDS = 26,
    parameter NUM_MFCCS     = 13,
    parameter SAMPLE_RATE   = 16000
)(
    input wire clk,
    input wire rst_n,

    // fft_real_flat: (FFT_SIZE/2+1) × DATA_WIDTH packed bus
    input wire signed [DATA_WIDTH*(FFT_SIZE/2+1)-1:0] fft_real_flat,
    input wire signed [DATA_WIDTH*(FFT_SIZE/2+1)-1:0] fft_imag_flat,
    input wire fft_valid,

    // time_samples_flat: FFT_SIZE × DATA_WIDTH packed bus
    input wire signed [DATA_WIDTH*FFT_SIZE-1:0] time_samples_flat,

    output reg signed [DATA_WIDTH-1:0] spectral_centroid,
    output reg signed [DATA_WIDTH-1:0] zero_crossing_rate,
    output reg signed [DATA_WIDTH-1:0] energy_envelope,
   
    output reg signed [DATA_WIDTH*NUM_MFCCS-1:0] mfcc_flat,
    output reg features_valid
);

  
    // Helper functions to unpack flat buses
    // (synthesisers handle these as constant-index wire selects)
  
    // fft_real[k] = fft_real_flat[ k*DATA_WIDTH +: DATA_WIDTH ]
    // time_samples[k] = time_samples_flat[ k*DATA_WIDTH +: DATA_WIDTH ]


    reg signed [DATA_WIDTH-1:0] magnitude     [0:FFT_SIZE/2];
    reg signed [DATA_WIDTH-1:0] mel_energy    [0:NUM_MEL_BANDS-1];
    reg signed [DATA_WIDTH-1:0] log_mel_energy[0:NUM_MEL_BANDS-1];
    reg signed [DATA_WIDTH-1:0] mfcc_temp     [0:NUM_MFCCS-1];

    integer i; // FIX2: was "integer 1;" — '1' is not a legal identifier

    
    // Stage 1: CORDIC magnitude for each FFT bin
   
    reg stage1_valid;
    reg [8:0] cordic_index;
    wire signed [DATA_WIDTH-1:0] cordic_magnitude;
    wire cordic_done;

    // Unpack current FFT bin from flat bus
    wire signed [DATA_WIDTH-1:0] cur_fft_real = fft_real_flat[cordic_index*DATA_WIDTH +: DATA_WIDTH];
    wire signed [DATA_WIDTH-1:0] cur_fft_imag = fft_imag_flat[cordic_index*DATA_WIDTH +: DATA_WIDTH];

   
    cordic_magnitude_computer #(
        .DATA_WIDTH(DATA_WIDTH)
    ) mag_cordic (
        .clk          (clk),
        .rst_n        (rst_n),
        .real_in      (cur_fft_real),
        .imag_in      (cur_fft_imag),
        .magnitude_out(cordic_magnitude),
        .valid_out    (cordic_done)
    );

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cordic_index <= 9'd0;
            stage1_valid <= 1'b0;
        end
        else if (fft_valid) begin
            cordic_index <= 9'd0;
            stage1_valid <= 1'b0;
        end
        else if (cordic_index <= FFT_SIZE/2) begin
            if (cordic_done) begin
                magnitude[cordic_index] <= cordic_magnitude;
                if (cordic_index == FFT_SIZE/2) begin
                    stage1_valid <= 1'b1;
                    cordic_index <= 9'd0;
                end
                else
                    cordic_index <= cordic_index + 1'b1;
            end
        end
        else
            stage1_valid <= 1'b0;
    end

    // Stage 2: Mel filterbank
  
    reg stage2_valid;
    wire signed [DATA_WIDTH-1:0] mel_filter_coeff;
    reg [4:0] mel_band_index;
    reg [8:0] fft_bin_index;
    reg signed [DATA_WIDTH+8:0] mel_accumulator;

    mel_filterbank_rom #(
        .FFT_SIZE    (FFT_SIZE),
        .NUM_MEL_BANDS(NUM_MEL_BANDS),
        .SAMPLE_RATE (SAMPLE_RATE)
    ) mel_rom (
        .mel_band   (mel_band_index),
        .fft_bin    (fft_bin_index),
        .coefficient(mel_filter_coeff)
    );

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mel_band_index  <= 5'd0;
            fft_bin_index   <= 9'd0;
            mel_accumulator <= {(DATA_WIDTH+9){1'b0}};
            stage2_valid    <= 1'b0;
        end
        else if (stage1_valid) begin
            if (fft_bin_index < FFT_SIZE/2) begin
                mel_accumulator <= mel_accumulator + (magnitude[fft_bin_index] * mel_filter_coeff);
                fft_bin_index   <= fft_bin_index + 1'b1;
            end
            else begin
                mel_energy[mel_band_index] <= mel_accumulator[DATA_WIDTH+7:8];
                mel_accumulator <= {(DATA_WIDTH+9){1'b0}};
                fft_bin_index   <= 9'd0;
                if (mel_band_index < NUM_MEL_BANDS - 1) begin
                    mel_band_index <= mel_band_index + 1'b1;
                end
                else begin
                    stage2_valid   <= 1'b1;
                    mel_band_index <= 5'd0;
                end
            end
        end
        else
            stage2_valid <= 1'b0;
    end

    // Stage 3: Log mel energy via LUT

    reg stage3_valid;
    reg [4:0] log_index;
    wire signed [DATA_WIDTH-1:0] log_value;

    logarithm_lut #(.DATA_WIDTH(DATA_WIDTH)) log_table (
        .value_in(mel_energy[log_index]),
        .log_out (log_value)
    );

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            log_index    <= 5'd0;
            stage3_valid <= 1'b0;
        end
        else if (stage2_valid) begin
            if (log_index < NUM_MEL_BANDS) begin
                log_mel_energy[log_index] <= log_value;
                log_index <= log_index + 1'b1;
            end
            else begin
                stage3_valid <= 1'b1;
                log_index    <= 5'd0;
            end
        end
        else
            stage3_valid <= 1'b0;
    end


    // Stage 4: DCT to MFCCs

    reg stage4_valid;
    reg [4:0] dct_coeff_index;
    reg [4:0] dct_band_index;
    wire signed [DATA_WIDTH-1:0] dct_matrix_element;
    reg signed [DATA_WIDTH+8:0] dct_accumulator;

    dct_coefficient_rom #(
        .NUM_MFCCS    (NUM_MFCCS),
        .NUM_MEL_BANDS(NUM_MEL_BANDS)
    ) dct_rom (
        .coefficient_index(dct_coeff_index),
        .band_index       (dct_band_index),
        .dct_value        (dct_matrix_element)
    );

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            dct_coeff_index <= 5'd0;
            dct_band_index  <= 5'd0;
            dct_accumulator <= {(DATA_WIDTH+9){1'b0}};
            stage4_valid    <= 1'b0;
        end
        else if (stage3_valid) begin
            if (dct_band_index < NUM_MEL_BANDS) begin
                dct_accumulator <= dct_accumulator + (log_mel_energy[dct_band_index] * dct_matrix_element);
                dct_band_index  <= dct_band_index + 1'b1;
            end
            else begin
                mfcc_temp[dct_coeff_index] <= dct_accumulator[DATA_WIDTH+7:8];
                dct_accumulator <= {(DATA_WIDTH+9){1'b0}};
                dct_band_index  <= 5'd0;
                if (dct_coeff_index < NUM_MFCCS - 1) begin
                    dct_coeff_index <= dct_coeff_index + 1'b1;
                end
                else begin
                    stage4_valid    <= 1'b1;
                    dct_coeff_index <= 5'd0;
                end
            end
        end
        else
            stage4_valid <= 1'b0;
    end

    
    // Stage 5: Spectral centroid
    // centroid = Σ(freq[k]*mag[k]) / Σ(mag[k])
    
    reg signed [DATA_WIDTH+16:0] weighted_sum;
    reg signed [DATA_WIDTH+8:0]  magnitude_sum;
    reg signed [DATA_WIDTH-1:0]  centroid_temp;
    reg [8:0] centroid_bin_index;
    reg centroid_valid;

  
    wire [15:0] bin_frequency = (centroid_bin_index * SAMPLE_RATE) / FFT_SIZE;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            weighted_sum      <= {(DATA_WIDTH+17){1'b0}};
            magnitude_sum     <= {(DATA_WIDTH+9){1'b0}};
            centroid_bin_index<= 9'd0;
            centroid_valid    <= 1'b0;
        end
        else if (stage1_valid) begin
            if (centroid_bin_index < FFT_SIZE/2) begin
                weighted_sum      <= weighted_sum  + (bin_frequency * magnitude[centroid_bin_index]);
                magnitude_sum     <= magnitude_sum + magnitude[centroid_bin_index];
                centroid_bin_index<= centroid_bin_index + 1'b1;
            end
            else begin
                centroid_temp      <= (magnitude_sum != 0) ? weighted_sum / magnitude_sum : 0;
                centroid_valid     <= 1'b1;
                weighted_sum      <= {(DATA_WIDTH+17){1'b0}};
                magnitude_sum     <= {(DATA_WIDTH+9){1'b0}};
                centroid_bin_index <= 9'd0;
            end
        end
        else
            centroid_valid <= 1'b0;
    end

   
    // Stage 6: Zero crossing rate
    
    reg [15:0] zero_crossing_count;
    reg signed [DATA_WIDTH-1:0] zcr_temp;
    reg [8:0] zcr_sample_index;
    reg zcr_valid;

    // Unpack two adjacent time samples for sign comparison
    wire signed [DATA_WIDTH-1:0] ts_cur  = time_samples_flat[ zcr_sample_index   *DATA_WIDTH +: DATA_WIDTH];
    wire signed [DATA_WIDTH-1:0] ts_next = time_samples_flat[(zcr_sample_index+1)*DATA_WIDTH +: DATA_WIDTH];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            zero_crossing_count <= 16'd0;
            zcr_sample_index    <= 9'd0;
            zcr_valid           <= 1'b0;
        end
        else if (fft_valid) begin
            if (zcr_sample_index < FFT_SIZE - 1) begin
                if (ts_cur[DATA_WIDTH-1] != ts_next[DATA_WIDTH-1])
                    zero_crossing_count <= zero_crossing_count + 1'b1;
                zcr_sample_index <= zcr_sample_index + 1'b1;
            end
            else begin
                zcr_temp            <= (zero_crossing_count << (DATA_WIDTH-9));
                zcr_valid           <= 1'b1;
                zero_crossing_count <= 16'd0;
                zcr_sample_index    <= 9'd0;
            end
        end
        else
            zcr_valid <= 1'b0;
    end

 
    // Stage 7: Energy envelope = Σ x[n]²
   
    reg signed [DATA_WIDTH+16:0] energy_accumulator;
    reg signed [DATA_WIDTH-1:0]  energy_temp;
    reg [8:0] energy_sample_index;
    reg energy_valid;

    wire signed [DATA_WIDTH-1:0] ts_energy = time_samples_flat[energy_sample_index*DATA_WIDTH +: DATA_WIDTH];

    // energy_valid was asserted under fft_valid guard but fft_valid is
    //       a single pulse; the energy loop runs for FFT_SIZE+1 cycles after
    //       that pulse and would never set energy_valid.  Decouple: run the
    //       energy accumulation independently once fft_valid triggers a start
    //       flag.
    reg energy_running;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            energy_accumulator  <= {(DATA_WIDTH+17){1'b0}};
            energy_sample_index <= 9'd0;
            energy_valid        <= 1'b0;
            energy_running      <= 1'b0;
        end
        else begin
            if (fft_valid) begin
                energy_accumulator  <= {(DATA_WIDTH+17){1'b0}};
                energy_sample_index <= 9'd0;
                energy_valid        <= 1'b0;
                energy_running      <= 1'b1;
            end
            else if (energy_running) begin
                if (energy_sample_index < FFT_SIZE) begin
                    energy_accumulator  <= energy_accumulator + (ts_energy * ts_energy);
                    energy_sample_index <= energy_sample_index + 1'b1;
                    energy_valid        <= 1'b0;
                end
                else begin
                    energy_temp    <= energy_accumulator[DATA_WIDTH+15:16];
                    energy_valid   <= 1'b1;
                    energy_running <= 1'b0;
                end
            end
            else
                energy_valid <= 1'b0;
        end
    end

    
    // Output synchronisation: wait for all stages
   
    reg all_features_ready;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            features_valid    <= 1'b0;
            all_features_ready<= 1'b0;
            mfcc_flat         <= {(DATA_WIDTH*NUM_MFCCS){1'b0}};
        end
        else begin
            all_features_ready <= stage4_valid && centroid_valid && zcr_valid && energy_valid;
            if (all_features_ready) begin
                for (i = 0; i < NUM_MFCCS; i = i + 1)
                    mfcc_flat[i*DATA_WIDTH +: DATA_WIDTH] <= mfcc_temp[i];
                spectral_centroid  <= centroid_temp;
                zero_crossing_rate <= zcr_temp;
                energy_envelope    <= energy_temp;
                features_valid     <= 1'b1;
            end
            else
                features_valid <= 1'b0;
        end
    end

endmodule
