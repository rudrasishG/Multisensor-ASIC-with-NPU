module audio_feature_extractor #(
    parameter FFT_SIZE = 512,
    parameter DATA_WIDTH = 16,
    parameter NUM_MEL_BANDS = 26,
    parameter NUM_MFCCS = 13,
    parameter SAMPLE_RATE = 16000
) (
    input wire clk,
    input wire rst_n,
    //fft mag spec
    input wire signed [DATA_WIDTH -1 : 0] fft_real [0 : FFT_SIZE/2],
    input wire signed [DATA_WIDTH -1 : 0] fft_imag [0 : FFT_SIZE/2],
    input wire fft_valid,
    //zero crossing rate time domain samples
    input wire signed [DATA_WIDTH-1 : 0] time_samples [0 : FFT_SIZE-1],
    //feature output
    output reg signed [DATA_WIDTH-1 :0] spectral_centroid,
    output reg signed [DATA_WIDTH-1:0] zero_crossing_rate,
    output reg signed [DATA_WIDTH-1 : 0] energy_envelope,
    output reg signed [DATA_WIDTH-1:0] mfcc [0:NUM_MFCCS-1],
    output reg features_valid    
);
//computing fft mag using CORDIC
    reg signed [DATA_WIDTH-1:0] magnitude [0:FFT_SIZE/2];
    reg stage1_valid;
    integer 1;
    //cordic mag computation instance
    reg [8:0] cordic_index;
    wire signed [DATA_WIDTH-1:0] cordic_magnitude;
    wire cordic_done;
    
    cordic_magnitude_computer #(
        .DATA_WIDTH(DATA_WIDTH),
    ) mag_cordic (
        .clk(clk),
        .rst_n(rst_n),
        .real_in(fft_real[cordic_index]),
        .imag_in(fft_imag[cordic_index]),
        .magnitude_out(cordic_magnitude),
        .valid_out(cordic_done)
    );
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cordic_index <= 9'd0;
            stage1_valid <= 1'b0;
        end
        else if (fft_valid) begin
            //compute mag for all fft bins
            cordic_index <= 9'd0;
            stage1_valid <= 1'b0;
        end
        else if (cordic_index < FFT_SIZE/2) begin
            if(cordic_done) begin
                magnitude[cordic_index] <= cordic_magnitude;
                cordic_index <= cordic_index + 1'b1;
            end
        end
        else begin
            stage1_valid <= 1'b1;
            cordic_index <= 9'd0;
        end
    end
    //stage 2: mel scale filter bank
    reg signed [DATA_WIDTH-1:0] mel_energy [0:NUM_MEL_BANDS-1];
    reg stage2_valid;
    reg signed [DATA_WIDTH-1:0] mel_filter_coeff;
    reg [4:0] mel_band_index;
    reg [8:0] fft_bin_index;

    mel_filterbank_rom #(
        .FFT_SIZE(FFT_SIZE),
        .NUM_MEL_BANDS(NUM_MEL_BANDS),
        .SAMPLE_RATE(SAMPLE_RATE)
    ) mel_rom(
        .mel_band(mel_band_index),
        .fft_bin(fft_bin_index),
        .coefficient(mel_filter_coeff)
    );
    reg signed [DATA_WIDTH+8:0] mel_accumulator;
    
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            mel_band_index <= 5'd0;
            fft_bin_index <= 9'd0;
            mel_accumulator <= {(DATA_WIDTH+9){1'b0}};
            stage2_valid <= 1'b0;
        end
        else if (stage1_valid) begin
            if (fft_bin_index < FFT_SIZE/2) begin
                mel_accumulator <= mel_accumulator + (magnitude[fft_bin_index] * mel_filter_coeff);
                fft_bin_index <= fft_bin_index + 1'b1;
            end
            else begin
                mel_energy[mel_band_index] <= mel_accumulator[DATA_WIDTH+7:8];
                mel_accumulator <= { (DATA_WIDTH+9){1'b0}};
                fft_bin_index <= 9'd0;
                if(mel_band_index < NUM_MEL_BANDS -1) begin
                    mel_band_index <= mel_band_index + 1'b1;
                end
                else begin
                    //all compute done
                    stage2_valid <= 1'b1;
                    mel_band_index <= 5'd0;
                end
            end
        end
        else begin
            stage2_valid <= 1'b0;
        end
    end

    //stage 3 : log energy from mel band...we'll use lut because log computation is difficult 
    reg signed [DATA_WIDTH-1:0] log_mel_energy [0:NUM_MEL_BANDS-1];
    reg stage3_valid;
    reg [4:0] log_index;
    wire signed [DATA_WIDTH-1:0] log_value;

    logarithm_lut #(
        .DATA_WIDTH(DATA_WIDTH)
    ) log_table (
        .value_in(mel_energy[log_index]),
        .log_out (log_value)
    );

    always @(posedge clk or negedge rst_n) begin
        if( !rst_n) begin
            log_index <= 5'd0;
            stage3_valid <= 1'b0;
        end
        else if (stage2_valid) begin
            if (log_index < NUM_MEL_BANDS) begin
                log_mel_energy[log_index] <= log_value;
                log_index <= log_index + 1'b1;
            end
            else begin
                stage3_valid <= 1'b1;
                log_index <= 5'd0;
            end
        end
        else begin
            stage3_valid <= 1'b0;
        end
    end

    //stage 4: DCT to get MFCCs..we'll keep 13 coeff for now but that is aimed for a wearable prototype..final prototype is hopefully wall mounted
    reg signed [DATA_WIDTH-1:0] mfcc_temp [0:NUM_MFCCS-1];
    reg stage4_valid;
    reg [4:0] dct_coeff_index;
    reg [4:0] dct_band_index;
    wire signed [DATA_WIDTH-1:0] dct_matrix_element;
    reg signed [DATA_WIDTH+8:0] dct_accumulator;

    dct_coefficient_rom #(
        .NUM_MFCCS(NUM_MFCCS),
        .NUM_MEL_BANDS(NUM_MEL_BANDS)       
    ) dct_rom (
        .coefficent_index(dct_coeff_index),
        .band_index(dct_band_index),
        .dct_value(dct_matrix_element)
    );
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            dct_coeff_index <= 5'd0;
            dct_band_index <= 5'd0;
            dct_accumulator <= {(DATA_WIDTH+9){1'b0}};
            stage4_valid <= 1'b0;
        end
        else if (stage3_valid) begin
            if (dct_band_index < NUM_MEL_BANDS) begin
                dct_accumulator <= dct_accumulator + (log_mel_energy[dct_band_index]* dct_matrix_element);
                dct_band_index <= dct_band_index + 1'b1;
            end
            else begin 
                mfcc_temp[dct_coeff_index] <= dct_accumulator[DATA_WIDTH+7:8];
                dct_accumulator <= {(DATA_WIDTH+9){1'b0}};
                dct_band_index <= 5'd0;
                if (dct_coeff_index < NUM_MFCCS -1) begin
                    dct_coeff_index <= dct_coeff_index + 1'b1;
                end
                else begin
                    stage4_valid <= 1'b1;
                    dct_coeff_index <= 5'd0;
                end
            end
        end
        else begin
            stage4_valid <= 1'b0;
        end
    end

    //stage 5 : spectral centroid computation ( parallel with mfcc)
    //centroid = sum(freq[i]*mag[i])/sum(mag[i])
    reg signed [DATA_WIDTH+16:0] weighted_sum;
    reg signed [DATA_WIDTH+8:0] magnitude_sum;
    reg signed [DATA_WIDTH-1:0] centroid_temp;
    reg [8:0] centroid_bin_index;
    reg centroid_valid;

    //frequency of each fft bin
    wire [15:0] bin_frequency = (cordic_index * SAMPLE_RATE) / FFT_SIZE;
    always @ (posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            weighted_sum <= { (DATA_WIDTH+17){1'b0}};
            magnitude_sum <= { (DATA_WIDTH+9){1'b0}};
            centroid_bin_index <= 9'd0;
            centroid_valid <= 1'b0;
        end
        else if (stage1_valid) begin
            //accumulate weighted frequencies
            if (centroid_bin_index < FFT_SIZE/2) begin
                weighted_sum <= weighted_sum + (bin_frequency*magnitude[centroid_bin_index]);
                magnitude_sum <= magnitude_sum + magnitude[centroid_bin_index];
                centroid_bin_index <= centroid_bin_index + 1'b1;
            end
            else begin
                //division for final centroid value
                centroid_temp <= weighted_sum / magnitude_sum;
                centroid_valid <= 1'b1;
                //reset for next frame
                weighted_sum <= {(DATA_WIDTH+17){1'b0}};
                magnitude_sum <= {(DATA_WIDTH+9){1'b0}};
                centroid_bin_index <= 9'b0;
            end
        end
        else begin
            centroid_valid <= 1'b0;
        end
    end

    //stage 6: zero crossing rate computation
    //zcr = (1/2N) * sum of |sign(x[n]) - sign(x[n-1])|
    reg [15:0] zero_crossing_count;
    reg signed [DATA_WIDTH-1:0] zcr_temp;
    reg [8:0] zcr_sample_index;
    reg zcr_valid;
    reg signed [DATA_WIDTH-1:0] prev_sample;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            zero_crossing_count <= 16'd0;
            zcr_sample_index <= 9'd0;
            zcr_valid <= 1'b0;
            prev_sample <= {DATA_WIDTH{1'b0}};
        end
        else if (fft_valid) begin
            //checking sample pair for sign change
            if (zcr_sample_index < FFT_SIZE-1) begin
                if ((time_samples[zcr_sample_index][DATA_WIDTH-1] != time_samples[zcr_sample_index+1][DATA_WIDTH-1])) begin
                    zero_crossing_count <= zero_crossing_count + 1'b1;
                end
                zcr_sample_index <= zcr_sample_index + 1'b1;
            end
            else begin
                // Normalize by frame length and scale to Q15 format
                zcr_temp <= (zero_crossing_count << (DATA_WIDTH-9));
                zcr_valid <= 1'b1;
                // Reset for next frame
                zero_crossing_count <= 16'd0;
                zcr_sample_index <= 9'd0;
            end
        end
        else begin
            zcr_valid <= 1'b0;
        end
    end

    //stage 7: energy envelope computation
    //energy = sum of x[n]^2 over frame

    reg signed [DATA_WIDTH+16:0] energy_accumulator;
    reg signed [DATA_WIDTH-1:0] energy_temp;
    reg [8:0] energy_sample_index;
    reg energy_valid;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            energy_accumulator <= { (DATA_WIDTH+17){1'b0}};
            energy_sample_index <= 9'd0;
            energy_valid <= 1'b0;
        end
        else if (fft_valid) begin
            if (energy_sample_index < FFT_SIZE) begin
                energy_accumulator <= energy_accumulator + (time_samples[energy_sample_index]* time_samples[energy_sample_index]);
                energy_sample_index <= energy_sample_index + 1'b1;
            end
            else begin
                energy_temp <= energy_accumulator [DATA_WIDTH+15:16];
                energy_valid <= 9'd0;
                //reset
                energy_accumulator <= { (DATA_WIDTH+17) {1'b0}};
                energy_sample_index <= 9'd0;
            end
        end
        else begin
            energy_valid <= 1'b0;
        end
    end

    //synchronise output
    reg all_features_ready;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
        features_valid <= 1'b0;
        all_features_ready <= 1'b0;
        end
        else begin
        // Check if all feature computation stages are complete
            all_features_ready <= stage4_valid && centroid_valid &&
            zcr_valid && energy_valid;
            if (all_features_ready) begin
                // Transfer all features to output registers
                for (i = 0; i < NUM_MFCCS; i = i + 1) begin
                mfcc[i] <= mfcc_temp[i];
            end
            spectral_centroid <= centroid_temp;
            zero_crossing_rate <= zcr_temp;
            energy_envelope <= energy_temp;
            features_valid <= 1'b1;
        end
        else begin
            features_valid <= 1'b0;
        end
        end
    end
endmodule


            


