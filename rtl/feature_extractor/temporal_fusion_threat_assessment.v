module temporal_fusion_threat_assessment #(
    parameter DATA_WIDTH = 16,
    parameter AUDIO_BUFFER_SIZE = 100,
    parameter MOTION_BUFFER_SIZE = 300,
    parameter NUM_MFCCS = 13
)(
    input wire clk,
    input wire rst_n,

    input wire signed [DATA_WIDTH-1:0] audio_spectral_centroid,
    input wire signed [DATA_WIDTH-1:0] audio_zcr,
    input wire signed [DATA_WIDTH-1:0] audio_energy,
    input wire signed [DATA_WIDTH-1:0] audio_mfcc [0:NUM_MFCCS-1],
    input wire audio_features_valid,

    input wire signed [DATA_WIDTH-1:0] motion_total_accel,
    input wire signed [DATA_WIDTH-1:0] motion_jerk,
    input wire signed [DATA_WIDTH-1:0] motion_variance,
    input wire signed [DATA_WIDTH-1:0] motion_freq,
    input wire signed [DATA_WIDTH-1:0] motion_direction_changes,
    input wire signed [DATA_WIDTH-1:0] motion_pitch,
    input wire signed [DATA_WIDTH-1:0] motion_roll,
    input wire motion_features_valid,

    input wire [7:0] config_log_threshold,
    input wire [7:0] config_npu_threshold,
    input wire [7:0] config_alert_threshold,
    input wire config_enable_anomaly_detect,
    input wire config_enable_pattern_match,

    output reg [7:0] threat_confidence,
    output reg [2:0] threat_category,
    output reg [1:0] alert_level,
    output reg npu_interrupt,

    output reg signed [DATA_WIDTH-1:0] debug_correlation_score,
    output reg signed [DATA_WIDTH-1:0] debug_anomaly_score,
    output reg signed [DATA_WIDTH-1:0] debug_pattern_score
);

    localparam [2:0] CATEGORY_NONE       = 3'd0,
                     CATEGORY_HARASSMENT = 3'd1,
                     CATEGORY_ASSAULT    = 3'd2,
                     CATEGORY_FALL       = 3'd3,
                     CATEGORY_CHASE      = 3'd4,
                     CATEGORY_GENERAL    = 3'd5;

    localparam [1:0] ALERT_NONE      = 2'd0,
                     ALERT_LOG       = 2'd1,
                     ALERT_WAKE_NPU  = 2'd2,
                     ALERT_IMMEDIATE = 2'd3;

    localparam BASELINE_SAMPLES = 1000;
    localparam ALPHA_SHIFT      = 10;   // EMA alpha = 1/1024, ~32s time constant

  
    // Sliding Window Buffers
   

    reg signed [DATA_WIDTH-1:0] audio_centroid_buf [0:AUDIO_BUFFER_SIZE-1];
    reg signed [DATA_WIDTH-1:0] audio_zcr_buf      [0:AUDIO_BUFFER_SIZE-1];
    reg signed [DATA_WIDTH-1:0] audio_energy_buf   [0:AUDIO_BUFFER_SIZE-1];
    reg [6:0] audio_buf_index;
    reg audio_buf_full;

    reg signed [DATA_WIDTH-1:0] motion_accel_buf [0:MOTION_BUFFER_SIZE-1];
    reg signed [DATA_WIDTH-1:0] motion_jerk_buf  [0:MOTION_BUFFER_SIZE-1];
    reg signed [DATA_WIDTH-1:0] motion_var_buf   [0:MOTION_BUFFER_SIZE-1];
    reg [8:0] motion_buf_index;
    reg motion_buf_full;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            audio_buf_index  <= 7'd0;
            audio_buf_full   <= 1'b0;
            motion_buf_index <= 9'd0;
            motion_buf_full  <= 1'b0;
        end
        else begin
            if (audio_features_valid) begin
                audio_centroid_buf[audio_buf_index] <= audio_spectral_centroid;
                audio_zcr_buf[audio_buf_index]      <= audio_zcr;
                audio_energy_buf[audio_buf_index]   <= audio_energy;
                if (audio_buf_index == AUDIO_BUFFER_SIZE - 1) begin
                    audio_buf_index <= 7'd0;
                    audio_buf_full  <= 1'b1;
                end else
                    audio_buf_index <= audio_buf_index + 1'b1;
            end

            if (motion_features_valid) begin
                motion_accel_buf[motion_buf_index] <= motion_total_accel;
                motion_jerk_buf[motion_buf_index]  <= motion_jerk;
                motion_var_buf[motion_buf_index]   <= motion_variance;
                if (motion_buf_index == MOTION_BUFFER_SIZE - 1) begin
                    motion_buf_index <= 9'd0;
                    motion_buf_full  <= 1'b1;
                end else
                    motion_buf_index <= motion_buf_index + 1'b1;
            end
        end
    end

    // Cross-Modal Correlation


    reg [7:0]  correlation_score;
    reg signed [DATA_WIDTH+8:0]  audio_mean, motion_mean;
    reg signed [DATA_WIDTH+16:0] covariance_sum;
    reg signed [DATA_WIDTH+16:0] audio_variance_sum, motion_variance_sum;
    reg [1:0] corr_state;
    reg [6:0] corr_index;

    localparam CORR_IDLE    = 2'd0,
               CORR_MEAN    = 2'd1,
               CORR_COMPUTE = 2'd2,
               CORR_DONE    = 2'd3;

    reg signed [DATA_WIDTH-1:0] motion_downsampled [0:AUDIO_BUFFER_SIZE-1];

    //comb wires used here for illegal wire in always block..better fix here?
    wire signed [DATA_WIDTH-1:0] audio_dev       = audio_energy_buf[corr_index]   - audio_mean[DATA_WIDTH-1:0];
    wire signed [DATA_WIDTH-1:0] motion_dev      = motion_downsampled[corr_index] - motion_mean[DATA_WIDTH-1:0];
    wire signed [31:0]           variance_product = audio_variance_sum[23:0] * motion_variance_sum[23:0];

    integer ds_idx;
    always @(posedge clk) begin
        if (motion_buf_full) begin
            for (ds_idx = 0; ds_idx < AUDIO_BUFFER_SIZE; ds_idx = ds_idx + 1)
                motion_downsampled[ds_idx] <=
                    (motion_accel_buf[ds_idx*3] +
                     motion_accel_buf[ds_idx*3+1] +
                     motion_accel_buf[ds_idx*3+2]) / 3;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            corr_state        <= CORR_IDLE;
            correlation_score <= 8'd0;
        end
        else if (audio_buf_full && motion_buf_full) begin
            case (corr_state)
                CORR_IDLE: begin
                    corr_state   <= CORR_MEAN;
                    corr_index   <= 7'd0;
                    audio_mean   <= {(DATA_WIDTH+9){1'b0}};
                    motion_mean  <= {(DATA_WIDTH+9){1'b0}};
                end

                CORR_MEAN: begin
                    if (corr_index < AUDIO_BUFFER_SIZE) begin
                        audio_mean  <= audio_mean  + audio_energy_buf[corr_index];
                        motion_mean <= motion_mean + motion_downsampled[corr_index];
                        corr_index  <= corr_index + 1'b1;
                    end else begin
                        audio_mean          <= audio_mean  / AUDIO_BUFFER_SIZE;
                        motion_mean         <= motion_mean / AUDIO_BUFFER_SIZE;
                        corr_state          <= CORR_COMPUTE;
                        corr_index          <= 7'd0;
                        covariance_sum      <= {(DATA_WIDTH+17){1'b0}};
                        audio_variance_sum  <= {(DATA_WIDTH+17){1'b0}};
                        motion_variance_sum <= {(DATA_WIDTH+17){1'b0}};
                    end
                end

                CORR_COMPUTE: begin
                    if (corr_index < AUDIO_BUFFER_SIZE) begin
                        covariance_sum      <= covariance_sum      + (audio_dev  * motion_dev);
                        audio_variance_sum  <= audio_variance_sum  + (audio_dev  * audio_dev);
                        motion_variance_sum <= motion_variance_sum + (motion_dev * motion_dev);
                        corr_index          <= corr_index + 1'b1;
                    end else begin
                        if (variance_product > 0)
                            correlation_score <= ((covariance_sum * 100) >>> 16) /
                                                 (variance_product >>> 16);
                        else
                            correlation_score <= 8'd0;
                        corr_state <= CORR_DONE;
                    end
                end

                CORR_DONE: corr_state <= CORR_IDLE;
            endcase
        end
    end

    assign debug_correlation_score = correlation_score;

    // Pattern: HARASSMENT
   

    reg [7:0] harassment_score;
    reg [7:0] harassment_match_count;
    integer h_idx;
    integer buf_pos;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            harassment_score <= 8'd0;
        end
        else if (audio_buf_full && motion_buf_full) begin
            harassment_match_count = 8'd0;
            for (h_idx = 0; h_idx < 60; h_idx = h_idx + 1) begin
                buf_pos = (audio_buf_index - h_idx) % AUDIO_BUFFER_SIZE;
                if (audio_energy_buf[buf_pos] > 16'h4000)
                    harassment_match_count = harassment_match_count + 1;
            end
            if (motion_direction_changes > 16'h3000)
                harassment_match_count = harassment_match_count + 10;
            if (motion_total_accel > 16'h5000)
                harassment_match_count = harassment_match_count + 15;
            harassment_score <= (harassment_match_count > 100) ? 100 : harassment_match_count;
        end
    end


    // Pattern: ASSAULT


    reg [7:0] assault_score;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            assault_score <= 8'd0;
        end
        else if (audio_buf_full && motion_buf_full) begin
            assault_score = 8'd0;
            if (audio_energy    > 16'h6000) assault_score = assault_score + 40;
            if (motion_jerk     > 16'h7000) assault_score = assault_score + 40;
            if (motion_variance > 16'h5000) assault_score = assault_score + 20;
            if (assault_score > 100)        assault_score = 100;
        end
    end

    // Pattern: FALL (state machine)


    reg [7:0] fall_score;
    reg [2:0] fall_state_tracker;

    localparam FALL_NORMAL    = 3'd0,
               FALL_FREE_FALL = 3'd1,
               FALL_IMPACT    = 3'd2,
               FALL_STILLNESS = 3'd3;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            fall_score         <= 8'd0;
            fall_state_tracker <= FALL_NORMAL;
        end
        else if (motion_features_valid) begin
            case (fall_state_tracker)
                FALL_NORMAL: begin
                    if (motion_total_accel < 16'h0800) begin
                        fall_state_tracker <= FALL_FREE_FALL;
                        fall_score         <= 8'd30;
                    end
                end
                FALL_FREE_FALL: begin
                    if (motion_jerk > 16'h7800) begin
                        fall_state_tracker <= FALL_IMPACT;
                        fall_score         <= fall_score + 40;
                    end else if (motion_total_accel > 16'h2000) begin
                        fall_state_tracker <= FALL_NORMAL;
                        fall_score         <= 8'd0;
                    end
                end
                FALL_IMPACT: begin
                    if (motion_total_accel < 16'h1000 && motion_variance < 16'h0800) begin
                        fall_state_tracker <= FALL_STILLNESS;
                        fall_score         <= fall_score + 30;
                    end else begin
                        fall_state_tracker <= FALL_NORMAL;
                        fall_score         <= 8'd0;
                    end
                end
                FALL_STILLNESS: begin
                    fall_state_tracker <= FALL_NORMAL;
                end
            endcase
            if (fall_score > 100) fall_score <= 100;
        end
    end

    // Pattern: CHASE


    reg [7:0] chase_score;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            chase_score <= 8'd0;
        end
        else if (audio_buf_full && motion_buf_full) begin
            chase_score = 8'd0;
            if (motion_total_accel > 16'h4800)                          chase_score = chase_score + 30;
            if (motion_freq > 16'h0400 && motion_freq < 16'h0C00)       chase_score = chase_score + 30;
            if (audio_energy > 16'h4000)                                chase_score = chase_score + 40;
            if (chase_score > 100)                                       chase_score = 100;
        end
    end

   
    // Adaptive Baseline Anomaly Detection


    reg signed [DATA_WIDTH-1:0] baseline_audio_energy_mean;
    reg signed [DATA_WIDTH-1:0] baseline_motion_accel_mean;
    reg        baseline_learned;
    reg [15:0] baseline_sample_count;

    reg [7:0] audio_anomaly_score;
    reg [7:0] motion_anomaly_score;
    reg [7:0] combined_anomaly_score;

 
    wire signed [DATA_WIDTH-1:0] audio_deviation  = audio_energy       - baseline_audio_energy_mean;
    wire signed [DATA_WIDTH-1:0] motion_deviation = motion_total_accel - baseline_motion_accel_mean;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            baseline_sample_count      <= 16'd0;
            baseline_learned           <= 1'b0;
            baseline_audio_energy_mean <= {DATA_WIDTH{1'b0}};
            baseline_motion_accel_mean <= {DATA_WIDTH{1'b0}};
            audio_anomaly_score        <= 8'd0;
            motion_anomaly_score       <= 8'd0;
            combined_anomaly_score     <= 8'd0;
        end
        else begin
            // Phase 1: cold-start running mean for first BASELINE_SAMPLES frames
            if (!baseline_learned) begin
                if (audio_features_valid) begin
                    baseline_audio_energy_mean <=
                        ((baseline_audio_energy_mean * baseline_sample_count) +
                          audio_energy) / (baseline_sample_count + 1);
                    baseline_sample_count <= baseline_sample_count + 1;
                    if (baseline_sample_count == BASELINE_SAMPLES - 1)
                        baseline_learned <= 1'b1;
                end
                if (motion_features_valid) begin
                    baseline_motion_accel_mean <=
                        ((baseline_motion_accel_mean * baseline_sample_count) +
                          motion_total_accel) / (baseline_sample_count + 1);
                end
            end

            // Phase 2: protected EMA — only update while system is calm ....gate to prevent baseline to cross to real threat
            else if (threat_confidence < config_log_threshold) begin
                if (audio_features_valid)
                    baseline_audio_energy_mean <= baseline_audio_energy_mean +
                        ((audio_energy - baseline_audio_energy_mean) >>> ALPHA_SHIFT);
                if (motion_features_valid)
                    baseline_motion_accel_mean <= baseline_motion_accel_mean +
                        ((motion_total_accel - baseline_motion_accel_mean) >>> ALPHA_SHIFT);
            end

            // Anomaly scoring against current baseline
            if (baseline_learned && config_enable_anomaly_detect) begin
                audio_anomaly_score  <= (audio_deviation  > 16'h3000) ? 8'd100 :
                                        (audio_deviation  > 16'h2000) ? 8'd70  : 8'd0;
                motion_anomaly_score <= (motion_deviation > 16'h3000) ? 8'd100 :
                                        (motion_deviation > 16'h2000) ? 8'd70  : 8'd0;
                combined_anomaly_score <= (audio_anomaly_score + motion_anomaly_score) >> 1;
            end
        end
    end

    assign debug_anomaly_score = combined_anomaly_score;

    // Threat Confidence Fusion


    reg [7:0] pattern_max_score;
    reg [2:0] matched_category;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            threat_confidence  <= 8'd0;
            threat_category    <= CATEGORY_NONE;
            pattern_max_score  <= 8'd0;
            matched_category   <= CATEGORY_NONE;
        end
        else if (config_enable_pattern_match) begin
            pattern_max_score = harassment_score;
            matched_category  = CATEGORY_HARASSMENT;

            if (assault_score > pattern_max_score) begin
                pattern_max_score = assault_score;
                matched_category  = CATEGORY_ASSAULT;
            end
            if (fall_score > pattern_max_score) begin
                pattern_max_score = fall_score;
                matched_category  = CATEGORY_FALL;
            end
            if (chase_score > pattern_max_score) begin
                pattern_max_score = chase_score;
                matched_category  = CATEGORY_CHASE;
            end

            threat_confidence <= ((pattern_max_score * 40) +
                                  (correlation_score  * 30) +
                                  (combined_anomaly_score * 30)) / 100;
            threat_category   <= matched_category;
        end
        else begin
            threat_confidence <= (correlation_score + combined_anomaly_score) >> 1;
            threat_category   <= CATEGORY_GENERAL;
        end
    end

    assign debug_pattern_score = pattern_max_score;

    // Alert Generation
    

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            alert_level   <= ALERT_NONE;
            npu_interrupt <= 1'b0;
        end
        else begin
            if      (threat_confidence >= config_alert_threshold) begin
                alert_level   <= ALERT_IMMEDIATE;
                npu_interrupt <= 1'b1;
            end
            else if (threat_confidence >= config_npu_threshold) begin
                alert_level   <= ALERT_WAKE_NPU;
                npu_interrupt <= 1'b1;
            end
            else if (threat_confidence >= config_log_threshold) begin
                alert_level   <= ALERT_LOG;
                npu_interrupt <= 1'b0;
            end
            else begin
                alert_level   <= ALERT_NONE;
                npu_interrupt <= 1'b0;
            end
        end
    end

endmodule