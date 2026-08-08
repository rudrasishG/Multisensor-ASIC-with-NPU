module motion_feature_extractor #(
    parameter DATA_WIDTH  = 16,
    parameter SAMPLE_RATE = 100,
    parameter WINDOW_SIZE = 100,
    parameter FFT_SIZE    = 128
)(
    input wire clk,
    input wire rst_n,
    input wire signed [DATA_WIDTH-1:0] accel_x,
    input wire signed [DATA_WIDTH-1:0] accel_y,
    input wire signed [DATA_WIDTH-1:0] accel_z,
    input wire signed [DATA_WIDTH-1:0] gyro_x,
    input wire signed [DATA_WIDTH-1:0] gyro_y,
    input wire signed [DATA_WIDTH-1:0] gyro_z,
    input wire imu_data_valid,

    output reg signed [DATA_WIDTH-1:0] total_acceleration,
    output reg signed [DATA_WIDTH-1:0] jerk_magnitude,
    output reg signed [DATA_WIDTH-1:0] motion_variance,
    output reg signed [DATA_WIDTH-1:0] dominant_frequency,
    output reg signed [DATA_WIDTH-1:0] direction_change_rate,
    output reg signed [DATA_WIDTH-1:0] pitch_angle,
    output reg signed [DATA_WIDTH-1:0] roll_angle,
    output reg features_valid
);

    // Complementary filter coefficient: 0.98 in Q15
    localparam signed [DATA_WIDTH-1:0] COMP_ALPHA = 16'h7CCD;
    // dt = 1/100Hz = 0.01s in Q15
    localparam signed [DATA_WIDTH-1:0] DT = 16'h0147;

    // Orientation estimation
  
    reg signed  [DATA_WIDTH-1:0] gyro_pitch, gyro_roll, gyro_yaw;
    wire signed [DATA_WIDTH-1:0] accel_pitch, accel_roll;
    reg signed  [DATA_WIDTH-1:0] fused_pitch, fused_roll, fused_yaw;

    // pitch = atan2(accel_x, sqrt(ay²+az²))
    //       accel_yz_squared is DATA_WIDTH+1 bits (sum of two DATA_WIDTH
    //       products would need 2*DATA_WIDTH bits; kept as 17-bit sum).
    //       sqrt_lut.value_in is DATA_WIDTH (16-bit unsigned).
    //       Original code had [DATA_WIDTH:1] slice of a DATA_WIDTH+1 signal
    //       which drops the MSB — acceptable approximation preserved here
    //       but width annotation corrected.
    wire [DATA_WIDTH:0] accel_yz_squared_w = (accel_y * accel_y) + (accel_z * accel_z);
    wire [DATA_WIDTH-1:0] accel_yz_magnitude;

    sqrt_lut #(.DATA_WIDTH(DATA_WIDTH)) sqrt_yz (
        .value_in (accel_yz_squared_w[DATA_WIDTH:1]), // upper 16 of 17 bits
        .sqrt_out (accel_yz_magnitude)
    );

    cordic_atan2 #(.DATA_WIDTH(DATA_WIDTH)) atan2_pitch (
        .clk      (clk),
        .rst_n    (rst_n),
        .y_in     (accel_x),
        .x_in     (accel_yz_magnitude),
        .angle_out(accel_pitch),
        .valid_out()
    );

    wire [DATA_WIDTH:0] accel_xz_squared_w = (accel_x * accel_x) + (accel_z * accel_z);
    wire [DATA_WIDTH-1:0] accel_xz_magnitude;

    sqrt_lut #(.DATA_WIDTH(DATA_WIDTH)) sqrt_xz (
        .value_in (accel_xz_squared_w[DATA_WIDTH:1]),
        .sqrt_out (accel_xz_magnitude)
    );

    cordic_atan2 #(.DATA_WIDTH(DATA_WIDTH)) atan2_roll (
        .clk      (clk),
        .rst_n    (rst_n),
        .y_in     (accel_y),
        .x_in     (accel_xz_magnitude),
        .angle_out(accel_roll),
        .valid_out()
    );

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            gyro_pitch  <= {DATA_WIDTH{1'b0}};
            gyro_roll   <= {DATA_WIDTH{1'b0}};
            gyro_yaw    <= {DATA_WIDTH{1'b0}};
            fused_pitch <= {DATA_WIDTH{1'b0}};
            fused_roll  <= {DATA_WIDTH{1'b0}};
            fused_yaw   <= {DATA_WIDTH{1'b0}};
        end
        else if (imu_data_valid) begin
            gyro_pitch <= gyro_pitch + ((gyro_x * DT) >>> (DATA_WIDTH-1));
            gyro_roll  <= gyro_roll  + ((gyro_y * DT) >>> (DATA_WIDTH-1));
            gyro_yaw   <= gyro_yaw   + ((gyro_z * DT) >>> (DATA_WIDTH-1));
            fused_pitch <= ((COMP_ALPHA         * gyro_pitch) >>> (DATA_WIDTH-1)) +
                           (((16'h7FFF - COMP_ALPHA) * accel_pitch) >>> (DATA_WIDTH-1));
            fused_roll  <= ((COMP_ALPHA         * gyro_roll)  >>> (DATA_WIDTH-1)) +
                           (((16'h7FFF - COMP_ALPHA) * accel_roll)  >>> (DATA_WIDTH-1));
            fused_yaw   <= gyro_yaw;
        end
    end

 
    // Total acceleration magnitude: sqrt(ax²+ay²+az²)

  
    wire [DATA_WIDTH+1:0] accel_sq_sum_w =
        (accel_x * accel_x) + (accel_y * accel_y) + (accel_z * accel_z);
    wire [DATA_WIDTH-1:0] accel_magnitude;

    sqrt_lut #(.DATA_WIDTH(DATA_WIDTH)) accel_mag_sqrt (
        .value_in (accel_sq_sum_w[DATA_WIDTH+1:2]), // upper 16 of 18 bits
        .sqrt_out (accel_magnitude)
    );


    // Jerk = Δacceleration/sample

    reg signed [DATA_WIDTH-1:0] prev_accel_x, prev_accel_y, prev_accel_z;
    wire signed [DATA_WIDTH-1:0] jerk_x = accel_x - prev_accel_x;
    wire signed [DATA_WIDTH-1:0] jerk_y = accel_y - prev_accel_y;
    wire signed [DATA_WIDTH-1:0] jerk_z = accel_z - prev_accel_z;

    wire [DATA_WIDTH+1:0] jerk_sq_sum_w =
        (jerk_x * jerk_x) + (jerk_y * jerk_y) + (jerk_z * jerk_z);
    wire [DATA_WIDTH-1:0] jerk_mag;

    sqrt_lut #(.DATA_WIDTH(DATA_WIDTH)) jerk_mag_sqrt (
        .value_in (jerk_sq_sum_w[DATA_WIDTH+1:2]),
        .sqrt_out (jerk_mag)
    );

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            prev_accel_x <= {DATA_WIDTH{1'b0}};
            prev_accel_y <= {DATA_WIDTH{1'b0}};
            prev_accel_z <= {DATA_WIDTH{1'b0}};
        end
        else if (imu_data_valid) begin
            prev_accel_x <= accel_x;
            prev_accel_y <= accel_y;
            prev_accel_z <= accel_z;
        end
    end


    // Circular buffer + variance computation

    reg signed [DATA_WIDTH-1:0] accel_buffer [0:WINDOW_SIZE-1];
    reg [6:0] buffer_index;
    reg       buffer_full;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            buffer_index <= 7'd0;
            buffer_full  <= 1'b0;
        end
        else if (imu_data_valid) begin
            accel_buffer[buffer_index] <= accel_magnitude;
            if (buffer_index == WINDOW_SIZE - 1) begin
                buffer_index <= 7'd0;
                buffer_full  <= 1'b1;
            end
            else
                buffer_index <= buffer_index + 1'b1;
        end
    end

    reg signed [DATA_WIDTH+8:0]  sum_accumulator;
    reg signed [DATA_WIDTH-1:0]  mean_value;
    reg signed [DATA_WIDTH+16:0] variance_accumulator;
    reg signed [DATA_WIDTH-1:0]  variance_value;
    reg [6:0]  stat_index;
    reg [1:0]  stat_state;

    localparam STAT_IDLE = 2'd0, STAT_MEAN = 2'd1,
               STAT_VAR  = 2'd2, STAT_DONE = 2'd3;

    reg signed [DATA_WIDTH-1:0] deviation;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            stat_state           <= STAT_IDLE;
            stat_index           <= 7'd0;
            sum_accumulator      <= {(DATA_WIDTH+9){1'b0}};
            variance_accumulator <= {(DATA_WIDTH+17){1'b0}};
            mean_value           <= {DATA_WIDTH{1'b0}};
            variance_value       <= {DATA_WIDTH{1'b0}};
            deviation            <= {DATA_WIDTH{1'b0}};
        end
        else begin
            case (stat_state)
                STAT_IDLE: begin
                    if (buffer_full) begin
                        stat_state      <= STAT_MEAN;
                        stat_index      <= 7'd0;
                        sum_accumulator <= {(DATA_WIDTH+9){1'b0}};
                    end
                end
                STAT_MEAN: begin
                    if (stat_index < WINDOW_SIZE) begin
                        sum_accumulator <= sum_accumulator + accel_buffer[stat_index];
                        stat_index      <= stat_index + 1'b1;
                    end
                    else begin
                        mean_value           <= sum_accumulator / WINDOW_SIZE;
                        stat_state           <= STAT_VAR;
                        stat_index           <= 7'd0;
                        variance_accumulator <= {(DATA_WIDTH+17){1'b0}};
                    end
                end
                STAT_VAR: begin
                    if (stat_index < WINDOW_SIZE) begin
                        deviation            <= accel_buffer[stat_index] - mean_value; // FIX3
                        variance_accumulator <= variance_accumulator + (deviation * deviation);
                        stat_index           <= stat_index + 1'b1;
                    end
                    else begin
                        variance_value <= variance_accumulator / WINDOW_SIZE;
                        stat_state     <= STAT_DONE;
                    end
                end
                STAT_DONE: stat_state <= STAT_IDLE;
            endcase
        end
    end


    // FFT for dominant frequency

    reg  signed [DATA_WIDTH*128-1:0] fft_input_flat;
    wire signed [DATA_WIDTH*64-1:0]  fft_magnitude_flat;
    wire   fft_done;
    reg    start_fft;

    fft_motion_128 #(.DATA_WIDTH(DATA_WIDTH)) motion_fft (
        .clk                (clk),
        .rst_n              (rst_n),
        .start              (start_fft),
        .time_data_flat     (fft_input_flat),
        .freq_magnitude_flat(fft_magnitude_flat),
        .done               (fft_done)
    );

    reg signed [DATA_WIDTH-1:0] peak_frequency_val;
    reg [6:0]  peak_frequency_bin;
    reg [6:0]  fft_bin_index;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            fft_bin_index     <= 7'd0;
            peak_frequency_val <= {DATA_WIDTH{1'b0}};
            peak_frequency_bin <= 7'd0;
        end
        else if (fft_done) begin
            if (fft_bin_index < FFT_SIZE/2) begin
                if (fft_magnitude_flat[fft_bin_index*DATA_WIDTH +: DATA_WIDTH] > peak_frequency_val) begin
                    peak_frequency_val <= fft_magnitude_flat[fft_bin_index*DATA_WIDTH +: DATA_WIDTH];
                    peak_frequency_bin <= fft_bin_index;
                end
                fft_bin_index <= fft_bin_index + 1'b1;
            end
            else begin
                fft_bin_index <= 7'd0;
            end
        end
    end

    // Direction change rate

    reg signed [DATA_WIDTH-1:0] prev_motion_x, prev_motion_y, prev_motion_z;
    reg [7:0] direction_change_count;
    reg [7:0] direction_sample_count;
    reg signed [DATA_WIDTH-1:0] direction_change_rate_reg; // FIX4: latch before reset

    wire signed [DATA_WIDTH+1:0] dot_product =
        (accel_x * prev_motion_x) +
        (accel_y * prev_motion_y) +
        (accel_z * prev_motion_z);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            prev_motion_x              <= {DATA_WIDTH{1'b0}};
            prev_motion_y              <= {DATA_WIDTH{1'b0}};
            prev_motion_z              <= {DATA_WIDTH{1'b0}};
            direction_change_count     <= 8'd0;
            direction_sample_count     <= 8'd0;
            direction_change_rate_reg  <= {DATA_WIDTH{1'b0}};
        end
        else if (imu_data_valid) begin
            if (dot_product[DATA_WIDTH+1]) // sign bit set = negative → direction reversed
                direction_change_count <= direction_change_count + 1'b1;

            direction_sample_count <= direction_sample_count + 1'b1;
            prev_motion_x <= accel_x;
            prev_motion_y <= accel_y;
            prev_motion_z <= accel_z;

            // FIX4: compute and latch rate BEFORE resetting counters
            if (direction_sample_count == WINDOW_SIZE - 1) begin
                direction_change_rate_reg <= (direction_change_count << 8) / WINDOW_SIZE;
                direction_sample_count    <= 8'd0;
                direction_change_count    <= 8'd0;
            end
        end
    end


    // Output register

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            features_valid <= 1'b0;
        else if (stat_state == STAT_DONE) begin
            total_acceleration   <= accel_magnitude;
            jerk_magnitude       <= jerk_mag;
            motion_variance      <= variance_value;
            dominant_frequency   <= (peak_frequency_bin * SAMPLE_RATE) / FFT_SIZE;
            direction_change_rate <= direction_change_rate_reg; // FIX4
            pitch_angle          <= fused_pitch;
            roll_angle           <= fused_roll;
            features_valid       <= 1'b1;
        end
        else
            features_valid <= 1'b0;
    end

endmodule
