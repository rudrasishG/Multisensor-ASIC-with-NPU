module motion_feature_extractor #(
    parameter DATA_WIDTH = 16,
    parameter SAMPLE_RATE = 100, 
    parameter WINDOW_SIZE = 100, 
    parameter FFT_SIZE = 128 
  )(
    input wire clk,
    input wire rst_n,
    // Raw IMU sensor inputs (3-axis each)
    input wire signed [DATA_WIDTH-1:0] accel_x,
    input wire signed [DATA_WIDTH-1:0] accel_y,
    input wire signed [DATA_WIDTH-1:0] accel_z,
    input wire signed [DATA_WIDTH-1:0] gyro_x,
    input wire signed [DATA_WIDTH-1:0] gyro_y,
    input wire signed [DATA_WIDTH-1:0] gyro_z,
    input wire imu_data_valid,
    // Motion feature vector output
    output reg signed [DATA_WIDTH-1:0] total_acceleration,
    output reg signed [DATA_WIDTH-1:0] jerk_magnitude,
    output reg signed [DATA_WIDTH-1:0] motion_variance,
    output reg signed [DATA_WIDTH-1:0] dominant_frequency,
    output reg signed [DATA_WIDTH-1:0] direction_change_rate,
    output reg signed [DATA_WIDTH-1:0] pitch_angle,
    output reg signed [DATA_WIDTH-1:0] roll_angle,
    output reg features_valid
  );
  /* Complementary filter combines accelerometer (accurate long-term, noisy
   short-term) with gyroscope (accurate short-term, drifts long-term)
   Complementary filter coefficient (0.98 means 98% gyro, 2% accel) */
  localparam signed [DATA_WIDTH-1:0] COMP_ALPHA = 16'h7CCD; // 0.98 in Q15
  reg signed [DATA_WIDTH-1:0] gyro_pitch, gyro_roll, gyro_yaw;
  wire signed [DATA_WIDTH-1:0] accel_pitch, accel_roll;
  // Fused orientation estimates
  reg signed [DATA_WIDTH-1:0] fused_pitch, fused_roll, fused_yaw;
  // Compute orientation from accelerometer using atan2
  // pitch = atan2(accel_x, sqrt(accel_y^2 + accel_z^2))
  // roll = atan2(accel_y, sqrt(accel_x^2 + accel_z^2))
  wire signed [DATA_WIDTH:0] accel_yz_squared;
  wire signed [DATA_WIDTH-1:0] accel_yz_magnitude;
  assign accel_yz_squared = (accel_y * accel_y) + (accel_z * accel_z);
  // Square root approximation using lookup table
  sqrt_lut #(
             .DATA_WIDTH(DATA_WIDTH)
           ) sqrt_inst (
             .value_in(accel_yz_squared[DATA_WIDTH:1]),
             .sqrt_out(accel_yz_magnitude)
           );
  // Arctangent computation using CORDIC
  cordic_atan2 #(
                 .DATA_WIDTH(DATA_WIDTH)
               ) atan2_pitch (
                 .y_in(accel_x),
                 .x_in(accel_yz_magnitude),
                 .angle_out(accel_pitch)
               );
  wire signed [DATA_WIDTH:0] accel_xz_squared;
  wire signed [DATA_WIDTH-1:0] accel_xz_magnitude;
  assign accel_xz_squared = (accel_x * accel_x) + (accel_z * accel_z);
  sqrt_lut #(
             .DATA_WIDTH(DATA_WIDTH)
           ) sqrt_inst2 (
             .value_in(accel_xz_squared[DATA_WIDTH:1]),
             .sqrt_out(accel_xz_magnitude)
           );
  cordic_atan2 #(
                 .DATA_WIDTH(DATA_WIDTH)
               ) atan2_roll (
                 .y_in(accel_y),
                 .x_in(accel_xz_magnitude),
                 .angle_out(accel_roll)
               );
  // Integration time step (dt) in fixed-point
  // dt = 1/SAMPLE_RATE = 0.01 seconds for 100Hz
  localparam signed [DATA_WIDTH-1:0] DT = 16'h0147; // 0.01 in Q15
  always @(posedge clk or negedge rst_n)
  begin
    if (!rst_n)
    begin
      gyro_pitch <= {DATA_WIDTH{1'b0}};
      gyro_roll <= {DATA_WIDTH{1'b0}};
      gyro_yaw <= {DATA_WIDTH{1'b0}};
      fused_pitch <= {DATA_WIDTH{1'b0}};
      fused_roll <= {DATA_WIDTH{1'b0}};
      fused_yaw <= {DATA_WIDTH{1'b0}};
    end
    else if (imu_data_valid)
    begin
      // angle += gyro_rate * dt
      gyro_pitch <= gyro_pitch + ((gyro_x * DT) >>> (DATA_WIDTH-1));
      gyro_roll <= gyro_roll + ((gyro_y * DT) >>> (DATA_WIDTH-1));
      gyro_yaw <= gyro_yaw + ((gyro_z * DT) >>> (DATA_WIDTH-1));
      //from complimentary filter
      // fused = alpha * gyro_integrated + (1-alpha) * accel_measured
      fused_pitch <= ((COMP_ALPHA * gyro_pitch) >>> (DATA_WIDTH-1)) +
                  (((16'h7FFF - COMP_ALPHA) * accel_pitch) >>> (DATA_WIDTH-1));
      fused_roll <= ((COMP_ALPHA * gyro_roll) >>> (DATA_WIDTH-1)) +
                 (((16'h7FFF - COMP_ALPHA) * accel_roll) >>> (DATA_WIDTH-1));
      // Yaw can only come from gyroscope (no magnetometer in this design)
      fused_yaw <= gyro_yaw;
    end
  end
  // Total acceleration is the magnitude of the 3D acceleration vector
  // magnitude = sqrt(ax^2 + ay^2 + az^2)
  wire signed [DATA_WIDTH+1:0] accel_squared_sum;
  reg signed [DATA_WIDTH-1:0] accel_magnitude;
  assign accel_squared_sum = (accel_x * accel_x) +
         (accel_y * accel_y) +
         (accel_z * accel_z);
  sqrt_lut #(
             .DATA_WIDTH(DATA_WIDTH)
           ) accel_mag_sqrt (
             .value_in(accel_squared_sum[DATA_WIDTH:1]),
             .sqrt_out(accel_magnitude)
           );
  // High jerk indicates sudden impacts, starts, stops
  // jerk = d(acceleration)/dt
  reg signed [DATA_WIDTH-1:0] prev_accel_x, prev_accel_y, prev_accel_z;
  wire signed [DATA_WIDTH-1:0] jerk_x, jerk_y, jerk_z;
  reg signed [DATA_WIDTH-1:0] jerk_mag;
  assign jerk_x = accel_x - prev_accel_x;
  assign jerk_y = accel_y - prev_accel_y;
  assign jerk_z = accel_z - prev_accel_z;
  wire signed [DATA_WIDTH+1:0] jerk_squared_sum;
  assign jerk_squared_sum = (jerk_x * jerk_x) +
         (jerk_y * jerk_y) +
         (jerk_z * jerk_z);
  sqrt_lut #(
             .DATA_WIDTH(DATA_WIDTH)
           ) jerk_mag_sqrt (
             .value_in(jerk_squared_sum[DATA_WIDTH:1]),
             .sqrt_out(jerk_mag)
           );
  always @(posedge clk or negedge rst_n)
  begin
    if (!rst_n)
    begin
      prev_accel_x <= {DATA_WIDTH{1'b0}};
      prev_accel_y <= {DATA_WIDTH{1'b0}};
      prev_accel_z <= {DATA_WIDTH{1'b0}};
    end
    else if (imu_data_valid)
    begin
      prev_accel_x <= accel_x;
      prev_accel_y <= accel_y;
      prev_accel_z <= accel_z;
    end
  end
  //sliding window for statistical answers
  // Maintain circular buffer of recent acceleration magnitudes
  // This allows computation of variance and other statistical measures
  reg signed [DATA_WIDTH-1:0] accel_buffer [0:WINDOW_SIZE-1];
  reg [6:0] buffer_index;
  reg buffer_full;
  always @(posedge clk or negedge rst_n)
  begin
    if (!rst_n)
    begin
      buffer_index <= 7'd0;
      buffer_full <= 1'b0;
    end
    else if (imu_data_valid)
    begin
      accel_buffer[buffer_index] <= accel_magnitude;
      if (buffer_index == WINDOW_SIZE - 1)
      begin
        buffer_index <= 7'd0;
        buffer_full <= 1'b1;
      end
      else
      begin
        buffer_index <= buffer_index + 1'b1;
      end
    end
  end
  // Variance measures how much acceleration fluctuates
  // Low variance = steady motion, High variance = chaotic motion
  // variance = E[(X - mean)^2]
  reg signed [DATA_WIDTH+8:0] sum_accumulator;
  reg signed [DATA_WIDTH-1:0] mean_value;
  reg signed [DATA_WIDTH+16:0] variance_accumulator;
  reg signed [DATA_WIDTH-1:0] variance_value;
  reg [6:0] stat_index;
  reg [1:0] stat_state;
  localparam STAT_IDLE = 2'd0;
  localparam STAT_MEAN = 2'd1;
  localparam STAT_VAR = 2'd2;
  localparam STAT_DONE = 2'd3;
  always @(posedge clk or negedge rst_n)
  begin
    if (!rst_n)
    begin
      stat_state <= STAT_IDLE;
      stat_index <= 7'd0;
      sum_accumulator <= {(DATA_WIDTH+9){1'b0}};
      variance_accumulator <= {(DATA_WIDTH+17){1'b0}};
    end
    else
    begin
      case (stat_state)
        STAT_IDLE:
        begin
          if (buffer_full)
          begin
            stat_state <= STAT_MEAN;
            stat_index <= 7'd0;
            sum_accumulator <= {(DATA_WIDTH+9){1'b0}};
          end
        end
        STAT_MEAN:
        begin
          // mean
          if (stat_index < WINDOW_SIZE)
          begin
            sum_accumulator <= sum_accumulator + accel_buffer[stat_index];
            stat_index <= stat_index + 1'b1;
          end
          else
          begin
            // Divide sum by window size for mean
            mean_value <= sum_accumulator / WINDOW_SIZE;
            stat_state <= STAT_VAR;
            stat_index <= 7'd0;
            variance_accumulator <= {(DATA_WIDTH+17){1'b0}};
          end
        end
        STAT_VAR:
        begin
          // Compute variance by summing squared deviations
          if (stat_index < WINDOW_SIZE)
          begin
            wire signed [DATA_WIDTH-1:0] deviation =
            accel_buffer[stat_index] - mean_value;
            variance_accumulator <= variance_accumulator +
            (deviation * deviation);
            stat_index <= stat_index + 1'b1;
          end
          else
          begin
            // Divide by window size to get variance
            variance_value <= variance_accumulator / WINDOW_SIZE;
            stat_state <= STAT_DONE;
          end
        end
        STAT_DONE:
        begin
          stat_state <= STAT_IDLE;
        end
      endcase
    end
  end
  //frequency analysis
  // Periodic motion (walking, running) shows strong peaks
  // Chaotic motion (struggling) has flat spectrum
  reg signed [DATA_WIDTH-1:0] fft_input [0:FFT_SIZE-1];
  wire signed [DATA_WIDTH-1:0] fft_magnitude [0:FFT_SIZE/2];
  wire fft_done;
  reg start_fft;
  // Simple FFT module for motion analysis (smaller size than audio)
  fft_motion_128 #(
                   .DATA_WIDTH(DATA_WIDTH)
                 ) motion_fft (
                   .clk(clk),
                   .rst_n(rst_n),
                   .start(start_fft),
                   .time_data(fft_input),
                   .freq_magnitude(fft_magnitude),
                   .done(fft_done)
                 );
  // Find peak frequency and total energy
  reg signed [DATA_WIDTH-1:0] peak_frequency_val;
  reg [6:0] peak_frequency_bin;
  reg signed [DATA_WIDTH+8:0] total_fft_energy;
  reg [6:0] fft_bin_index;
  reg [1:0] freq_state;
  always @(posedge clk or negedge rst_n)
  begin
    if (!rst_n)
    begin
      freq_state <= 2'd0;
      fft_bin_index <= 7'd0;
      peak_frequency_val <= {DATA_WIDTH{1'b0}};
      peak_frequency_bin <= 7'd0;
      total_fft_energy <= {(DATA_WIDTH+9){1'b0}};
    end
    else if (fft_done)
    begin
      // Scan FFT output to find peak and total energy
      if (fft_bin_index < FFT_SIZE/2)
      begin
        total_fft_energy <= total_fft_energy + fft_magnitude[fft_bin_index];
        if (fft_magnitude[fft_bin_index] > peak_frequency_val)
        begin
          peak_frequency_val <= fft_magnitude[fft_bin_index];
          peak_frequency_bin <= fft_bin_index;
        end
        fft_bin_index <= fft_bin_index + 1'b1;
      end
      else
      begin
        // Convert bin index to actual frequency in Hz
        fft_bin_index <= 7'd0;
      end
    end
  end
  // Tracking how often motion direction changes significantly
  // High rate indicates erratic, unpredictable movement
  reg signed [DATA_WIDTH-1:0] prev_motion_x, prev_motion_y, prev_motion_z;
  reg [7:0] direction_change_count;
  reg [7:0] direction_sample_count;
  // Computing dot product between consecutive motion vectors
  // Dot product < 0 indicates direction reversal
  wire signed [DATA_WIDTH+1:0] dot_product;
  assign dot_product = (accel_x * prev_motion_x) +
         (accel_y * prev_motion_y) +
         (accel_z * prev_motion_z);
  always @(posedge clk or negedge rst_n)
  begin
    if (!rst_n)
    begin
      prev_motion_x <= {DATA_WIDTH{1'b0}};
      prev_motion_y <= {DATA_WIDTH{1'b0}};
      prev_motion_z <= {DATA_WIDTH{1'b0}};
      direction_change_count <= 8'd0;
      direction_sample_count <= 8'd0;
    end
    else if (imu_data_valid)
    begin
      if (dot_product[DATA_WIDTH+1])
      begin // Sign bit set = negative
        direction_change_count <= direction_change_count + 1'b1;
      end
      direction_sample_count <= direction_sample_count + 1'b1;
      prev_motion_x <= accel_x;
      prev_motion_y <= accel_y;
      prev_motion_z <= accel_z;
      // Every 100 samples, computing rate and reset
      if (direction_sample_count == WINDOW_SIZE - 1)
      begin
        // Rate is count normalized by window size
        direction_sample_count <= 8'd0;
        direction_change_count <= 8'd0;
      end
    end
  end
  //Output
  always @(posedge clk or negedge rst_n)
  begin
    if (!rst_n)
    begin
      features_valid <= 1'b0;
    end
    else if (stat_state == STAT_DONE)
    begin
      total_acceleration <= accel_magnitude;
      jerk_magnitude <= jerk_mag;
      motion_variance <= variance_value;
      dominant_frequency <= (peak_frequency_bin * SAMPLE_RATE) / FFT_SIZE;
      direction_change_rate <= (direction_change_count << 8) / WINDOW_SIZE;
      pitch_angle <= fused_pitch;
      roll_angle <= fused_roll;
      features_valid <= 1'b1;
    end
    else
    begin
      features_valid <= 1'b0;
    end
  end
endmodule
