//==============================================================================
// Module: fft_512_radix2
// Description: Complete 512-point sequential radix-2 FFT processor
//
// Implements a complete 512-point Fast Fourier Transform using the Cooley-Tukey
// radix-2 decimation-in-time algorithm. The implementation is sequential, reusing
// a single butterfly computation unit across multiple clock cycles to minimize
// logic resource usage.
//
// The FFT operates on complex input data (real and imaginary components) and
// produces complex output in frequency domain. For audio applications, real input
// is zero-padded in the imaginary component.
//
// Computation takes log2(512) = 9 stages with 256 butterflies per stage, totaling
// approximately 2,300 butterfly operations. With 2 cycles per butterfly (1 for
// compute, 1 for memory), total time is about 4,600 clock cycles.
//
// Design Notes:
// - Uses dual-port RAM for ping-pong buffering between stages
// - Twiddle factors computed via ROM lookup with symmetry optimization
// - Bit-reversal addressing handled during input loading
// - Fixed-point arithmetic with per-stage scaling to prevent overflow
// - Fully pipelined butterfly unit provides 1-cycle throughput
//
// Author: Generated for Sensor Fusion ASIC Project
// Date: January 2026
//==============================================================================

module fft_512_radix2 #(
    parameter DATA_WIDTH = 16,
    parameter FFT_SIZE = 512,
    parameter FFT_STAGES = 9  // log2(512)
)(
    input wire clk,
    input wire rst_n,
    
    // Control
    input wire start,                                  // Start FFT computation
    
    // Input data interface
    input wire signed [DATA_WIDTH-1:0] data_in_real,
    input wire signed [DATA_WIDTH-1:0] data_in_imag,
    input wire [8:0] data_in_addr,                     // 0-511
    input wire data_in_valid,
    
    // Output data interface
    output reg signed [DATA_WIDTH-1:0] data_out_real,
    output reg signed [DATA_WIDTH-1:0] data_out_imag,
    output reg [8:0] data_out_addr,
    output reg data_out_valid,
    
    // Status
    output reg fft_done,
    output reg busy
);

    //==========================================================================
    // State Machine Definitions
    //==========================================================================
    
    localparam [2:0]
        STATE_IDLE    = 3'd0,
        STATE_LOAD    = 3'd1,
        STATE_COMPUTE = 3'd2,
        STATE_OUTPUT  = 3'd3;
    
    reg [2:0] state;
    
    //==========================================================================
    // Stage and Butterfly Control
    //==========================================================================
    
    reg [3:0] stage_count;        // 0-8 for 9 stages
    reg [8:0] butterfly_count;    // 0-255 butterflies per stage
    reg [1:0] bf_cycle;           // Butterfly computation cycle (0-1)
    
    //==========================================================================
    // Dual-Port RAM for Ping-Pong Buffering
    // Bank A and Bank B alternate between read and write each stage
    //==========================================================================
    
    reg signed [DATA_WIDTH-1:0] ram_a_real [0:FFT_SIZE-1];
    reg signed [DATA_WIDTH-1:0] ram_a_imag [0:FFT_SIZE-1];
    reg signed [DATA_WIDTH-1:0] ram_b_real [0:FFT_SIZE-1];
    reg signed [DATA_WIDTH-1:0] ram_b_imag [0:FFT_SIZE-1];
    
    //==========================================================================
    // Butterfly Computation Signals
    //==========================================================================
    
    reg signed [DATA_WIDTH-1:0] bf_in_a_real, bf_in_a_imag;
    reg signed [DATA_WIDTH-1:0] bf_in_b_real, bf_in_b_imag;
    wire signed [DATA_WIDTH-1:0] bf_out_a_real, bf_out_a_imag;
    wire signed [DATA_WIDTH-1:0] bf_out_b_real, bf_out_b_imag;
    
    // Twiddle factor for current butterfly
    wire signed [DATA_WIDTH-1:0] twiddle_real, twiddle_imag;
    
    // Address calculation signals
    reg [8:0] addr_a;       // Address for top input of butterfly
    reg [8:0] addr_b;       // Address for bottom input of butterfly
    wire [8:0] stride;      // Distance between butterfly pairs
    wire [8:0] group_size;  // Size of butterfly groups
    
    //==========================================================================
    // Butterfly Addressing Calculation
    // For stage s:
    //   - Group size = 2^(s+1)
    //   - Stride (distance between pairs) = 2^s
    //   - Number of groups = 512 / group_size
    //   - Butterflies per group = stride
    //==========================================================================
    
    assign stride = 9'd1 << stage_count;           // 2^s
    assign group_size = 9'd1 << (stage_count + 1); // 2^(s+1)
    
    // Calculate which group and position within group
    wire [8:0] group_num = butterfly_count >> stage_count;  // butterfly_count / stride
    wire [8:0] pos_in_group = butterfly_count & (stride - 9'd1);  // butterfly_count % stride
    
    // Calculate butterfly pair addresses
    wire [8:0] base_addr = (group_num * group_size) + pos_in_group;
    assign addr_a = base_addr;
    assign addr_b = base_addr + stride;
    
    //==========================================================================
    // Bit Reversal Function for Input Ordering
    // FFT requires bit-reversed input order for in-place computation
    //==========================================================================
    
    function [8:0] bit_reverse;
        input [8:0] addr;
        integer i;
        begin
            for (i = 0; i < 9; i = i + 1) begin
                bit_reverse[i] = addr[8-i];
            end
        end
    endfunction
    
    //==========================================================================
    // Instantiate Butterfly Computation Unit
    //==========================================================================
    
    fft_butterfly #(
        .DATA_WIDTH(DATA_WIDTH)
    ) butterfly_inst (
        .clk(clk),
        .rst_n(rst_n),
        .in_a_real(bf_in_a_real),
        .in_a_imag(bf_in_a_imag),
        .in_b_real(bf_in_b_real),
        .in_b_imag(bf_in_b_imag),
        .twiddle_real(twiddle_real),
        .twiddle_imag(twiddle_imag),
        .out_a_real(bf_out_a_real),
        .out_a_imag(bf_out_a_imag),
        .out_b_real(bf_out_b_real),
        .out_b_imag(bf_out_b_imag)
    );
    
    //==========================================================================
    // Instantiate Twiddle Factor ROM
    //==========================================================================
    
    fft_twiddle_rom #(
        .DATA_WIDTH(DATA_WIDTH),
        .FFT_SIZE(FFT_SIZE)
    ) twiddle_rom_inst (
        .stage(stage_count),
        .index(butterfly_count),
        .twiddle_real(twiddle_real),
        .twiddle_imag(twiddle_imag)
    );
    
    //==========================================================================
    // Main State Machine Control
    //==========================================================================
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= STATE_IDLE;
            busy <= 1'b0;
            fft_done <= 1'b0;
            stage_count <= 4'd0;
            butterfly_count <= 9'd0;
            bf_cycle <= 2'd0;
        end
        else begin
            // Default
            fft_done <= 1'b0;
            
            case (state)
                STATE_IDLE: begin
                    busy <= 1'b0;
                    if (start) begin
                        state <= STATE_LOAD;
                        busy <= 1'b1;
                        stage_count <= 4'd0;
                        butterfly_count <= 9'd0;
                        bf_cycle <= 2'd0;
                    end
                end
                
                STATE_LOAD: begin
                    // Wait for all input data to be loaded
                    // External entity loads data with bit-reversed addressing
                    if (data_in_valid && data_in_addr == (FFT_SIZE - 1)) begin
                        state <= STATE_COMPUTE;
                        stage_count <= 4'd0;
                        butterfly_count <= 9'd0;
                        bf_cycle <= 2'd0;
                    end
                end
                
                STATE_COMPUTE: begin
                    // Butterfly computation cycle control
                    // Cycle 0: Read RAM, feed butterfly
                    // Cycle 1: Write butterfly results back to RAM
                    
                    if (bf_cycle == 2'd0) begin
                        bf_cycle <= 2'd1;
                    end
                    else begin
                        bf_cycle <= 2'd0;
                        
                        // Move to next butterfly
                        if (butterfly_count == (FFT_SIZE / 2) - 1) begin
                            // Stage complete
                            butterfly_count <= 9'd0;
                            
                            if (stage_count == FFT_STAGES - 1) begin
                                // All stages complete
                                state <= STATE_OUTPUT;
                            end
                            else begin
                                stage_count <= stage_count + 1'b1;
                            end
                        end
                        else begin
                            butterfly_count <= butterfly_count + 1'b1;
                        end
                    end
                end
                
                STATE_OUTPUT: begin
                    // Output complete, signal done
                    state <= STATE_IDLE;
                    busy <= 1'b0;
                    fft_done <= 1'b1;
                end
                
                default: state <= STATE_IDLE;
            endcase
        end
    end
    
    //==========================================================================
    // Input Data Loading with Bit Reversal
    //==========================================================================
    
    always @(posedge clk) begin
        if (state == STATE_LOAD && data_in_valid) begin
            // Load input data into RAM A with bit-reversed addressing
            ram_a_real[bit_reverse(data_in_addr)] <= data_in_real;
            ram_a_imag[bit_reverse(data_in_addr)] <= data_in_imag;
        end
    end
    
    //==========================================================================
    // RAM Read Logic for Butterfly Inputs
    //==========================================================================
    
    // Determine which RAM bank to read from based on stage parity
    wire ram_rd_select = stage_count[0];  // 0=read A, 1=read B
    
    always @(posedge clk) begin
        if (state == STATE_COMPUTE && bf_cycle == 2'd0) begin
            // Read data for butterfly computation
            if (ram_rd_select == 1'b0) begin
                // Read from RAM A
                bf_in_a_real <= ram_a_real[addr_a];
                bf_in_a_imag <= ram_a_imag[addr_a];
                bf_in_b_real <= ram_a_real[addr_b];
                bf_in_b_imag <= ram_a_imag[addr_b];
            end
            else begin
                // Read from RAM B
                bf_in_a_real <= ram_b_real[addr_a];
                bf_in_a_imag <= ram_b_imag[addr_a];
                bf_in_b_real <= ram_b_real[addr_b];
                bf_in_b_imag <= ram_b_imag[addr_b];
            end
        end
    end
    
    //==========================================================================
    // RAM Write Logic for Butterfly Outputs
    //==========================================================================
    
    // Write to opposite bank from read
    wire ram_wr_select = ~ram_rd_select;  // 0=write A, 1=write B
    
    // Registered addresses for write (delayed by butterfly latency)
    reg [8:0] addr_a_dly;
    reg [8:0] addr_b_dly;
    
    always @(posedge clk) begin
        // Delay addresses to align with butterfly output
        addr_a_dly <= addr_a;
        addr_b_dly <= addr_b;
    end
    
    always @(posedge clk) begin
        if (state == STATE_COMPUTE && bf_cycle == 2'd1) begin
            // Write butterfly results to RAM
            if (ram_wr_select == 1'b0) begin
                // Write to RAM A
                ram_a_real[addr_a_dly] <= bf_out_a_real;
                ram_a_imag[addr_a_dly] <= bf_out_a_imag;
                ram_a_real[addr_b_dly] <= bf_out_b_real;
                ram_a_imag[addr_b_dly] <= bf_out_b_imag;
            end
            else begin
                // Write to RAM B
                ram_b_real[addr_a_dly] <= bf_out_a_real;
                ram_b_imag[addr_a_dly] <= bf_out_a_imag;
                ram_b_real[addr_b_dly] <= bf_out_b_real;
                ram_b_imag[addr_b_dly] <= bf_out_b_imag;
            end
        end
    end
    
    //==========================================================================
    // Output Data Read
    // After final stage, results are in one of the RAM banks
    //==========================================================================
    
    reg [8:0] output_counter;
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            output_counter <= 9'd0;
            data_out_valid <= 1'b0;
            data_out_real <= {DATA_WIDTH{1'b0}};
            data_out_imag <= {DATA_WIDTH{1'b0}};
            data_out_addr <= 9'd0;
        end
        else begin
            if (state == STATE_OUTPUT) begin
                // Determine which RAM has final results
                // After 9 stages (odd), results are in RAM B
                wire final_ram_select = FFT_STAGES[0];
                
                if (final_ram_select == 1'b0) begin
                    data_out_real <= ram_a_real[output_counter];
                    data_out_imag <= ram_a_imag[output_counter];
                end
                else begin
                    data_out_real <= ram_b_real[output_counter];
                    data_out_imag <= ram_b_imag[output_counter];
                end
                
                data_out_addr <= output_counter;
                data_out_valid <= 1'b1;
                
                if (output_counter == FFT_SIZE - 1) begin
                    output_counter <= 9'd0;
                end
                else begin
                    output_counter <= output_counter + 1'b1;
                end
            end
            else begin
                data_out_valid <= 1'b0;
                output_counter <= 9'd0;
            end
        end
    end

endmodule
