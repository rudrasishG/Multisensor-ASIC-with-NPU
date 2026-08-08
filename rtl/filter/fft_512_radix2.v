module fft_512_radix2 #(
    parameter DATA_WIDTH  = 16,
    parameter FFT_SIZE    = 512,
    parameter FFT_STAGES  = 9   // log2(512)
)(
    input wire clk,
    input wire rst_n,

    // Control
    input wire start,

    // Input data interface
    input wire signed [DATA_WIDTH-1:0] data_in_real,
    input wire signed [DATA_WIDTH-1:0] data_in_imag,
    input wire [8:0]  data_in_addr,
    input wire        data_in_valid,

    // Output data interface
    output reg signed [DATA_WIDTH-1:0] data_out_real,
    output reg signed [DATA_WIDTH-1:0] data_out_imag,
    output reg [8:0]  data_out_addr,
    output reg        data_out_valid,

    // Status
    output reg fft_done,
    output reg busy
);

    // -----------------------------------------------------------------------
    // State machine
    // -----------------------------------------------------------------------
    localparam [2:0]
        STATE_IDLE    = 3'd0,
        STATE_LOAD    = 3'd1,
        STATE_COMPUTE = 3'd2,
        STATE_OUTPUT  = 3'd3;

    reg [2:0] state;

    reg [3:0] stage_count;
    reg [8:0] butterfly_count;
    reg [1:0] bf_cycle;

    // -----------------------------------------------------------------------
    // Ping-pong RAM banks
    // -----------------------------------------------------------------------
    reg signed [DATA_WIDTH-1:0] ram_a_real [0:FFT_SIZE-1];
    reg signed [DATA_WIDTH-1:0] ram_a_imag [0:FFT_SIZE-1];
    reg signed [DATA_WIDTH-1:0] ram_b_real [0:FFT_SIZE-1];
    reg signed [DATA_WIDTH-1:0] ram_b_imag [0:FFT_SIZE-1];

    // -----------------------------------------------------------------------
    // Butterfly I/O registers
    // -----------------------------------------------------------------------
    reg signed [DATA_WIDTH-1:0] bf_in_a_real, bf_in_a_imag;
    reg signed [DATA_WIDTH-1:0] bf_in_b_real, bf_in_b_imag;
    wire signed [DATA_WIDTH-1:0] bf_out_a_real, bf_out_a_imag;
    wire signed [DATA_WIDTH-1:0] bf_out_b_real, bf_out_b_imag;

    wire signed [DATA_WIDTH-1:0] twiddle_real, twiddle_imag;

    // -----------------------------------------------------------------------
    // Address calculation
    // -----------------------------------------------------------------------
    reg  [8:0] addr_a, addr_b;
    wire [8:0] stride     = 9'd1 << stage_count;
    wire [8:0] group_size = 9'd1 << (stage_count + 1);
    wire [8:0] group_num  = butterfly_count >> stage_count;
    wire [8:0] pos_in_grp = butterfly_count & (stride - 9'd1);
    wire [8:0] base_addr  = (group_num * group_size) + pos_in_grp;

    // Bit-reversal function for DIT input ordering
    function [8:0] bit_reverse;
        input [8:0] addr;
        integer j;
        begin
            for (j = 0; j < 9; j = j + 1)
                bit_reverse[j] = addr[8-j];
        end
    endfunction

    // -----------------------------------------------------------------------
    // Sub-module instantiation
    // -----------------------------------------------------------------------
    fft_butterfly #(.DATA_WIDTH(DATA_WIDTH)) butterfly_inst (
        .clk         (clk),
        .rst_n       (rst_n),
        .in_a_real   (bf_in_a_real),
        .in_a_imag   (bf_in_a_imag),
        .in_b_real   (bf_in_b_real),
        .in_b_imag   (bf_in_b_imag),
        .twiddle_real(twiddle_real),
        .twiddle_imag(twiddle_imag),
        .out_a_real  (bf_out_a_real),
        .out_a_imag  (bf_out_a_imag),
        .out_b_real  (bf_out_b_real),
        .out_b_imag  (bf_out_b_imag)
    );

    fft_twiddle_rom #(.DATA_WIDTH(DATA_WIDTH), .FFT_SIZE(FFT_SIZE)) twiddle_rom_inst (
        .stage       (stage_count),
        .index       (butterfly_count),
        .twiddle_real(twiddle_real),
        .twiddle_imag(twiddle_imag)
    );

    // -----------------------------------------------------------------------
    // FIX1: "wire final_ram_select = FFT_STAGES[0]" was declared inside an
    //       always block — illegal in Verilog-2005.  Declare at module scope.
    //       After FFT_STAGES (9, odd) stages the result is in RAM B.
    // -----------------------------------------------------------------------
    wire final_ram_select = FFT_STAGES[0]; // 1 → RAM B holds final result

    // -----------------------------------------------------------------------
    // Main FSM
    // -----------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state           <= STATE_IDLE;
            busy            <= 1'b0;
            fft_done        <= 1'b0;
            stage_count     <= 4'd0;
            butterfly_count <= 9'd0;
            bf_cycle        <= 2'd0;
        end
        else begin
            fft_done <= 1'b0; // default

            case (state)
                STATE_IDLE: begin
                    busy <= 1'b0;
                    if (start) begin
                        state           <= STATE_LOAD;
                        busy            <= 1'b1;
                        stage_count     <= 4'd0;
                        butterfly_count <= 9'd0;
                        bf_cycle        <= 2'd0;
                    end
                end

                STATE_LOAD: begin
                    // External logic writes data_in_valid/data_in_real/imag/addr.
                    // Transition once the last address (511) is seen.
                    if (data_in_valid && data_in_addr == (FFT_SIZE - 1)) begin
                        state           <= STATE_COMPUTE;
                        stage_count     <= 4'd0;
                        butterfly_count <= 9'd0;
                        bf_cycle        <= 2'd0;
                    end
                end

                STATE_COMPUTE: begin
                    // Two-cycle butterfly: cycle 0 = read/feed, cycle 1 = writeback
                    if (bf_cycle == 2'd0) begin
                        bf_cycle <= 2'd1;
                    end
                    else begin
                        bf_cycle <= 2'd0;
                        if (butterfly_count == (FFT_SIZE/2) - 1) begin
                            butterfly_count <= 9'd0;
                            if (stage_count == FFT_STAGES - 1)
                                state <= STATE_OUTPUT;
                            else
                                stage_count <= stage_count + 1'b1;
                        end
                        else begin
                            butterfly_count <= butterfly_count + 1'b1;
                        end
                    end
                end

                // FIX2: Original STATE_OUTPUT lasted exactly 1 clock cycle, so
                //       output_counter only ever reached 1.  The output block now
                //       stays in STATE_OUTPUT until all FFT_SIZE samples are read,
                //       then asserts fft_done for one cycle and returns to IDLE.
                STATE_OUTPUT: begin
                    // fft_done and de-assert of busy happen after last sample
                    if (output_counter == FFT_SIZE - 1) begin
                        state    <= STATE_IDLE;
                        busy     <= 1'b0;
                        fft_done <= 1'b1;
                    end
                end

                default: state <= STATE_IDLE;
            endcase
        end
    end

    // -----------------------------------------------------------------------
    // Input loading with bit-reversal
    // -----------------------------------------------------------------------
    always @(posedge clk) begin
        if (state == STATE_LOAD && data_in_valid) begin
            ram_a_real[bit_reverse(data_in_addr)] <= data_in_real;
            ram_a_imag[bit_reverse(data_in_addr)] <= data_in_imag;
        end
    end

    // -----------------------------------------------------------------------
    // Butterfly read (cycle 0)
    // -----------------------------------------------------------------------
    wire ram_rd_select = stage_count[0]; // even stage → read A; odd → read B

    always @(posedge clk) begin
        addr_a <= base_addr;
        addr_b <= base_addr + stride;

        if (state == STATE_COMPUTE && bf_cycle == 2'd0) begin
            if (!ram_rd_select) begin
                bf_in_a_real <= ram_a_real[base_addr];
                bf_in_a_imag <= ram_a_imag[base_addr];
                bf_in_b_real <= ram_a_real[base_addr + stride];
                bf_in_b_imag <= ram_a_imag[base_addr + stride];
            end
            else begin
                bf_in_a_real <= ram_b_real[base_addr];
                bf_in_a_imag <= ram_b_imag[base_addr];
                bf_in_b_real <= ram_b_real[base_addr + stride];
                bf_in_b_imag <= ram_b_imag[base_addr + stride];
            end
        end
    end

    // -----------------------------------------------------------------------
    // Butterfly writeback (cycle 1) — delayed addresses align with BF output
    // -----------------------------------------------------------------------
    wire   ram_wr_select = ~ram_rd_select;
    reg [8:0] addr_a_dly, addr_b_dly;

    always @(posedge clk) begin
        addr_a_dly <= addr_a;
        addr_b_dly <= addr_b;
    end

    always @(posedge clk) begin
        if (state == STATE_COMPUTE && bf_cycle == 2'd1) begin
            if (!ram_wr_select) begin
                ram_a_real[addr_a_dly] <= bf_out_a_real;
                ram_a_imag[addr_a_dly] <= bf_out_a_imag;
                ram_a_real[addr_b_dly] <= bf_out_b_real;
                ram_a_imag[addr_b_dly] <= bf_out_b_imag;
            end
            else begin
                ram_b_real[addr_a_dly] <= bf_out_a_real;
                ram_b_imag[addr_a_dly] <= bf_out_a_imag;
                ram_b_real[addr_b_dly] <= bf_out_b_real;
                ram_b_imag[addr_b_dly] <= bf_out_b_imag;
            end
        end
    end

    // -----------------------------------------------------------------------
    // Output streaming
    // -----------------------------------------------------------------------
    reg [8:0] output_counter;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            output_counter <= 9'd0;
            data_out_valid <= 1'b0;
            data_out_real  <= {DATA_WIDTH{1'b0}};
            data_out_imag  <= {DATA_WIDTH{1'b0}};
            data_out_addr  <= 9'd0;
        end
        else begin
            if (state == STATE_OUTPUT) begin
                // Select result RAM (9 stages → final result in RAM B)
                if (!final_ram_select) begin
                    data_out_real <= ram_a_real[output_counter];
                    data_out_imag <= ram_a_imag[output_counter];
                end
                else begin
                    data_out_real <= ram_b_real[output_counter];
                    data_out_imag <= ram_b_imag[output_counter];
                end
                data_out_addr  <= output_counter;
                data_out_valid <= 1'b1;

                // FIX2 cont: advance counter every cycle while in STATE_OUTPUT
                if (output_counter == FFT_SIZE - 1)
                    output_counter <= 9'd0;
                else
                    output_counter <= output_counter + 1'b1;
            end
            else begin
                data_out_valid <= 1'b0;
                output_counter <= 9'd0;
            end
        end
    end

endmodule
