module i2c_imu_interface #(
    parameter sys_clk_freq = 50_000_000,
    parameter i2c_clk_freq = 400_000,
    parameter slave_addr   = 7'h68
)(
    // system interface
    input  wire       clk,
    input  wire       rst_n,
    // control interface
    input  wire       start,
    input  wire       rw,
    input  wire [7:0] reg_addr,
    input  wire [7:0] write_data,
    input  wire [3:0] burst_length,
    output reg  [7:0] read_data,
    output reg        data_valid,
    output reg        busy,
    output reg        error,
    //physical I2C lines
    inout  wire       sda,
    output reg        scl
);

// Open-drain SDA model

wire sda_in;
assign sda    = sda_oe ? sda_out : 1'bz;
assign sda_in = sda;

// I2C bclock generation  (4 phases per bit period)
localparam clk_divide = (sys_clk_freq / (4 * i2c_clk_freq));

reg [15:0] clk_counter;
reg  [1:0] clk_phase;
wire       clk_tick;

assign clk_tick = (clk_counter == clk_divide - 1);

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        clk_counter <= 16'b0;
        clk_phase   <= 2'b00;
    end
    else if (busy) begin
        if (clk_tick) begin
            clk_counter <= 16'b0;
            clk_phase   <= clk_phase + 1'b1;
        end
        else begin
            clk_counter <= clk_counter + 1'b1;
        end
    end
    else begin
        clk_counter <= 16'b0;
        clk_phase   <= 2'b00;
    end
end


// State encoding
localparam [3:0]
    state_idle       = 4'd0,
    state_start      = 4'd1,
    state_addr_w     = 4'd2,
    state_addr_ack   = 4'd3,
    state_reg_addr   = 4'd4,
    state_reg_ack    = 4'd5,
    state_write_data = 4'd6,
    state_write_ack  = 4'd7,
    state_restart    = 4'd8,
    state_addr_r     = 4'd9,
    state_addr_r_ack = 4'd10,   // fix 4: dedicated ack after state_addr_r
    state_read_data  = 4'd11,   // (indices shifted by 1 from original)
    state_read_ack   = 4'd12,
    state_stop       = 4'd13,
    state_error      = 4'd14;

reg [3:0] state;
reg [3:0] next_state;

reg [3:0] bit_index;
reg [3:0] byte_count;
reg [7:0] shift_reg;
reg       sda_out;
reg       sda_oe;
reg       rw_bit;
reg [7:0] reg_addr_buf;
reg [3:0] burst_len_buf;
// Sequential state register + busy / buffered inputs

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state         <= state_idle;
        busy          <= 1'b0;
        error         <= 1'b0;
        rw_bit        <= 1'b0;
        reg_addr_buf  <= 8'h00;
        burst_len_buf <= 4'd0;
    end
    else begin
        state <= next_state;
        // fix 3
        busy  <= (next_state != state_idle);

        if (start && !busy) begin
            rw_bit        <= rw;
            reg_addr_buf  <= reg_addr;
            burst_len_buf <= burst_length;
            error         <= 1'b0;   // fix 13: clear error on new txn
        end
    end
end

// Combinational next-state logic

always @(*) begin
    next_state = state;
    case (state)
        state_idle: begin
            if (start)
                next_state = state_start;
        end

        state_start: begin
            if (clk_tick && clk_phase == 2'b11)
                next_state = state_addr_w;
        end

        state_addr_w: begin
            if (clk_tick && clk_phase == 2'b11 && bit_index == 4'd8)
                next_state = state_addr_ack;
        end

        state_addr_ack: begin
            if (clk_tick && clk_phase == 2'b11) begin
                if (sda_in == 1'b0)
                    next_state = state_reg_addr;
                else
                    next_state = state_error;
            end
        end

        state_reg_addr: begin
            if (clk_tick && clk_phase == 2'b11 && bit_index == 4'd8)
                next_state = state_reg_ack;
        end

        state_reg_ack: begin
            if (clk_tick && clk_phase == 2'b11) begin
                if (sda_in == 1'b0) begin
                    if (rw_bit)
                        next_state = state_restart;
                    else
                        next_state = state_write_data;
                end
                else begin
                    next_state = state_error;
                end
            end
        end

        state_write_data: begin
            if (clk_tick && clk_phase == 2'b11 && bit_index == 4'd8)
                next_state = state_write_ack;
        end

        state_write_ack: begin
            if (clk_tick && clk_phase == 2'b11)
                next_state = state_stop;
        end

        state_restart: begin
            if (clk_tick && clk_phase == 2'b11)
                next_state = state_addr_r;
        end

        state_addr_r: begin
            if (clk_tick && clk_phase == 2'b11 && bit_index == 4'd8)
                next_state = state_addr_r_ack;
        end

        state_addr_r_ack: begin
            if (clk_tick && clk_phase == 2'b11) begin
                if (sda_in == 1'b0)
                    next_state = state_read_data;
                else
                    next_state = state_error;
            end
        end

        state_read_data: begin
            if (clk_tick && clk_phase == 2'b11 && bit_index == 4'd8)
                next_state = state_read_ack;
        end

        state_read_ack: begin
            if (clk_tick && clk_phase == 2'b11) begin
                if (byte_count < burst_len_buf)
                    next_state = state_read_data;
                else
                    next_state = state_stop;
            end
        end

        state_stop: begin
            if (clk_tick && clk_phase == 2'b11)
                next_state = state_idle;
        end

        state_error: begin
            next_state = state_idle;
        end

        default: next_state = state_idle;
    endcase
end

// I2C signal generation (sequential output logic)

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        scl        <= 1'b1;
        sda_out    <= 1'b1;
        sda_oe     <= 1'b0;
        bit_index  <= 4'd0;
        byte_count <= 4'd0;
        shift_reg  <= 8'h00;
        read_data  <= 8'h00;
        data_valid <= 1'b0;
        error      <= 1'b0;
    end
    else begin
        data_valid <= 1'b0;  // single-cycle pulse default

        case (state)
            state_idle: begin
                scl        <= 1'b1;
                sda_out    <= 1'b1;
                sda_oe     <= 1'b0;
                bit_index  <= 4'd0;
                byte_count <= 4'd0;
            end

            // Start: SDA falls while SCL is high
            state_start: begin
                sda_oe <= 1'b1;
                if (clk_tick) begin
                    case (clk_phase)
                        2'b00: begin scl <= 1'b1; sda_out <= 1'b1; end
                        2'b01: begin scl <= 1'b1; sda_out <= 1'b0; end  // SDA falls
                        2'b10: begin scl <= 1'b0; sda_out <= 1'b0; end  // SCL falls
                        2'b11: begin
                            scl       <= 1'b0;
                            sda_out   <= 1'b0;
                            shift_reg <= {slave_addr, 1'b0};  // addr + W
                            bit_index <= 4'd0;
                        end
                    endcase
                end
            end


            // Send 8 bits: slave address + write bit
      
            state_addr_w: begin
                sda_oe <= 1'b1;
                if (clk_tick) begin
                    case (clk_phase)
                        2'b00: begin
                            scl     <= 1'b0;
                            sda_out <= shift_reg[7 - bit_index]; // fix 12
                        end
                        2'b01: begin scl <= 1'b1; end
                        2'b10: begin scl <= 1'b1; end
                        2'b11: begin
                            scl       <= 1'b0;
                            bit_index <= bit_index + 1'b1;
                        end
                    endcase
                end
            end


            // Receive ACK from slave
            state_addr_ack: begin
                sda_oe <= 1'b0;
                if (clk_tick) begin
                    case (clk_phase)
                        2'b00: begin scl <= 1'b0; end
                        2'b01: begin scl <= 1'b1; end
                        2'b10: begin
                            scl <= 1'b1;
                            if (sda_in == 1'b1)
                                error <= 1'b1;
                        end
                        2'b11: begin
                            scl       <= 1'b0;
                            shift_reg <= reg_addr_buf;
                            bit_index <= 4'd0;
                        end
                    endcase
                end
            end

            // Send 8-bit register address
            state_reg_addr: begin
                sda_oe <= 1'b1;
                if (clk_tick) begin
                    case (clk_phase)
                        2'b00: begin
                            scl     <= 1'b0;
                            sda_out <= shift_reg[7 - bit_index];
                        end
                        2'b01: begin scl <= 1'b1; end
                        2'b10: begin scl <= 1'b1; end
                        2'b11: begin
                            scl       <= 1'b0;
                            bit_index <= bit_index + 1'b1;
                        end
                    endcase
                end
            end

            // Receive ACK after register address
            state_reg_ack: begin
                sda_oe <= 1'b0;
                if (clk_tick) begin
                    case (clk_phase)
                        2'b00: begin scl <= 1'b0; end
                        2'b01: begin scl <= 1'b1; end
                        2'b10: begin
                            scl <= 1'b1;
                            if (sda_in == 1'b1)
                                error <= 1'b1;
                        end
                        2'b11: begin
                            scl <= 1'b0;
                            if (rw_bit) begin
                                bit_index <= 4'd0;
                            end else begin
                                shift_reg <= write_data;
                                bit_index <= 4'd0;
                            end
                        end
                    endcase
                end
            end 

            // Send write data byte 
            state_write_data: begin
                sda_oe <= 1'b1;
                if (clk_tick) begin
                    case (clk_phase)
                        2'b00: begin
                            scl     <= 1'b0;
                            sda_out <= shift_reg[7 - bit_index];
                        end
                        2'b01: begin scl <= 1'b1; end
                        2'b10: begin scl <= 1'b1; end
                        2'b11: begin
                            scl       <= 1'b0;
                            bit_index <= bit_index + 1'b1;  // fix 9: clk_tcik → clk_tick
                        end
                    endcase
                end
            end

            // Receive write ACK
            state_write_ack: begin
                sda_oe <= 1'b0;
                if (clk_tick) begin
                    case (clk_phase)
                        2'b00: begin scl <= 1'b0; end
                        2'b01: begin scl <= 1'b1; end
                        2'b10: begin
                            scl <= 1'b1;
                            if (sda_in == 1'b1)
                                error <= 1'b1;
                        end
                        2'b11: begin scl <= 1'b0; end
                    endcase
                end
            end

            state_restart: begin
                sda_oe <= 1'b1;
                if (clk_tick) begin
                    case (clk_phase)
                        2'b00: begin scl <= 1'b0; sda_out <= 1'b1; end  // release SDA
                        2'b01: begin scl <= 1'b1; sda_out <= 1'b1; end  // SCL high
                        2'b10: begin scl <= 1'b1; sda_out <= 1'b0; end  // SDA falls = restart
                        2'b11: begin
                            scl       <= 1'b0;                           // SCL falls
                            sda_out   <= 1'b0;
                            shift_reg <= {slave_addr, 1'b1};             // addr + R
                            bit_index <= 4'd0;
                        end
                    endcase
                end
            end

            // Send slave address + read bit
            state_addr_r: begin
                sda_oe <= 1'b1;
                if (clk_tick) begin
                    case (clk_phase)
                        2'b00: begin
                            scl     <= 1'b0;
                            sda_out <= shift_reg[7 - bit_index];
                        end
                        2'b01: begin scl <= 1'b1; end
                        2'b10: begin scl <= 1'b1; end
                        2'b11: begin
                            scl       <= 1'b0;
                            bit_index <= bit_index + 1'b1;
                        end
                    endcase
                end
            end

            // fix 4: receive ACK after addr_r
            state_addr_r_ack: begin
                sda_oe <= 1'b0;
                if (clk_tick) begin
                    case (clk_phase)
                        2'b00: begin scl <= 1'b0; end
                        2'b01: begin scl <= 1'b1; end
                        2'b10: begin
                            scl <= 1'b1;
                            if (sda_in == 1'b1)
                                error <= 1'b1;
                        end
                        2'b11: begin
                            scl        <= 1'b0;
                            bit_index  <= 4'd0;
                            byte_count <= 4'd1;  // fix 6 location: init here
                        end
                    endcase
                end
            end

            // Receive data byte from slave
            state_read_data: begin
                sda_oe <= 1'b0;
                if (clk_tick) begin
                    case (clk_phase)
                        2'b00: begin scl <= 1'b0; end
                        2'b01: begin scl <= 1'b1; end
                        2'b10: begin
                            scl       <= 1'b1;
                            shift_reg <= {shift_reg[6:0], sda_in};
                        end
                        2'b11: begin
                            scl       <= 1'b0;
                            bit_index <= bit_index + 1'b1;
                            if (bit_index == 4'd7) begin
                                read_data  <= {shift_reg[6:0], sda_in};
                                data_valid <= 1'b1;
                                bit_index  <= 4'd0;
                            end
                        end
                    endcase
                end
            end

            // Send ACK (more bytes) or NACK (last byte)
            state_read_ack: begin
                sda_oe <= 1'b1;
                if (clk_tick) begin
                    case (clk_phase)
                        2'b00: begin
                            scl     <= 1'b0;
                            sda_out <= (byte_count < burst_len_buf) ? 1'b0 : 1'b1;
                        end
                        2'b01: begin scl <= 1'b1; end
                        2'b10: begin scl <= 1'b1; end
                        2'b11: begin
                            scl <= 1'b0;
                            if (byte_count < burst_len_buf) begin   // fix 10: burst_len_buf
                                byte_count <= byte_count + 1'b1;
                                bit_index  <= 4'd0;
                            end
                        end
                    endcase
                end
            end


            // Stop condition
            state_stop: begin
                sda_oe <= 1'b1;
                if (clk_tick) begin
                    case (clk_phase)
                        2'b00: begin scl <= 1'b0; sda_out <= 1'b0; end
                        2'b01: begin scl <= 1'b1; sda_out <= 1'b0; end
                        2'b10: begin scl <= 1'b1; sda_out <= 1'b1; end
                        2'b11: begin scl <= 1'b1; sda_out <= 1'b1; end  // fix 11
                    endcase
                end
            end

            state_error: begin
                scl     <= 1'b1;
                sda_out <= 1'b1;
                sda_oe  <= 1'b0;
                error   <= 1'b1;
            end

            default: begin
                scl     <= 1'b1;
                sda_out <= 1'b1;
                sda_oe  <= 1'b0;
            end
        endcase
    end
end

endmodule