// =============================================================================
// tb_i2c_imu_interface.v  –  Plain Verilog Testbench
// Tests: single write, single read, burst read (4 bytes), NACK error handling
// =============================================================================
`timescale 1ns/1ps

module tb_i2c;

// Parameters
localparam SYS_CLK   = 50_000_000;
localparam I2C_CLK   = 400_000;
localparam SLAVE_ADDR = 7'h68;
localparam CLK_PERIOD = 20;  // 50 MHz → 20 ns


// DUT ports

reg        clk;
reg        rst_n;
reg        start;
reg        rw;
reg  [7:0] reg_addr;
reg  [7:0] write_data;
reg  [3:0] burst_length;
wire [7:0] read_data;
wire       data_valid;
wire       busy;
wire       error;
wire       scl;
tri1       sda;   // pull-up on open-drain bus

// Slave model drives sda low when needed
reg  sda_slave_drive;
assign sda = sda_slave_drive ? 1'b0 : 1'bz;

// DUT instantiation

i2c_imu_interface #(
    .sys_clk_freq (SYS_CLK),
    .i2c_clk_freq (I2C_CLK),
    .slave_addr   (SLAVE_ADDR)
) dut (
    .clk          (clk),
    .rst_n        (rst_n),
    .start        (start),
    .rw           (rw),
    .reg_addr     (reg_addr),
    .write_data   (write_data),
    .burst_length (burst_length),
    .read_data    (read_data),
    .data_valid   (data_valid),
    .busy         (busy),
    .error        (error),
    .sda          (sda),
    .scl          (scl)
);


// Clock generation

initial clk = 0;
always #(CLK_PERIOD/2) clk = ~clk;

// Bit-period helpers (one I2C bit = 4 clk_phases × clk_divide cycles)
localparam CLK_DIV    = SYS_CLK / (4 * I2C_CLK);
localparam BIT_CYCLES = 4 * CLK_DIV;

// Slave ACK model
//   Monitors SCL and drives SDA low during the 9th bit window (ACK slot)

integer bit_cnt;
reg     slave_ack_enable;
reg     slave_inject_nack;  // set to force a NACK on next ack slot
reg [7:0] slave_tx_byte;    // byte slave returns during reads

// Simple slave FSM: counts SCL rising edges, drives ACK on 9th
initial begin
    sda_slave_drive   = 1'b0;
    slave_ack_enable  = 1'b0;
    slave_inject_nack = 1'b0;
    slave_tx_byte     = 8'hA5;
    bit_cnt           = 0;
end

reg scl_prev;
always @(posedge clk) begin
    scl_prev <= scl;

    if (!rst_n) begin
        bit_cnt         <= 0;
        sda_slave_drive <= 1'b0;
    end
    else begin
        // Detect SCL rising edge
        if (scl && !scl_prev && slave_ack_enable) begin
            bit_cnt <= bit_cnt + 1;
        end
        if (bit_cnt == 9 && slave_ack_enable) begin
            sda_slave_drive <= slave_inject_nack ? 1'b0 : 1'b1;
            // Note: sda_slave_drive=1 means "driving 0 via assign", =0 means release
            // Invert semantics: slave pulls low to ACK
        end
        else begin
            sda_slave_drive <= 1'b0;
        end

        // Reset counter after ACK phase (SCL falls after 9th bit)
        if (bit_cnt >= 10) begin
            bit_cnt <= 0;
        end
    end
end

// Corrected slave model: 1 = pull sda low (ACK), 0 = release (NACK/idle)
// Re-declare as a cleaner task-driven approach below.

// Waveform dump

initial begin
    $dumpfile("tb_i2c.vcd");
    $dumpvars(0, tb_i2c);
end


// Task: wait for busy to deassert (transaction complete)

task wait_done;
    integer timeout;
    begin
        timeout = 0;
        while (busy && timeout < 2_000_000) begin
            @(posedge clk);
            timeout = timeout + 1;
        end
        if (timeout >= 2_000_000)
            $display("[ERROR] Timeout waiting for busy=0 at time %0t", $time);
    end
endtask


// Task: issue a single write transaction

task do_write;
    input [7:0] addr;
    input [7:0] data;
    begin
        @(negedge clk);
        reg_addr     = addr;
        write_data   = data;
        burst_length = 4'd1;
        rw           = 1'b0;
        start        = 1'b1;
        @(negedge clk);
        start = 1'b0;
        wait_done;
    end
endtask


// Task: issue a read transaction

task do_read;
    input [7:0]  addr;
    input [3:0]  len;
    begin
        @(negedge clk);
        reg_addr     = addr;
        burst_length = len;
        rw           = 1'b1;
        start        = 1'b1;
        @(negedge clk);
        start = 1'b0;
        wait_done;
    end
endtask


// Simple ACK injector – drives SDA low for the ACK slot of each byte
// (Proper open-drain slave behaviour on the 9th SCL pulse)

// Counts SCL rising edges independently; during the 9th pulse drives sda low
reg [3:0] scl_edge_cnt;
reg       in_ack_slot;

initial begin
    scl_edge_cnt    = 0;
    in_ack_slot     = 0;
    sda_slave_drive = 0;
end

// Detect SCL edges via dedicated always block
always @(posedge scl or negedge rst_n) begin
    if (!rst_n) begin
        scl_edge_cnt    <= 0;
        in_ack_slot     <= 0;
        sda_slave_drive <= 0;
    end
    else begin
        scl_edge_cnt <= scl_edge_cnt + 1;
        if (scl_edge_cnt == 8) begin  // 9th rising edge (0-indexed)
            in_ack_slot     <= 1;
            sda_slave_drive <= ~slave_inject_nack;  // pull low = ACK
        end
        else begin
            in_ack_slot     <= 0;
            sda_slave_drive <= 0;
        end
        if (scl_edge_cnt == 9)
            scl_edge_cnt <= 0;
    end
end


// Scoreboard / checker

integer pass_count, fail_count;
initial begin
    pass_count = 0;
    fail_count = 0;
end

task check;
    input        condition;
    input [63:0] test_id;
    input [127:0] msg;
    begin
        if (condition) begin
            $display("[PASS] Test %0d: %s", test_id, msg);
            pass_count = pass_count + 1;
        end
        else begin
            $display("[FAIL] Test %0d: %s  (time=%0t)", test_id, msg, $time);
            fail_count = fail_count + 1;
        end
    end
endtask


// Main stimulus

integer i;
reg [7:0] captured_data [0:3];
integer   data_idx;

initial begin
    // Initialise
    rst_n        = 1'b0;
    start        = 1'b0;
    rw           = 1'b0;
    reg_addr     = 8'h00;
    write_data   = 8'h00;
    burst_length = 4'd1;
    slave_inject_nack = 1'b0;
    data_idx     = 0;

    repeat(10) @(posedge clk);
    rst_n = 1'b1;
    repeat(5)  @(posedge clk);


    // TEST 1: Single write – expect no error, busy returns to 0

    $display("\n--- TEST 1: Single Write ---");
    do_write(8'h3B, 8'hDE);
    #100;
    check(!error,   1, "No error after write");
    check(!busy,    1, "Bus idle after write");


    // TEST 2: Single read
  
    $display("\n--- TEST 2: Single Read ---");
    slave_tx_byte = 8'hA5;
    data_idx = 0;
    do_read(8'h3B, 4'd1);
    #100;
    check(!error,  2, "No error after single read");
    check(!busy,   2, "Bus idle after single read");

  
    // TEST 3: Burst read – 4 bytes
   
    $display("\n--- TEST 3: Burst Read (4 bytes) ---");
    slave_tx_byte = 8'hC3;
    data_idx = 0;
    do_read(8'h3B, 4'd4);
    #200;
    check(!error, 3, "No error after burst read");
    check(!busy,  3, "Bus idle after burst read");


    // TEST 4: NACK injection – expect error flag
 
    $display("\n--- TEST 4: NACK Error Injection ---");
    slave_inject_nack = 1'b1;
    do_write(8'h75, 8'hFF);
    #200;
    check(error, 4, "Error flag set on NACK");
    slave_inject_nack = 1'b0;

  
    // TEST 5: Back-to-back writes
   
    $display("\n--- TEST 5: Back-to-back Writes ---");
    do_write(8'h1A, 8'h11);
    do_write(8'h1B, 8'h22);
    check(!error, 5, "No error on back-to-back writes");

    // Results
  
    #1000;
    $display("\n========================================");
    $display(" Results: %0d PASSED, %0d FAILED", pass_count, fail_count);
    $display("========================================\n");
    $finish;
end

// Capture data_valid pulses
always @(posedge clk) begin
    if (data_valid) begin
        captured_data[data_idx % 4] = read_data;
        $display("[TB] Byte received [%0d] = 0x%02X", data_idx, read_data);
        data_idx = data_idx + 1;
    end
end

// Timeout watchdog
initial begin
    #50_000_000;
    $display("[WATCHDOG] Simulation timeout!");
    $finish;
end

endmodule