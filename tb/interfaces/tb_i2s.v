`timescale 1ns/1ps
module tb_i2s;
    //control signals
    reg sys_clk;
    reg sys_rst_n;
    reg i2s_bclk;
    reg i2s_ws;
    reg i2s_sd;
    //output wires
    wire [15:0] sample_out;
    wire sample_valid;
    wire channel_id;
    integer errors; //count number of errors
    integer test_number; //which test is running
    reg [15:0] shift_reg;
    reg [15:0] expected_sample;
    wire ws_edge_detected;
    
    //dut instantiation
    i2s_audio_reciever dut(
       .sys_clk(sys_clk),
       .sys_rst_n(sys_rst_n),
       .i2s_bclk(i2s_bclk),
       .i2s_ws(i2s_ws),
       .i2s_sd(i2s_sd),
       .sample_out(sample_out),
       .sample_valid(sample_valid),
       .channel_id(channel_id)
    );
    //debug block ..will be removed later
    always @(posedge i2s_bclk) begin
       if (sys_rst_n) begin
           $display("Time=%0t: i2s_sd=%b, shift_reg=%h, bit_counter=%0d", 
                     $time, i2s_sd, dut.shift_reg, dut.bit_counter);
       end
    end
    //system clock
    initial begin
       sys_clk = 0;
       forever #10 sys_clk = ~sys_clk;
    end
    //i2s clock
    initial begin
       i2s_bclk = 0;
       forever #976.5 i2s_bclk = ~i2s_bclk;
    end
    task send_i2s_sample;
       input [15:0] data;
       input channel;
       integer i;
       begin
       @(negedge i2s_bclk);
       i2s_ws = channel;
       #50
       //@(negedge i2s_bclk);
       for(i = 15; i>= 0 ; i= i - 1) begin
          @(negedge i2s_bclk);
          #50
          i2s_sd = data[i];
       end
       @(negedge i2s_bclk);
       i2s_ws = ~channel;
       //i2s_sd = 0;
       $display("sent 0x%04h on channel %0d", data , channel);
       end
    endtask
    
    task wait_for_valid;
       integer timeout;
    begin
       timeout = 0;
       while(!sample_valid && timeout< 10000) begin
          @(posedge sys_clk);
          timeout = timeout + 1;
          if ( timeout == 500) begin 
             $display("debug: still waiting for sample");
          end 
       end
       if (timeout >= 10000) begin
          $display("error: timeout waiting for valid signal");
          errors = errors+1;
       end
       // @(posedge sys_clk);
    end
    endtask       
    //test
    initial begin
       $dumpfile("tb_i2s.vcd");
       $dumpvars(0 , tb_i2s);
       //reset
       sys_rst_n = 0;
       i2s_ws = 0;
       i2s_sd = 0;
       errors = 0;
       test_number = 0;
       //release reset
       #100
       sys_rst_n = 1;
       #100
       test_number = 1;
       $display("pattern 0xABCD sent");
       expected_sample = 16'hABCD;
       send_i2s_sample(16'hABCD , 1'b0);
       wait_for_valid();
       if (sample_out !== 16'hABCD) begin
          $display("error: expected 0xABCD , got 0x%04h" ,sample_out);
          errors= errors + 1;
       end else if ( channel_id !== 1'b0) begin
          $display("error: expecting channel 0 , got %0d", channel_id);
          errors= errors + 1;
       end else begin
          $display("passed");
       end
    test_number = 2;
    $display("\nTest %0d: Send pattern 0x1234 on channel 1", test_number);
    expected_sample = 16'h1234;
    send_i2s_sample(16'h1234, 1'b1);  // Channel 1
    wait_for_valid();
    
    if (sample_out !== 16'h1234) begin
        $display("  ✗ ERROR: Expected 0x1234, got 0x%04h", sample_out);
        errors = errors + 1;
    end else if (channel_id !== 1'b1) begin
        $display("  ✗ ERROR: Expected channel 1, got %0d", channel_id);
        errors = errors + 1;
    end else begin
        $display("  ✓ PASS");
    end
    
    //----------------------------------------------------------------------
    // TEST 3: Edge case - all zeros
    // THINK: "What if data is 0x0000? Should still work"
    //----------------------------------------------------------------------
    test_number = 3;
    $display("\nTest %0d: Send all zeros", test_number);
    send_i2s_sample(16'h0000, 1'b0);
    wait_for_valid();
    
    if (sample_out !== 16'h0000) begin
        $display("  ✗ ERROR: Expected 0x0000, got 0x%04h", sample_out);
        errors = errors + 1;
    end else begin
        $display("  ✓ PASS");
    end
    
    //----------------------------------------------------------------------
    // TEST 4: Edge case - all ones
    // THINK: "Test maximum value"
    //----------------------------------------------------------------------
    test_number = 4;
    $display("\nTest %0d: Send all ones", test_number);
    send_i2s_sample(16'hFFFF, 1'b1);
    wait_for_valid();
    
    if (sample_out !== 16'hFFFF) begin
        $display("  ✗ ERROR: Expected 0xFFFF, got 0x%04h", sample_out);
        errors = errors + 1;
    end else begin
        $display("  ✓ PASS");
    end
    
    //----------------------------------------------------------------------
    // TEST 5: Rapid sequence
    // THINK: "What if I send multiple samples back-to-back?"
    //----------------------------------------------------------------------
    test_number = 5;
    $display("\nTest %0d: Send rapid sequence", test_number);
    send_i2s_sample(16'h0001, 1'b0);
    send_i2s_sample(16'h0002, 1'b1);
    send_i2s_sample(16'h0004, 1'b0);
    send_i2s_sample(16'h0008, 1'b1);
    
    $display("  Sent 4 samples in sequence");
     #10000; 
    
    $display("\n========================================");
    if (errors == 0) begin
        $display(" ✓ ALL TESTS PASSED!");
    end else begin
        $display(" ✗ %0d TESTS FAILED", errors);
    end
    $display("========================================\n");
    
    $finish;
end
endmodule
          
       
         
    
       
     
     
