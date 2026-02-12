`timescale 1ns/1ps
module i2s_audio_reciever#(
	parameter sample_width=16)
	(
	//system clock
	input wire sys_clk,
	input wire sys_rst_n,
	//i2s interface signals
	input wire i2s_bclk, //bit clock
	input wire i2s_ws,   //word select
	input wire i2s_sd,   //serial data in
	//output parallel samples
	output reg [sample_width-1:0] sample_out, //main sample
	output reg sample_valid, //validation
	output reg channel_id
	);
	reg [sample_width-1:0] shift_reg; //deserializing incoming bits
	reg [4:0] bit_counter; //5 bits to count upto 32 bit should that be used
	reg ws_delayed; //previous state for edge detection
	//reg ws_edge_detected;//flag to know wether word is complete or not
	//sample buffer for cdc
	reg [sample_width-1:0] sample_buffer;
	reg buffer_valid;
	reg buffer_channel;
	//2 ff sync chain for cdc
	reg buffer_valid_sync1,buffer_valid_sync2;
	reg [sample_width-1:0] sample_sync1,sample_sync2;
	reg channel_sync1, channel_sync2;
	//deserializer always block
	always @(posedge i2s_bclk or negedge sys_rst_n) begin
	   if(!sys_rst_n) begin
	      //reset regs to known states
	      shift_reg <= {sample_width{1'b0}};
	      bit_counter <=5'b0;
	      ws_delayed <= 1'b0;
	      //ws_edge_detected <= 1'b0;
	      sample_buffer <={sample_width{1'b0}};
	      buffer_valid <= 1'b0;
	      buffer_channel <= 1'b0;
	   end
	   else begin
	      ws_delayed <= i2s_ws;
	      //ws_edge_detected <= (ws_delayed != i2s_ws);
	      //new transmission detection
	      if(ws_delayed != i2s_ws) begin
	         //new sample
	         if(bit_counter== sample_width) begin
	            //correct number of bits=transfer sample to buffer for cdc
	            sample_buffer <= shift_reg;
	            buffer_valid <= 1'b1;
	            buffer_channel <= ws_delayed;
	         /*end else begin
	         //bit mismatch= invalid for processing downstream
	            buffer_valid <= 1'b0;*/
	         end
	         bit_counter <= 5'b0;
	         shift_reg <= {sample_width{1'b0}};
	         end else begin
	         //recieving bits= shift left since since i2s transmits msb first
	         shift_reg <= {shift_reg[sample_width-2:0],i2s_sd};
	         if(bit_counter < sample_width) begin
	            bit_counter <= bit_counter+1'b1;
	         end
	         //clear buffer valid flag for downstream clock
	         if(buffer_valid) begin
	            buffer_valid <= 1'b0;
	         end
	      end
	   end
	end
	   //2ff chain for cdc sync
	   always @(posedge sys_clk or negedge sys_rst_n) begin
	      if(!sys_rst_n) begin
	         //reset chain
	         buffer_valid_sync1 <= 1'b0;
	         buffer_valid_sync2 <= 1'b0;
	         channel_sync1 <= 1'b0;
	         channel_sync2 <= 1'b0;
	      end
	      else begin
	         //first sync stage to capture signal from other domain
	         buffer_valid_sync1 <= buffer_valid;
	         sample_sync1 <= sample_buffer;
	         channel_sync1 <= buffer_channel;
	         //second sync stage
	         buffer_valid_sync2 <= buffer_valid_sync1;
	         sample_sync2 <= sample_sync1;
	         channel_sync2 <= channel_sync1;
	      end
	   end
	   //final output
	   always @(posedge sys_clk or negedge sys_rst_n) begin
	      if(!sys_rst_n) begin
	         sample_out <= {sample_width{1'b0}};
	         sample_valid <= 1'b0;
	         channel_id <= 1'b0;
	      end
	      else begin
	         sample_out <= sample_sync2;
	         sample_valid <= buffer_valid_sync2;
	         channel_id <= channel_sync2;
	      end
	   end
endmodule
