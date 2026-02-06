module i2c_imu_interface#(
	parameter sys_clk_freq = 50_000_000,
	parameter i2c_clk_freq = 400_000;
	parameter slave_addr = 7'h68
	)(
	//system interface
	input wire clk,
	input wire rst_n.
	//control interface 
	input wire start,
	input wire rw,
	input wire [7:0] reg_addr,
	input wire [7:0] write_data,
	input wire [3:0] burst_length,
	output reg [7:0] read_data,
	output reg data_valid,
	output reg busy,
	output reg error,
	//physical interface
	input wire sda,
	input wire scl
	);
	//i2c clk generation
	localparam clk_divide = (sys_clk_freq/ (4 * i2c_clk_freq));
	reg [15:0] clk_counter;
	reg [1:0] clk_phase;
	wire clk_tick;
	assign clk_tick= (clk_counter == clk_divide -1);
	always @(posedge clk or negedge rst_n) begin
	   if(!rst_n) begin
	      clk_counter <= 16'b0;
	      clk_phase <= 2'b00;
	   end
	   else if (busy) begin
	      if (clk_tick) begin
	         clk_counter <= 16'b0;
	         clk_phase <= clk_phase +1'b1;
	      end
	      else begin
	         clk_counter <= clk_counter +1'b1;
	      end
	   end
	   else begin
	      clk_counter <= 16'b0;
	      clk_phase <= 2'b00;
	   end
	end
	//control sequence state encoding
	localparam [3:0]
	   state_idle = 4'd0,
	   state_start = 4'd1,
	   state_addr_w =4'd2,//sending slave addr with write bit
	   state_addr_ack = 4'd3,
	   state_reg_addr = 4'd4,
	   state_reg_ack = 4'd5,
	   state_write_data = 4'd6,
	   state_write_ack = 4'd7,
	   state_restart = 4'd8, // restart for read
	   state_addr_r = 4'd9,  //send slave address with read bit
	   state_read_data = 4'd10,
	   state_read_ack = 4'd11,  //send ack/nack to slave
	   state_stop = 4'd12,
	   state_error = 4'd13;
	reg [3:0] state;
	reg [3:0] next_state;
	reg [3:0] bit_index; //tracks bit pos
	reg [3:0] byte_count; //tracks byte count during burst
	reg [7:0] shift_reg;
	reg sda_out; 
	reg sda_oe; //sda output enable
	reg rw_bit; //read/write command
	reg [7:0] reg_addr_buf;
	reg [3:0] burst_len_buf;
	//state machine seq logic
	always @(posedge clk or negedge rst_n) begin
	   if(!rst)n) begin
	      state <= state_idle;
	      busy <= 1'b0;
	      error <= 1'b0;
	   end
	   else begin
	      state <= next_state;
	      busy <= (state != state_idle);
	      if (start && !busy) begin
 	         rw_bit <= rw;
                 reg_addr_buf <= reg_addr;
                 burst_len_buf <= burst_length;
              end
           end
        end
        //state machine comb logic
        always @(*) begin
		next_state=state;
		case(state)
		   state_idle:begin
		      if(start) begin
		         next_state=state_start;
		      end
		   end
		   state_start: begin
		      if(clk_tick && clk_phase == 2'b11) begin
		         next_state = state_addr_w;
		      end
		   end
		   state_addr_w: begin
		      //send slave addr and write bit
		      if(clk_tick && clk_phase == 2'b11 && bit_index == 4'd8) begin
		         next_state = state_addr_ack;
		      end
		   end
		   state_addr_ack: begin
		      //ack from slave
		      if (clk_tick && clk_phase == 2'b11) begin
		         if (sda_in == 1'b0) begin //ack recieved
		            next_state = state_reg_addr;
		         end
		         else begin //nack
		            next_state = state_error;
		         end
		      end
		   end
		   state_reg_addr: begin
		      //send reg address
		      if (clk_tick && clk_phase == 2'b11 && bit_index == 4'b8) begin
		         next_state = state_reg_ack;
		      end
		   end
		   state_reg_ack: begin
		      if (clk_tick && clk_phase == 2'b11) begin
		         if (sda_in == 1'b0) begin //ack
		            if (rw_bit) begin
		               next_state = state_restart; //read needs restart
		            end
		            else begin
		               next_state = state_write_data; 
		            end
		         end
		         else begin
		            next_state = state_error;
		         end
		      end
		   end
		   state_write_data: begin
		      if (clk_tick && clk_phase == 2'b11 && bit_index == 4'b8) begin
		         next_state = state_write_ack;
		      end
		   end
		   state_write_ack: begin
		      if (clk_tick && clk_phase == 2'b11) begin
		         next_state = state_stop; //write over
		      end
		   end
		   state_restart: begin
		      //repeat start for read after write
		      if (clk_tick && clk_phase == 2'b11) begin
		         next_state = state_addr_r;
		      end
		   end
		   state_addr_r: begin
		      //sending slave add and read bit
		      if (clk_tick && clk_phase == 2'b11 && bit_index == 4'd8) begin
		         next_state = state_addr_ack;
		      end
		   end
		   state_read_data: begin
		      //receive data byte from slave
		      if (clk_tick && clk_phase == 2'b11 && bit_index == 4'd8) begin
		         next_state = state_read_ack;
		      end
		   end
		   state_read_ack: begin
		      //send ack or nack if more bytes expected
		      if (clk_tick && clk_phase == 2'b11) begin
		         if (byte_count < burst_len_buf) begin
		            next_state = state_read_data; // more bytes
		         end
		         else begin
		            next_state = state_stop; //burst complete
		         end
		      end
		   end
		   state_stop: begin
		      if (clk_tick && clk_phase == 2'b11) begin
		         next_state = state_idle;
		      end
		   end
		   state_error: begin
		      next_state = state_idle; // return to idle after error
		   end
		endcase
	     end
     //i2c signal generation
     always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
           scl <= 1'b1;
           sda_out <= 1'b1;
           sda_oe <= 1'b0;
           bit_index <= 4'd0;
           byte_count <= 4'd0;
           shift_reg <= 8'h00;
           read_data <= 8'h00;
           data_valid <= 1'b0;
        end
        else begin
           // Default: data_valid is a single-cycle pulse
           data_valid <= 1'b0;
           case (state)
              state_idle: begin
                 scl <= 1'b1;
                 sda_out <= 1'b1;
                 sda_oe <= 1'b0;
                 bit_index <= 4'd0;
                 byte_count <= 4'd0;
              end
              state_start: begin
                 // Start condition: SDA falls while SCL is high
                 sda_oe <= 1'b1;
		 case (clk_phase)
		    2'b00: begin scl <= 1'b1; sda_out <= 1'b1; end
		    2'b01: begin scl <= 1'b1; sda_out <= 1'b0; end // SDA falls
		    2'b10: begin scl <= 1'b0; sda_out <= 1'b0; end // SCL falls
		    2'b11: begin scl <= 1'b0; sda_out <= 1'b0; end
                 endcase
                 if (clk_tick && clk_phase == 2'b01) begin
                    // Load address into shift register for transmission
                    shift_reg <= {slave_addr, 1'b0}; // Append write bit
                    bit_index <= 4'd0;
                 end
              end
              state_addr_w: begin
                 //send slave address with write bit
                 sda_oe <= 1'b1;
                 case(clk_phase)
                    2'b00: begin
                       scl <= 1'b0;
                       //setup data at falling edge of scl
                       sda <= shift_reg[7- bit_index];
                    end
                    2'b01: begin scl <= 1'b1; end
                    2'b10: begin scl <= 1'b1; end
                    2'b11: begin 
                       scl <= 1'b0;
                       if(clk_tick) begin
                          bit_index <= bit_index + 1'b1;
                       end
                    end
                 endcase
              end
              state_addr_ack: begin
                 //release sda and check ack from slave
                 sda_oe <= 1'b0; //slave drives sda
                 case(clk_phase)
                    2'b00: begin scl <= 1'b0; end
                    2'b01: begin scl <=1'b1; end
                    2'b10: begin 
                       scl <= 1'b1;
                       if(sda_in == 1'b1) begin
                          error <= 1'b1; //nack received
                       end
                    end
                    2'b11: begin
                       scl <= 1'b0;
                       if (clk_tick) begin
                          //prepare reg for next state
                          shift_reg <= reg_addr_buf;
                          bit_index <= 4'd0;
                       end
                    end
                 endcase
              end
              state_reg_addr: begin
                  //send reg address(8bits)
                  sda_oe <= 1'b1;
                  case (clk_phase)
                     2'b000: begin
                        scl <= 1'b0;
                        sda_out <= shift_reg[7- bit_index];
                     end
                     2'b01: begin scl <= 1'b1; end
                     2'b10: begin scl <= 1'b1; end
                     2'b11: begin
                        scl <= 1'b0;
                        if(clk_tick) begin
                           bit_index <= bit_index + 1'b1;
                        end
                     end
                  endcase
               end
               state_reg_ack: begin
                  //wait for ack after reg address
                  sda_oe <= 1'b0; 
                  case (clk_phase)
                      2'b00: begin scl <= 1'b0; end
                      2'b01: begin scl <= 1'b1; end
                      2'b10: begin
                         scl <= 1'b1;
                         if (sda_in == 1'b1) begin
                            error <= 1'b1;
                         end
                      end
                      2'b11: begin
                         scl <= 1'b0;
                         if (clk_tick) begin
                            if (rw_bit) begin
                               //restart
                               bit_index <= 4'd0;
                            end else begin
                               //write op..prepare data
                               shift_reg <= write_data;
                               bit_index <= 4'b0;
                            end
                         end
                      end
                   endcase
                   state_write_data: begin
                      //send byte to write
                      sda_oe <= 1'b1;
                      case (clk_phase)
                         2'b00: begin
                            scl <= 1'b0;
                            sda_out <= shift_reg[7- bit_index];
                         end
                         2'b01: begin scl <= 1'b1; end
                         2'b10: begin scl <= 1'b1; end
                         2'b11: begin
                            scl <= 1'b0;
                            if (clk_tcik) begin
                               bit_index <= bit_index +1'b1;
                            end
                         end
                      endcase
                   end
                   state_write_ack: begin
                      //wait for ack from slave
                      sda_oe <= 1'b0;
                      case (clk_phase)
                         2'b00: begin scl<= 1'b0; end
                         2'b01: begin scl<= 1'b1; end
                         2'b10: begin
                            scl <= 1'b1;
                            if (sda_in == 1'b1) begin
                               error <= 1'b1;
                            end
                         end
                         2'b11: begin scl <= 1'b0; end
                      endcase
                   end
                   state_restart: begin
                      //repeat start..read after write
                      sda_oe <= 1'b1;
                      case (clk_phase)
                         2'b00: begin scl <= 1'b0; sda_out <= 1'b0; end
                         2'b01: begin scl <= 1'b0; sda_out <= 1'b0; end
                         2'b10: begin scl <= 1'b0; sda_out <= 1'b1; end
                         2'b11: begin
                            scl <= 1'b1;
                            sda_out <= 1'b0; //restart here
                            if (clk_tick) begin
                               //prepare address with read
                               shift_reg <= {slave_addr , 1'b1};
                               bit_index <= 4'b0;
                            end
                         end
                      endcase
                   end
                   state_addr_r: begin
                      //send slave address with read bit
                      sda_oe <= 1'b1;
                      case (clk_phase)
                         2'b00: begin
                            scl <= 1'b0;
                            sda_out <= shift_reg[7- bit_index];
                         end
                         2'b01: begin scl <= 1'b1; end
                         2'b10: begin scl <= 1'b1; end
                         2'b11: begin 
                            scl <= 1'b0;
                            if (clk_tick) begin
                               bit_index <= bit_index +1'b1;
                            end
                         end
                      endcase
                      //after this should go to address ack state and then read_data and so on
                      if (clk_tick && clk_phase == 2'b11 && bit_index == 4'd7) begin
                         bit_index <= 4'd0;
                         byte_count <= 4'd1; //byte ctr for burst read
                      end
                   end
                   state_read_data: begin
                      //read from slave
                      sda_oe <= 1'b0;
                      case (clk_phase)
                         2'b00: begin scl <= 1'b0; end
                         2'b01: begin scl <= 1'b1; end
                         2'b10: begin
                            scl <= 1'b1;
                            if (clk_tick) begin
                               shift_reg <= {shift_reg[6:0], sda_in};
                            end
                         end
                         2'b11: begin
                            scl <= 1'b0;
                            if (clk_tick) begin
                               bit_index <= bit_index + 1'b1;
                               if(bit_index ==4'd7) begin
                                  read_data <= shift_reg;
                                  data_valid <= 1'b1; //yay
                                  bit_index <= 4'd0;
                               end
                            end
                         end
                      endcase
                   end
                   state_read_ack: begin
                      //ack or nack to stop slave
                      sda_oe <=1'b1;
                      case (clk_phase)
                      2'b00: begin
                         scl <= 1'b0;
                         if(byte_count < burst_len_buf) begin
                            sda_out <= 1'b0; //more reading to be done
                         end else begin
                            sda_out <= 1'b1; //read done
                         end
                      end
                      2'b01: begin scl <= 1'b1; end
                      2'b10: begin scl<= 1'b1; end
                      2'b11: begin 
                         scl <= 1'b0;
                         if (clk_tick) begin
                            if (byte_count < byte_len_buf) begin
                               byte_count <= byte_count + 1'b1;
                               bit_index <= 4'd0;
                            end
                         end
                      end
                   endcase
                end
                state_stop: begin
                   sda_oe <= 1'b1;
                   case (clk_phase)
                      2'b00: begin scl <= 1'b0; sda_out <=1'b0; end
                      2'b01: begin scl <= 1'b1; sda_out <= 1'b0; end
                      2'b10: begin scl <= 1'b1; sda_out <= 1'b1; end
                      2'b11: begtin scl <= 1'b1; sda_out <= 1'b1; end
                   endcase
                end
                state_error: begin
                   //error= return to idle
                   scl <= 1'b1;
                   sda_out <= 1'b1;
                   sda_oe <= 1'b0;
                   error <= 1'b1;
                end
                default: begin
                   scl <= 1'b1;
                   sda_out <= 1'b1;
                   sda_oe <= 1'b0;
                end
             endcase
          end
       end
 endmodule
                   
                         
                   
                         
                          
                    
                       
                       
                       
                       
              
               
           
           
	
	   
	
