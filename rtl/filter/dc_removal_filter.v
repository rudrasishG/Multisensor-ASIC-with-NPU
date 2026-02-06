//first order iir high-pass filter
module dc_removal_filter #(
   parameter data_width = 16,
   parameter coeff_width = 16, //coeff precision
   parameter alpha = 16'h7FE0 // filter coeff(0.998)
   ) (
      input wire clk,
      input wire rst_n,
      input wire signed [data_width-1:0] data_in,
      input wire data_valid,
      output reg signed [data_width-1:0] data_out,
      output reg data_out_valid
    );
     //prev input samples
     reg signed [data_width-1:0] x_prev; //x[n-1]
     reg signed [data_width-1:0] y_prev; //y[n-1]
     //internal computation signals
     wire signed [data_width:0] diff_in; //x[n]-x[n-1]
     wire signed [data_width:0] sum_temp; //y[n-1] +(x[n] - x[n-1])
     wire signed [data_width + coeff_width -1:0] mult_result;
     assign diff_in = data_in - x_prev;
     assign sum_temp = y_prev + diff_in;
     assign mult_result = sum_temp * $signed(alpha);
     //q15 so we use [30:15] bits
     wire signed [data_width-1:0] filtered_output;
     assign filtered_output = mult_result[data_width + coeff_width -2 : coeff_width -1];
     always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
           x_prev <= {data_width{1'b0}};
           y_prev <= {data_width{1'b0}};
           data_out <= {data_width{1'b0}};
           data_out_valid <= 1'b0;
        end
        else begin
           if (data_valid) begin
              //update delay elements
              x_prev <= data_in;
              y_prev <= filtered_output;
              //actual output
              data_out <= filtered_output;
              data_out_valid <= 1'b1;
           end
           else begin 
              data_out_valid <= 1'b0;
           end
        end
     end
endmodule
              
     
