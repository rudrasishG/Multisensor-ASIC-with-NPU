module cordic_magnitude_computer #(
    parameter DATA_WIDTH = 16,
    parameter ITERATIONS = 12;
  ) (
    input wire clk,
    input wire rst_n,
    input wire signed [DATA_WIDTH-1:0] real_in,
    input wire signed [DATA_WIDTH-1:0] imag_in,
    output wire signed [DATA_WIDTH-1:0] magnitude_out,
    output reg valid_out
  );
  //rotate vector to x-axis
  reg signed [DATA_WIDTH-1:0] x, y;
  reg signed [DATA_WIDTH-1:0] x_next, y_next;
  reg [3:0] iteration;
  reg computing;
  // CORDIC scale factor (approximately 1.647) for division
  localparam signed [DATA_WIDTH-1:0] K = 16'h5323; // 1.647 in Q15
  always @(posedge clk or negedge rst_n)
  begin
    if (!rst_n)
    begin
      x <= {DATA_WIDTH{1'b0}};
      y <= {DATA_WIDTH{1'b0}};
      iteration <= 4'd0;
      computing <= 1'b0;
      valid_out <= 1'b0;
    end
    else
    begin
      if (!computing)
      begin
        // Start new computation
        // Pre-rotation to first quadrant
        x <= (real_in[DATA_WIDTH-1]) ? -real_in : real_in;
        y <= (imag_in[DATA_WIDTH-1]) ? -imag_in : imag_in;
        iteration <= 4'd0;
        computing <= 1'b1;
        valid_out <= 1'b0;
      end
      else if (iteration < ITERATIONS)
      begin
        // CORDIC micro-rotation
        if (y[DATA_WIDTH-1])
        begin // y is negative, rotate clockwise
          x_next = x - (y >>> iteration);
          y_next = y + (x >>> iteration);
        end
        else
        begin // y is positive, rotate counter-clockwise
          x_next = x + (y >>> iteration);
          y_next = y - (x >>> iteration);
        end
        x <= x_next;
        y <= y_next;
        iteration <= iteration + 1'b1;
      end
      else
      begin
        //scale factor
        magnitude_out <= (x * K) >>> (DATA_WIDTH-1);
        valid_out <= 1'b1;
        computing <= 1'b0;
      end
    end
  end
endmodule
