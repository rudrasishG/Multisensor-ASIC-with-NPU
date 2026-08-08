module cordic_magnitude_computer #(
    parameter DATA_WIDTH = 16,
    parameter ITERATIONS = 12   // FIX1: was "12;" — semicolon inside #() is illegal Verilog-2005 syntax
) (
    input wire clk,
    input wire rst_n,
    input wire signed [DATA_WIDTH-1:0] real_in,
    input wire signed [DATA_WIDTH-1:0] imag_in,
    output reg [DATA_WIDTH-1:0] magnitude_out,  // FIX2: was "output wire" but driven inside always block — must be reg
    output reg valid_out
);
    // Rotate vector to x-axis using CORDIC vectoring mode
    reg signed [DATA_WIDTH-1:0] x, y;
    reg signed [DATA_WIDTH-1:0] x_next, y_next;
    reg [3:0] iteration;
    reg computing;

    // CORDIC gain correction: 1/K ≈ 0.6073 in Q15 = 0x4DBA
    // K (product of sec(atan(2^-i))) ≈ 1.6468, so 1/K ≈ 0.6073
    localparam signed [DATA_WIDTH-1:0] K_INV = 16'h4DBA;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            x         <= {DATA_WIDTH{1'b0}};
            y         <= {DATA_WIDTH{1'b0}};
            iteration <= 4'd0;
            computing <= 1'b0;
            valid_out <= 1'b0;
        end
        else begin
            if (!computing) begin
                // Pre-rotate to first quadrant (take absolute values)
                x         <= (real_in[DATA_WIDTH-1]) ? -real_in : real_in;
                y         <= (imag_in[DATA_WIDTH-1]) ? -imag_in : imag_in;
                iteration <= 4'd0;
                computing <= 1'b1;
                valid_out <= 1'b0;
            end
            else if (iteration < ITERATIONS) begin
                // CORDIC vectoring: drive y toward zero
                if (y[DATA_WIDTH-1]) begin
                    // y negative → rotate clockwise
                    x_next = x - (y >>> iteration);
                    y_next = y + (x >>> iteration);
                end
                else begin
                    // y positive → rotate counter-clockwise
                    x_next = x + (y >>> iteration);
                    y_next = y - (x >>> iteration);
                end
                x         <= x_next;
                y         <= y_next;
                iteration <= iteration + 1'b1;
            end
            else begin
                // After ITERATIONS, x ≈ K * magnitude; apply 1/K correction
                magnitude_out <= (x * K_INV) >>> (DATA_WIDTH-1);
                valid_out     <= 1'b1;
                computing     <= 1'b0;
            end
        end
    end

endmodule
