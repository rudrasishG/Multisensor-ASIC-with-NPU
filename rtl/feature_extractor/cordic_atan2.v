module cordic_atan2 #(
    parameter DATA_WIDTH = 16,
    parameter ITERATIONS = 12
)(
    input wire clk,
    input wire rst_n,
    input wire signed [DATA_WIDTH-1:0] y_in,
    input wire signed [DATA_WIDTH-1:0] x_in,
    output reg signed [DATA_WIDTH-1:0] angle_out,
    output reg valid_out
);

    // CORDIC atan table (same as for magnitude computation)
    reg signed [DATA_WIDTH-1:0] atan_table [0:ITERATIONS-1];
    
    initial begin
        // atan(2^-i) in Q15 format (radians * 32768/pi)
        atan_table[0]  = 16'd8192;  
        atan_table[1]  = 16'd4836;   
        atan_table[2]  = 16'd2555;   
        atan_table[3]  = 16'd1297;  
        atan_table[4]  = 16'd651;    
        atan_table[5]  = 16'd326;    
        atan_table[6]  = 16'd163;   
        atan_table[7]  = 16'd81;    
        atan_table[8]  = 16'd41;     
        atan_table[9]  = 16'd20;     
        atan_table[10] = 16'd10;     
        atan_table[11] = 16'd5;      
    end
    
    // CORDIC state variables
    reg signed [DATA_WIDTH-1:0] x, y, z;
    reg signed [DATA_WIDTH-1:0] x_next, y_next;
    reg [3:0] iteration;
    reg computing;
    reg quadrant_adjust;
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            x <= {DATA_WIDTH{1'b0}};
            y <= {DATA_WIDTH{1'b0}};
            z <= {DATA_WIDTH{1'b0}};
            iteration <= 4'd0;
            computing <= 1'b0;
            valid_out <= 1'b0;
            quadrant_adjust <= 1'b0;
        end
        else begin
            if (!computing) begin
                // Need to pre-rotate if in quadrant 2 or 3
                
                if (x_in[DATA_WIDTH-1]) begin // x negative
                    // Quadrant 2 or 3: rotate by pi
                    x <= -x_in;
                    y <= -y_in;
                    quadrant_adjust <= 1'b1;
                end
                else begin
                    x <= x_in;
                    y <= y_in;
                    quadrant_adjust <= 1'b0;
                end
                
                z <= {DATA_WIDTH{1'b0}};
                iteration <= 4'd0;
                computing <= 1'b1;
                valid_out <= 1'b0;
            end
            else if (iteration < ITERATIONS) begin
                // CORDIC vectoring iteration
                // driving y to zero by rotating
                
                if (y[DATA_WIDTH-1]) begin 
                    // y is negative, rotate clockwise
                    x_next = x - (y >>> iteration);
                    y_next = y + (x >>> iteration);
                    z <= z - atan_table[iteration];
                end
                else begin 
                    // y is positive, rotate counter-clockwise
                    x_next = x + (y >>> iteration);
                    y_next = y - (x >>> iteration);
                    z <= z + atan_table[iteration];
                end
                
                x <= x_next;
                y <= y_next;
                iteration <= iteration + 1'b1;
            end
            else begin
                // Adjust for quadrant if needed
                if (quadrant_adjust) begin
                    if (y_in[DATA_WIDTH-1]) // Original y was negative
                        angle_out <= z - 16'd25736; // z - pi
                    else
                        angle_out <= z + 16'd25736; // z + pi
                end
                else begin
                    angle_out <= z;
                end
                
                valid_out <= 1'b1;
                computing <= 1'b0;
            end
        end
    end

endmodule