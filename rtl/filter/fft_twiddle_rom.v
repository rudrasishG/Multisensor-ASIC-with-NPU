/* Provides complex twiddle factors W_N^k = e^(-j*2*pi*k/N) where N=512
 Twiddle factors are the complex exponentials used in FFT butterfly operations
 
 For a 512-point FFT with 9 stages, we need twiddle factors with different
 granularities for each stage. Stage s uses W_512^(k*2^s) where k is the  butterfly index within that stage.
 
 Rather than storing all possible values, we store one quarter cycle (128 values)  and use symmetry properties of sine/cosine to generate all required values.
 Design Notes:
 - Stores only positive quarter-wave (0 to pi/2) values
- Uses symmetry: sin(x) = cos(pi/2-x), sin(pi-x) = sin(x), etc.
 - ROM initialized with pre-computed values
 - Could alternatively use CORDIC for on-the-fly computation*/


module fft_twiddle_rom #(
    parameter DATA_WIDTH = 16,
    parameter FFT_SIZE = 512,
    parameter ROM_DEPTH = 128  // Store quarter-wave, use symmetry
  )(
    input wire [3:0] stage,           // FFT stage (0-8)
    input wire [8:0] index,           // Butterfly index within stage

    output reg signed [DATA_WIDTH-1:0] twiddle_real,  // cos(theta)
    output reg signed [DATA_WIDTH-1:0] twiddle_imag   // -sin(theta)
  );

  //==========================================================================
  // Twiddle Factor ROM - Quarter Wave (0 to pi/2)
  // Values in Q15 format (scaled by 32767)
  //==========================================================================

  reg signed [DATA_WIDTH-1:0] cos_rom [0:ROM_DEPTH-1];
  reg signed [DATA_WIDTH-1:0] sin_rom [0:ROM_DEPTH-1];

  // Initialize ROM with pre-computed values
  // For actual synthesis, these would be calculated offline
  // Format: Q15 fixed-point (multiply by 32767)
  //initial begin
  // cos(0) = 1.0, sin(0) = 0.0
  //    cos_rom[0] = 16'h7FFF;   // 0.999969 in Q15
  //    sin_rom[0] = 16'h0000;   // 0.0
  // cos(pi/256), sin(pi/256) - incremental angle per ROM entry
  initial begin
    // Quadrant 1: 0 to π/2 (0° to 90°)
    // addr = angle × 256 / (2π)
    // For 512-point FFT, we only need 0 to π/2

    cos_rom[0] = 16'h7FFF;
    sin_rom[0] = 16'h0000;  // 0°
    cos_rom[1] = 16'h7FFD;
    sin_rom[1] = 16'h0192;  // 0.703125°
    cos_rom[2] = 16'h7FF6;
    sin_rom[2] = 16'h0324;  // 1.40625°
    cos_rom[3] = 16'h7FEA;
    sin_rom[3] = 16'h04B5;  // 2.109375°
    cos_rom[4] = 16'h7FD9;
    sin_rom[4] = 16'h0645;  // 2.8125°
    cos_rom[5] = 16'h7FC2;
    sin_rom[5] = 16'h07D5;  // 3.515625°
    cos_rom[6] = 16'h7FA7;
    sin_rom[6] = 16'h0963;  // 4.21875°
    cos_rom[7] = 16'h7F87;
    sin_rom[7] = 16'h0AF0;  // 4.921875°
    cos_rom[8] = 16'h7F62;
    sin_rom[8] = 16'h0C7C;  // 5.625°
    cos_rom[9] = 16'h7F38;
    sin_rom[9] = 16'h0E05;  // 6.328125°
    cos_rom[10] = 16'h7F09;
    sin_rom[10] = 16'h0F8C; // 7.03125°
    cos_rom[11] = 16'h7ED5;
    sin_rom[11] = 16'h1111; // 7.734375°
    cos_rom[12] = 16'h7E9D;
    sin_rom[12] = 16'h1293; // 8.4375°
    cos_rom[13] = 16'h7E5F;
    sin_rom[13] = 16'h1413; // 9.140625°
    cos_rom[14] = 16'h7E1D;
    sin_rom[14] = 16'h158F; // 9.84375°
    cos_rom[15] = 16'h7DD6;
    sin_rom[15] = 16'h1708; // 10.546875°
    cos_rom[16] = 16'h7D8A;
    sin_rom[16] = 16'h187D; // 11.25°
    cos_rom[17] = 16'h7D39;
    sin_rom[17] = 16'h19EF; // 11.953125°
    cos_rom[18] = 16'h7CE3;
    sin_rom[18] = 16'h1B5C; // 12.65625°
    cos_rom[19] = 16'h7C89;
    sin_rom[19] = 16'h1CC5; // 13.359375°
    cos_rom[20] = 16'h7C29;
    sin_rom[20] = 16'h1E2A; // 14.0625°
    cos_rom[21] = 16'h7BC5;
    sin_rom[21] = 16'h1F8B; // 14.765625°
    cos_rom[22] = 16'h7B5D;
    sin_rom[22] = 16'h20E6; // 15.46875°
    cos_rom[23] = 16'h7AEF;
    sin_rom[23] = 16'h223C; // 16.171875°
    cos_rom[24] = 16'h7A7D;
    sin_rom[24] = 16'h238D; // 16.875°
    cos_rom[25] = 16'h7A05;
    sin_rom[25] = 16'h24D9; // 17.578125°
    cos_rom[26] = 16'h7989;
    sin_rom[26] = 16'h261F; // 18.28125°
    cos_rom[27] = 16'h7909;
    sin_rom[27] = 16'h275F; // 18.984375°
    cos_rom[28] = 16'h7884;
    sin_rom[28] = 16'h289A; // 19.6875°
    cos_rom[29] = 16'h77FA;
    sin_rom[29] = 16'h29CE; // 20.390625°
    cos_rom[30] = 16'h776B;
    sin_rom[30] = 16'h2AFA; // 21.09375°
    cos_rom[31] = 16'h76D9;
    sin_rom[31] = 16'h2C21; // 21.796875°
    cos_rom[32] = 16'h7641;
    sin_rom[32] = 16'h2D41; // 22.5° (π/16)
    cos_rom[33] = 16'h75A5;
    sin_rom[33] = 16'h2E5A; // 23.203125°
    cos_rom[34] = 16'h7504;
    sin_rom[34] = 16'h2F6B; // 23.90625°
    cos_rom[35] = 16'h745F;
    sin_rom[35] = 16'h3075; // 24.609375°
    cos_rom[36] = 16'h73B5;
    sin_rom[36] = 16'h3178; // 25.3125°
    cos_rom[37] = 16'h7307;
    sin_rom[37] = 16'h3273; // 26.015625°
    cos_rom[38] = 16'h7254;
    sin_rom[38] = 16'h3367; // 26.71875°
    cos_rom[39] = 16'h719C;
    sin_rom[39] = 16'h3453; // 27.421875°
    cos_rom[40] = 16'h70E2;
    sin_rom[40] = 16'h3536; // 28.125°
    cos_rom[41] = 16'h7023;
    sin_rom[41] = 16'h3611; // 28.828125°
    cos_rom[42] = 16'h6F5F;
    sin_rom[42] = 16'h36E4; // 29.53125°
    cos_rom[43] = 16'h6E96;
    sin_rom[43] = 16'h37AF; // 30.234375°
    cos_rom[44] = 16'h6DC9;
    sin_rom[44] = 16'h3871; // 30.9375°
    cos_rom[45] = 16'h6CF9;
    sin_rom[45] = 16'h392A; // 31.640625°
    cos_rom[46] = 16'h6C23;
    sin_rom[46] = 16'h39DA; // 32.34375°
    cos_rom[47] = 16'h6B49;
    sin_rom[47] = 16'h3A81; // 33.046875°
    cos_rom[48] = 16'h6A6D;
    sin_rom[48] = 16'h3B20; // 33.75° (3π/32)
    cos_rom[49] = 16'h698C;
    sin_rom[49] = 16'h3BB6; // 34.453125°
    cos_rom[50] = 16'h68A6;
    sin_rom[50] = 16'h3C42; // 35.15625°
    cos_rom[51] = 16'h67BD;
    sin_rom[51] = 16'h3CC5; // 35.859375°
    cos_rom[52] = 16'h66CF;
    sin_rom[52] = 16'h3D3E; // 36.5625°
    cos_rom[53] = 16'h65DD;
    sin_rom[53] = 16'h3DAE; // 37.265625°
    cos_rom[54] = 16'h64E8;
    sin_rom[54] = 16'h3E14; // 37.96875°
    cos_rom[55] = 16'h63EF;
    sin_rom[55] = 16'h3E71; // 38.671875°
    cos_rom[56] = 16'h62F2;
    sin_rom[56] = 16'h3EC5; // 39.375°
    cos_rom[57] = 16'h61F1;
    sin_rom[57] = 16'h3F0F; // 40.078125°
    cos_rom[58] = 16'h60EC;
    sin_rom[58] = 16'h3F4F; // 40.78125°
    cos_rom[59] = 16'h5FE3;
    sin_rom[59] = 16'h3F85; // 41.484375°
    cos_rom[60] = 16'h5ED7;
    sin_rom[60] = 16'h3FB1; // 42.1875°
    cos_rom[61] = 16'h5DC7;
    sin_rom[61] = 16'h3FD3; // 42.890625°
    cos_rom[62] = 16'h5CB4;
    sin_rom[62] = 16'h3FEC; // 43.59375°
    cos_rom[63] = 16'h5B9D;
    sin_rom[63] = 16'h3FFB; // 44.296875°
    cos_rom[64] = 16'h5A82;
    sin_rom[64] = 16'h4000; // 45° (π/8)
    cos_rom[65] = 16'h5964;
    sin_rom[65] = 16'h3FFB; // 45.703125°
    cos_rom[66] = 16'h5842;
    sin_rom[66] = 16'h3FEC; // 46.40625°
    cos_rom[67] = 16'h571D;
    sin_rom[67] = 16'h3FD3; // 47.109375°
    cos_rom[68] = 16'h55F5;
    sin_rom[68] = 16'h3FB1; // 47.8125°
    cos_rom[69] = 16'h54CA;
    sin_rom[69] = 16'h3F85; // 48.515625°
    cos_rom[70] = 16'h539B;
    sin_rom[70] = 16'h3F4F; // 49.21875°
    cos_rom[71] = 16'h5269;
    sin_rom[71] = 16'h3F0F; // 49.921875°
    cos_rom[72] = 16'h5133;
    sin_rom[72] = 16'h3EC5; // 50.625°
    cos_rom[73] = 16'h4FFB;
    sin_rom[73] = 16'h3E71; // 51.328125°
    cos_rom[74] = 16'h4EBF;
    sin_rom[74] = 16'h3E14; // 52.03125°
    cos_rom[75] = 16'h4D81;
    sin_rom[75] = 16'h3DAE; // 52.734375°
    cos_rom[76] = 16'h4C3F;
    sin_rom[76] = 16'h3D3E; // 53.4375°
    cos_rom[77] = 16'h4AFB;
    sin_rom[77] = 16'h3CC5; // 54.140625°
    cos_rom[78] = 16'h49B4;
    sin_rom[78] = 16'h3C42; // 54.84375°
    cos_rom[79] = 16'h4869;
    sin_rom[79] = 16'h3BB6; // 55.546875°
    cos_rom[80] = 16'h471C;
    sin_rom[80] = 16'h3B20; // 56.25°
    cos_rom[81] = 16'h45CD;
    sin_rom[81] = 16'h3A81; // 56.953125°
    cos_rom[82] = 16'h447A;
    sin_rom[82] = 16'h39DA; // 57.65625°
    cos_rom[83] = 16'h4325;
    sin_rom[83] = 16'h392A; // 58.359375°
    cos_rom[84] = 16'h41CE;
    sin_rom[84] = 16'h3871; // 59.0625°
    cos_rom[85] = 16'h4073;
    sin_rom[85] = 16'h37AF; // 59.765625°
    cos_rom[86] = 16'h3F17;
    sin_rom[86] = 16'h36E4; // 60.46875°
    cos_rom[87] = 16'h3DB8;
    sin_rom[87] = 16'h3611; // 61.171875°
    cos_rom[88] = 16'h3C56;
    sin_rom[88] = 16'h3536; // 61.875°
    cos_rom[89] = 16'h3AF2;
    sin_rom[89] = 16'h3453; // 62.578125°
    cos_rom[90] = 16'h398C;
    sin_rom[90] = 16'h3367; // 63.28125°
    cos_rom[91] = 16'h3824;
    sin_rom[91] = 16'h3273; // 63.984375°
    cos_rom[92] = 16'h36BA;
    sin_rom[92] = 16'h3178; // 64.6875°
    cos_rom[93] = 16'h354D;
    sin_rom[93] = 16'h3075; // 65.390625°
    cos_rom[94] = 16'h33DE;
    sin_rom[94] = 16'h2F6B; // 66.09375°
    cos_rom[95] = 16'h326E;
    sin_rom[95] = 16'h2E5A; // 66.796875°
    cos_rom[96] = 16'h30FB;
    sin_rom[96] = 16'h2D41; // 67.5° (3π/16)
    cos_rom[97] = 16'h2F87;
    sin_rom[97] = 16'h2C21; // 68.203125°
    cos_rom[98] = 16'h2E11;
    sin_rom[98] = 16'h2AFA; // 68.90625°
    cos_rom[99] = 16'h2C98;
    sin_rom[99] = 16'h29CE; // 69.609375°
    cos_rom[100] = 16'h2B1F;
    sin_rom[100] = 16'h289A; // 70.3125°
    cos_rom[101] = 16'h29A3;
    sin_rom[101] = 16'h275F; // 71.015625°
    cos_rom[102] = 16'h2826;
    sin_rom[102] = 16'h261F; // 71.71875°
    cos_rom[103] = 16'h26A8;
    sin_rom[103] = 16'h24D9; // 72.421875°
    cos_rom[104] = 16'h2528;
    sin_rom[104] = 16'h238D; // 73.125°
    cos_rom[105] = 16'h23A6;
    sin_rom[105] = 16'h223C; // 73.828125°
    cos_rom[106] = 16'h2223;
    sin_rom[106] = 16'h20E6; // 74.53125°
    cos_rom[107] = 16'h209F;
    sin_rom[107] = 16'h1F8B; // 75.234375°
    cos_rom[108] = 16'h1F19;
    sin_rom[108] = 16'h1E2A; // 75.9375°
    cos_rom[109] = 16'h1D93;
    sin_rom[109] = 16'h1CC5; // 76.640625°
    cos_rom[110] = 16'h1C0B;
    sin_rom[110] = 16'h1B5C; // 77.34375°
    cos_rom[111] = 16'h1A82;
    sin_rom[111] = 16'h19EF; // 78.046875°
    cos_rom[112] = 16'h18F8;
    sin_rom[112] = 16'h187D; // 78.75°
    cos_rom[113] = 16'h176D;
    sin_rom[113] = 16'h1708; // 79.453125°
    cos_rom[114] = 16'h15E2;
    sin_rom[114] = 16'h158F; // 80.15625°
    cos_rom[115] = 16'h1455;
    sin_rom[115] = 16'h1413; // 80.859375°
    cos_rom[116] = 16'h12C8;
    sin_rom[116] = 16'h1293; // 81.5625°
    cos_rom[117] = 16'h113A;
    sin_rom[117] = 16'h1111; // 82.265625°
    cos_rom[118] = 16'h0FAB;
    sin_rom[118] = 16'h0F8C; // 82.96875°
    cos_rom[119] = 16'h0E1B;
    sin_rom[119] = 16'h0E05; // 83.671875°
    cos_rom[120] = 16'h0C8B;
    sin_rom[120] = 16'h0C7C; // 84.375°
    cos_rom[121] = 16'h0AFB;
    sin_rom[121] = 16'h0AF0; // 85.078125°
    cos_rom[122] = 16'h096A;
    sin_rom[122] = 16'h0963; // 85.78125°
    cos_rom[123] = 16'h07D9;
    sin_rom[123] = 16'h07D5; // 86.484375°
    cos_rom[124] = 16'h0647;
    sin_rom[124] = 16'h0645; // 87.1875°
    cos_rom[125] = 16'h04B6;
    sin_rom[125] = 16'h04B5; // 87.890625°
    cos_rom[126] = 16'h0324;
    sin_rom[126] = 16'h0324; // 88.59375°
    cos_rom[127] = 16'h0192;
    sin_rom[127] = 16'h0192; // 89.296875° (close to π/2)
  end

  //==========================================================================
  // Address Calculation and Quadrant Mapping
  // Calculate which twiddle factor is needed based on stage and index
  //==========================================================================

  reg [8:0] twiddle_angle;   // Angle index (0-511)
  reg [1:0] quadrant;        // Which quadrant (0-3)
  reg [6:0] rom_addr;        // ROM address (0-127)
  reg signed [DATA_WIDTH-1:0] cos_val;
  reg signed [DATA_WIDTH-1:0] sin_val;

  always @(*)
  begin
    // Calculate twiddle factor angle based on FFT stage and butterfly index
    // For stage s, butterfly k needs W_512^(k * 2^s)
    // This maps to angle = (k * 2^s) mod 512

    case (stage)
      4'd0:
        twiddle_angle = {index[8:0]};         // k * 1
      4'd1:
        twiddle_angle = {index[7:0], 1'b0};   // k * 2
      4'd2:
        twiddle_angle = {index[6:0], 2'b0};   // k * 4
      4'd3:
        twiddle_angle = {index[5:0], 3'b0};   // k * 8
      4'd4:
        twiddle_angle = {index[4:0], 4'b0};   // k * 16
      4'd5:
        twiddle_angle = {index[3:0], 5'b0};   // k * 32
      4'd6:
        twiddle_angle = {index[2:0], 6'b0};   // k * 64
      4'd7:
        twiddle_angle = {index[1:0], 7'b0};   // k * 128
      4'd8:
        twiddle_angle = {index[0], 8'b0};     // k * 256
      default:
        twiddle_angle = 9'd0;
    endcase

    // Determine quadrant and ROM address using angle symmetry
    // 0-127: Q1 (0 to pi/2)     -> use ROM directly
    // 128-255: Q2 (pi/2 to pi)   -> ROM[256-angle], negate cos
    // 256-383: Q3 (pi to 3pi/2)  -> ROM[angle-256], negate both
    // 384-511: Q4 (3pi/2 to 2pi) -> ROM[512-angle], negate sin

    quadrant = twiddle_angle[8:7];

    case (quadrant)
      2'b00:
        rom_addr = twiddle_angle[6:0];           // Q1: 0-127
      2'b01:
        rom_addr = 7'd127 - twiddle_angle[6:0];  // Q2: mirror
      2'b10:
        rom_addr = twiddle_angle[6:0];           // Q3: 0-127
      2'b11:
        rom_addr = 7'd127 - twiddle_angle[6:0];  // Q4: mirror
    endcase

    // Lookup base values from ROM
    cos_val = cos_rom[rom_addr];
    sin_val = sin_rom[rom_addr];

    // Apply quadrant corrections
    case (quadrant)
      2'b00:
      begin  // Q1: (cos, -sin)
        twiddle_real = cos_val;
        twiddle_imag = -sin_val;
      end
      2'b01:
      begin  // Q2: (-sin, -cos)
        twiddle_real = -sin_val;
        twiddle_imag = -cos_val;
      end
      2'b10:
      begin  // Q3: (-cos, sin)
        twiddle_real = -cos_val;
        twiddle_imag = sin_val;
      end
      2'b11:
      begin  // Q4: (sin, cos)
        twiddle_real = sin_val;
        twiddle_imag = cos_val;
      end
    endcase
  end

endmodule
