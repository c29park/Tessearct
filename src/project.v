`default_nettype none

module tt_um_example(
    input  wire [7:0] ui_in,    // unused
    output wire [7:0] uo_out,   // {hsync,B0,G0,R0,vsync,B1,G1,R1}
    input  wire [7:0] uio_in,   // unused
    output wire [7:0] uio_out,  // unused
    output wire [7:0] uio_oe,   // unused
    input  wire       ena,      // unused
    input  wire       clk,      // ~25 MHz pixel clock
    input  wire       rst_n     // active-low reset
);

  // ============================================================
  // USER TUNABLES
  // ============================================================

  // Geometry scale:
  //   S0     = half-size in x/y/z
  //   WBOOST = half-size in w (distance between inner/outer cubes)
  localparam signed [8:0] S0     = 9'sd145;   // try 48..200
  localparam signed [8:0] WBOOST = 9'sd145;   // start same as S0

  // Screen placement:
  localparam signed [10:0] CENTER_X = 11'd320;
  localparam signed [10:0] CENTER_Y = 11'd240;

  // Global scale factor after projection:
  // Bigger GLOBAL_GAIN = larger on screen
  localparam signed [7:0] GLOBAL_GAIN = 8'sd130;

  // "Camera distance" knobs for perspective:
  // DEPTH_K_W: farther -> less crazy 4D foreshortening
  // DEPTH_K_Z: farther -> less 3D bulge
  localparam signed [7:0] DEPTH_K_W = 8'sd128;
  localparam signed [7:0] DEPTH_K_Z = 8'sd110;

  // Plane enable flags (1 = include that rotation plane in the pipeline)
  // Planes correspond to: zw, yw, yz, xw, xz, xy
  localparam ENABLE_ZW = 1'b0;
  localparam ENABLE_YW = 1'b1;
  localparam ENABLE_YZ = 1'b0;
  localparam ENABLE_XW = 1'b0;
  localparam ENABLE_XZ = 1'b0;
  localparam ENABLE_XY = 1'b0;

  // Spin control:
  // Each plane gets: which frame_ctr bits feed its angle (speed),
  // plus an optional PHASE offset into the LUT.
  //
  // Larger ROT_SPEED_SEL_* => slower spin (because we tap higher bits).
  // PHASE_* just shifts starting angle for that plane.
  localparam integer ROT_SPEED_SEL_ZW = 0;
  localparam integer ROT_SPEED_SEL_YW = 0;
  localparam integer ROT_SPEED_SEL_YZ = 0;
  localparam integer ROT_SPEED_SEL_XW = 0;
  localparam integer ROT_SPEED_SEL_XZ = 0;
  localparam integer ROT_SPEED_SEL_XY = 0;

  localparam [5:0] PHASE_ZW = 6'd0;
  localparam [5:0] PHASE_YW = 6'd8;
  localparam [5:0] PHASE_YZ = 6'd16;
  localparam [5:0] PHASE_XW = 6'd0;
  localparam [5:0] PHASE_XZ = 6'd12;
  localparam [5:0] PHASE_XY = 6'd20;

  // ============================================================
  // VGA TIMING
  // ============================================================
  wire hsync, vsync, video_active;
  wire [9:0] pix_x, pix_y;

  hvsync_generator hvsync_gen (
    .clk(clk),
    .reset(~rst_n),
    .hsync(hsync),
    .vsync(vsync),
    .display_on(video_active),
    .hpos(pix_x),
    .vpos(pix_y)
  );

  // VGA PMOD packing (2:2:2 RGB split across the byte)
  reg [1:0] R, G, B;
  assign uo_out = {hsync, B[0], G[0], R[0], vsync, B[1], G[1], R[1]};

  // unused IO
  assign uio_out = 8'b0;
  assign uio_oe  = 8'b0;
  wire _unused_ok = &{ena, ui_in, uio_in};

  // ============================================================
  // FRAME COUNTER FOR ANIMATION
  // ============================================================
  reg vsync_d;
  reg [15:0] frame_ctr;
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      vsync_d   <= 1'b0;
      frame_ctr <= 16'd0;
    end else begin
      vsync_d <= vsync;
      if (vsync && !vsync_d)
        frame_ctr <= frame_ctr + 16'd1;
    end
  end

  // Angles for all six planes.
  // We'll tap different bit slices of frame_ctr for different speeds.
  wire [5:0] ang_zw = frame_ctr[ROT_SPEED_SEL_ZW +: 6] + PHASE_ZW;
  wire [5:0] ang_yw = frame_ctr[ROT_SPEED_SEL_YW +: 6] + PHASE_YW;
  wire [5:0] ang_yz = frame_ctr[ROT_SPEED_SEL_YZ +: 6] + PHASE_YZ;
  wire [5:0] ang_xw = frame_ctr[ROT_SPEED_SEL_XW +: 6] + PHASE_XW;
  wire [5:0] ang_xz = frame_ctr[ROT_SPEED_SEL_XZ +: 6] + PHASE_XZ;
  wire [5:0] ang_xy = frame_ctr[ROT_SPEED_SEL_XY +: 6] + PHASE_XY;

  // LUT sin/cos for each plane (Q1.7 signed)
  wire signed [7:0] cos_zw_q7, sin_zw_q7;
  wire signed [7:0] cos_yw_q7, sin_yw_q7;
  wire signed [7:0] cos_yz_q7, sin_yz_q7;
  wire signed [7:0] cos_xw_q7, sin_xw_q7;
  wire signed [7:0] cos_xz_q7, sin_xz_q7;
  wire signed [7:0] cos_xy_q7, sin_xy_q7;

  sincos64 lut_zw(.idx(ang_zw), .cos_q(cos_zw_q7), .sin_q(sin_zw_q7));
  sincos64 lut_yw(.idx(ang_yw), .cos_q(cos_yw_q7), .sin_q(sin_yw_q7));
  sincos64 lut_yz(.idx(ang_yz), .cos_q(cos_yz_q7), .sin_q(sin_yz_q7));
  sincos64 lut_xw(.idx(ang_xw), .cos_q(cos_xw_q7), .sin_q(sin_xw_q7));
  sincos64 lut_xz(.idx(ang_xz), .cos_q(cos_xz_q7), .sin_q(sin_xz_q7));
  sincos64 lut_xy(.idx(ang_xy), .cos_q(cos_xy_q7), .sin_q(sin_xy_q7));

  // ============================================================
  // HELPER FUNCTIONS
  // ============================================================

  // abs for ~11-bit signed
  function [10:0] abs11;
    input signed [10:0] v;
    begin
      abs11 = v[10] ? (~v + 1'b1) : v;
    end
  endfunction

  // clamp X to visible 0..639
  function [9:0] clamp_x;
    input signed [10:0] v;
    begin
      if (v < 0)             clamp_x = 10'd0;
      else if (v > 11'sd639) clamp_x = 10'd639;
      else                   clamp_x = v[9:0];
    end
  endfunction

  // clamp Y to visible 0..479
  function [9:0] clamp_y;
    input signed [10:0] v;
    begin
      if (v < 0)             clamp_y = 10'd0;
      else if (v > 11'sd479) clamp_y = 10'd479;
      else                   clamp_y = v[9:0];
    end
  endfunction

  // ~4px-thick line fill check
  function line4;
    input [9:0] px, py;
    input [9:0] x1, y1, x2, y2;
    reg signed [10:0] dx, dy;
    reg signed [10:1] nx_dummy; //unused slice hack to quiet lints
    reg signed [10:0] nx, ny;
    reg signed [21:0] area;
    reg [10:0] adx, ady, maxd;
    reg [9:0] xmin, xmax, ymin, ymax;
    reg box_ok;
    reg signed [21:0] area_abs;
    reg signed [21:0] thresh;
    begin
      dx = $signed({1'b0,x2}) - $signed({1'b0,x1});
      dy = $signed({1'b0,y2}) - $signed({1'b0,y1});
      nx = $signed({1'b0,px}) - $signed({1'b0,x1});
      ny = $signed({1'b0,py}) - $signed({1'b0,y1});
      nx_dummy = nx[10:1]; // keep tools calm
      area = dy * nx - dx * ny;

      adx   = abs11(dx);
      ady   = abs11(dy);
      maxd  = (adx > ady) ? adx : ady;

      xmin = (x1 < x2) ? x1 : x2;
      xmax = (x1 < x2) ? x2 : x1;
      ymin = (y1 < y2) ? y1 : y2;
      ymax = (y1 < y2) ? y2 : y1;

      box_ok = (px >= xmin-3) && (px <= xmax+3) &&
               (py >= ymin-3) && (py <= ymax+3);

      area_abs = area[21] ? -area : area;
      thresh   = {{11{1'b0}}, (maxd<<2)}; // widen ~4px

      line4 = box_ok && (area_abs <= thresh);
    end
  endfunction

  // tiny diamond point (~3px manhattan radius)
  function dot2;
    input [9:0] px, py;
    input [9:0] cx, cy;
    reg signed [10:0] dx, dy;
    reg [10:0] adx, ady;
    reg [10:0] man;
    begin
      dx  = $signed({1'b0,px}) - $signed({1'b0,cx});
      dy  = $signed({1'b0,py}) - $signed({1'b0,cy});
      adx = abs11(dx);
      ady = abs11(dy);
      man = adx + ady;
      dot2 = (man <= 11'd3);
    end
  endfunction

  // 16-entry reciprocal LUT in Q0.8
  // we use this for rough 1/(K - depth)
  function [7:0] inv16_q0p8;
    input [3:0] idx;
    begin
      case (idx)
        4'h0: inv16_q0p8=8'd255; 4'h1: inv16_q0p8=8'd224;
        4'h2: inv16_q0p8=8'd192; 4'h3: inv16_q0p8=8'd171;
        4'h4: inv16_q0p8=8'd153; 4'h5: inv16_q0p8=8'd137;
        4'h6: inv16_q0p8=8'd123; 4'h7: inv16_q0p8=8'd111;
        4'h8: inv16_q0p8=8'd101; 4'h9: inv16_q0p8=8'd92;
        4'hA: inv16_q0p8=8'd85;  4'hB: inv16_q0p8=8'd78;
        4'hC: inv16_q0p8=8'd73;  4'hD: inv16_q0p8=8'd68;
        4'hE: inv16_q0p8=8'd64;  4'hF: inv16_q0p8=8'd60;
      endcase
    end
  endfunction


  // ============================================================
  // project_vertex:
  //
  // This is the heart. For each vertex index vidx[3:0] we:
  //   1. create base +/-S0 in x,y,z and +/-WBOOST in w (Q1.7-ish)
  //   2. apply up to SIX plane rotations in this order:
  //        zw, yw, yz, xw, xz, xy
  //      Each plane is:
  //        [a']   [ cos -sin ] [a]
  //        [b'] = [ sin  cos ] [b]
  //      where (a,b) is that plane's coordinate pair.
  //      Q1.7 * Q1.7 -> Q2.14 -> >>7 back to ~Q1.7
  //
  //   3. perspective from W (DEPTH_K_W) and from Z (DEPTH_K_Z)
  //      using the same style you already had:
  //        scale_w ~ GLOBAL_GAIN * 1/(DEPTH_K_W - w)
  //        scale_z ~            1/(DEPTH_K_Z - z)
  //      combine them, multiply x,y.
  //
  //   4. translate to CENTER_X,Y and clamp
  //
  // Output is {10-bit x, 10-bit y}.
  // ============================================================
  function [19:0] project_vertex;
    input [3:0] vidx;

    // base Q1.7 coords
    reg signed [15:0] x_q, y_q, z_q, w_q;

    // temp mults
    reg signed [23:0] mulA, mulB;

    // perspective bits
    reg signed [15:0] w_clip_q;
    reg signed [8:0]  w_clip_lite;
    reg signed [8:0]  denom_w;
    reg [3:0]         recip_idx_w;
    reg [7:0]         recip_w_q;
    reg signed [23:0] s_w_mul;
    reg signed [15:0] s_w_q;

    reg signed [15:0] z_clip_q;
    reg signed [8:0]  z_clip_lite;
    reg signed [8:0]  denom_z;
    reg [3:0]         recip_idx_z;
    reg [7:0]         recip_z_q;
    reg signed [23:0] s_z_mul;
    reg signed [15:0] s_z_q;

    reg signed [31:0] s_total_mul;
    reg signed [15:0] s_total_q;

    reg signed [31:0] Xmul, Ymul;
    reg signed [10:0] sx, sy;
    reg [9:0] sx_clamped, sy_clamped;

    begin
      // ---- 1. base corner in Q1.7 ----
      x_q = vidx[0] ?  (S0     <<< 7) : -(S0     <<< 7);
      y_q = vidx[1] ?  (S0     <<< 7) : -(S0     <<< 7);
      z_q = vidx[2] ?  (S0     <<< 7) : -(S0     <<< 7);
      w_q = vidx[3] ?  (WBOOST <<< 7) : -(WBOOST <<< 7);

      // ---- 2. chained 4D rotations ----
      // Order: zw, yw, yz, xw, xz, xy
      // For each plane:
      //   mulA = a*cos - b*sin
      //   mulB = a*sin + b*cos
      //   a = mulA>>7; b = mulB>>7

      // zw plane (z <-> w)
      if (ENABLE_ZW) begin
        mulA = z_q * cos_zw_q7 - w_q * sin_zw_q7;
        mulB = z_q * sin_zw_q7 + w_q * cos_zw_q7;
        z_q  = mulA >>> 7;
        w_q  = mulB >>> 7;
      end

      // yw plane (y <-> w)
      if (ENABLE_YW) begin
        mulA = y_q * cos_yw_q7 - w_q * sin_yw_q7;
        mulB = y_q * sin_yw_q7 + w_q * cos_yw_q7;
        y_q  = mulA >>> 7;
        w_q  = mulB >>> 7;
      end

      // yz plane (y <-> z)
      if (ENABLE_YZ) begin
        mulA = y_q * cos_yz_q7 - z_q * sin_yz_q7;
        mulB = y_q * sin_yz_q7 + z_q * cos_yz_q7;
        y_q  = mulA >>> 7;
        z_q  = mulB >>> 7;
      end

      // xw plane (x <-> w)
      if (ENABLE_XW) begin
        mulA = x_q * cos_xw_q7 - w_q * sin_xw_q7;
        mulB = x_q * sin_xw_q7 + w_q * cos_xw_q7;
        x_q  = mulA >>> 7;
        w_q  = mulB >>> 7;
      end

      // xz plane (x <-> z)
      if (ENABLE_XZ) begin
        mulA = x_q * cos_xz_q7 - z_q * sin_xz_q7;
        mulB = x_q * sin_xz_q7 + z_q * cos_xz_q7;
        x_q  = mulA >>> 7;
        z_q  = mulB >>> 7;
      end

      // xy plane (x <-> y)
      if (ENABLE_XY) begin
        mulA = x_q * cos_xy_q7 - y_q * sin_xy_q7;
        mulB = x_q * sin_xy_q7 + y_q * cos_xy_q7;
        x_q  = mulA >>> 7;
        y_q  = mulB >>> 7;
      end

      // ---- 3. perspective from W (DEPTH_K_W) ----
      // clamp w to something ~[-100 .. +100] before 1/(K - w)
      w_clip_q = w_q;
      if (w_clip_q[15]) begin
        w_clip_lite = -9'sd100;
      end else if (w_clip_q[15:7] > 8'sd100) begin
        w_clip_lite = 9'sd100;
      end else begin
        w_clip_lite = {1'b0, w_clip_q[14:7]};
      end

      denom_w = {{1{DEPTH_K_W[7]}}, DEPTH_K_W} - w_clip_lite[7:0];
      if (denom_w[8])
        recip_idx_w = 4'hF;
      else
        recip_idx_w = denom_w[7:4];
      recip_w_q = inv16_q0p8(recip_idx_w); // Q0.8 approx 1/(DEPTH_K_W-w)

      // s_w_q ~ recip_w_q * GLOBAL_GAIN
      s_w_mul = $signed({1'b0,recip_w_q}) * $signed(GLOBAL_GAIN);
      s_w_q   = s_w_mul[22:7]; // back to ~Q1.7

      // ---- 4. perspective from Z (DEPTH_K_Z) ----
      z_clip_q = z_q;
      if (z_clip_q[15]) begin
        z_clip_lite = -9'sd100;
      end else if (z_clip_q[15:7] > 8'sd100) begin
        z_clip_lite = 9'sd100;
      end else begin
        z_clip_lite = {1'b0, z_clip_q[14:7]};
      end

      denom_z = {{1{DEPTH_K_Z[7]}}, DEPTH_K_Z} - z_clip_lite[7:0];
      if (denom_z[8])
        recip_idx_z = 4'hF;
      else
        recip_idx_z = denom_z[7:4];
      recip_z_q = inv16_q0p8(recip_idx_z); // Q0.8 approx 1/(DEPTH_K_Z-z)

      // s_z_q ~ recip_z_q * 128 (128 ≈ 1.0 in Q1.7)
      s_z_mul = $signed({1'b0,recip_z_q}) * $signed(8'sd128);
      s_z_q   = s_z_mul[22:7]; // ~Q1.7

      // combine
      s_total_mul = $signed(s_w_q) * $signed(s_z_q); // Q1.7*Q1.7 => Q2.14-ish
      s_total_q   = s_total_mul[22:7];               // back to ~Q1.7

      // ---- 5. final screen projection for x,y ----
      Xmul = $signed(x_q) * $signed(s_total_q);
      Ymul = $signed(y_q) * $signed(s_total_q);

      // shift down ~15 to turn Q-ish product into pixels
      sx = $signed(CENTER_X) + (Xmul >>> 15);
      sy = $signed(CENTER_Y) + (Ymul >>> 15);

      // clamp to visible range
      sx_clamped = clamp_x(sx);
      sy_clamped = clamp_y(sy);

      project_vertex = {sx_clamped, sy_clamped};
    end
  endfunction

  // ============================================================
  // GENERATE ALL 16 PROJECTED VERTICES
  // (still per pixel eval — this is heavy, but matches your current style)
  // ============================================================
  wire [19:0] v0  = project_vertex(4'd0 );
  wire [19:0] v1  = project_vertex(4'd1 );
  wire [19:0] v2  = project_vertex(4'd2 );
  wire [19:0] v3  = project_vertex(4'd3 );
  wire [19:0] v4  = project_vertex(4'd4 );
  wire [19:0] v5  = project_vertex(4'd5 );
  wire [19:0] v6  = project_vertex(4'd6 );
  wire [19:0] v7  = project_vertex(4'd7 );
  wire [19:0] v8  = project_vertex(4'd8 );
  wire [19:0] v9  = project_vertex(4'd9 );
  wire [19:0] v10 = project_vertex(4'd10);
  wire [19:0] v11 = project_vertex(4'd11);
  wire [19:0] v12 = project_vertex(4'd12);
  wire [19:0] v13 = project_vertex(4'd13);
  wire [19:0] v14 = project_vertex(4'd14);
  wire [19:0] v15 = project_vertex(4'd15);

  `define VX(v) v[19:10]
  `define VY(v) v[9:0]

  // ============================================================
  // EDGES OF THE TESSERACT
  // same as before
  // ============================================================
  wire e_x = line4(pix_x,pix_y, `VX(v0 ),`VY(v0 ), `VX(v1 ),`VY(v1 )) |
             line4(pix_x,pix_y, `VX(v2 ),`VY(v2 ), `VX(v3 ),`VY(v3 )) |
             line4(pix_x,pix_y, `VX(v4 ),`VY(v4 ), `VX(v5 ),`VY(v5 )) |
             line4(pix_x,pix_y, `VX(v6 ),`VY(v6 ), `VX(v7 ),`VY(v7 )) |
             line4(pix_x,pix_y, `VX(v8 ),`VY(v8 ), `VX(v9 ),`VY(v9 )) |
             line4(pix_x,pix_y, `VX(v10),`VY(v10), `VX(v11),`VY(v11)) |
             line4(pix_x,pix_y, `VX(v12),`VY(v12), `VX(v13),`VY(v13)) |
             line4(pix_x,pix_y, `VX(v14),`VY(v14), `VX(v15),`VY(v15));

  wire e_y = line4(pix_x,pix_y, `VX(v0 ),`VY(v0 ), `VX(v2 ),`VY(v2 )) |
             line4(pix_x,pix_y, `VX(v1 ),`VY(v1 ), `VX(v3 ),`VY(v3 )) |
             line4(pix_x,pix_y, `VX(v4 ),`VY(v4 ), `VX(v6 ),`VY(v6 )) |
             line4(pix_x,pix_y, `VX(v5 ),`VY(v5 ), `VX(v7 ),`VY(v7 )) |
             line4(pix_x,pix_y, `VX(v8 ),`VY(v8 ), `VX(v10),`VY(v10)) |
             line4(pix_x,pix_y, `VX(v9 ),`VY(v9 ), `VX(v11),`VY(v11)) |
             line4(pix_x,pix_y, `VX(v12),`VY(v12), `VX(v14),`VY(v14)) |
             line4(pix_x,pix_y, `VX(v13),`VY(v13), `VX(v15),`VY(v15));

  wire e_z = line4(pix_x,pix_y, `VX(v0 ),`VY(v0 ), `VX(v4 ),`VY(v4 )) |
             line4(pix_x,pix_y, `VX(v1 ),`VY(v1 ), `VX(v5 ),`VY(v5 )) |
             line4(pix_x,pix_y, `VX(v2 ),`VY(v2 ), `VX(v6 ),`VY(v6 )) |
             line4(pix_x,pix_y, `VX(v3 ),`VY(v3 ), `VX(v7 ),`VY(v7 )) |
             line4(pix_x,pix_y, `VX(v8 ),`VY(v8 ), `VX(v12),`VY(v12)) |
             line4(pix_x,pix_y, `VX(v9 ),`VY(v9 ), `VX(v13),`VY(v13)) |
             line4(pix_x,pix_y, `VX(v10),`VY(v10), `VX(v14),`VY(v14)) |
             line4(pix_x,pix_y, `VX(v11),`VY(v11), `VX(v15),`VY(v15));

  wire e_w = line4(pix_x,pix_y, `VX(v0 ),`VY(v0 ), `VX(v8 ),`VY(v8 )) |
             line4(pix_x,pix_y, `VX(v1 ),`VY(v1 ), `VX(v9 ),`VY(v9 )) |
             line4(pix_x,pix_y, `VX(v2 ),`VY(v2 ), `VX(v10),`VY(v10)) |
             line4(pix_x,pix_y, `VX(v3 ),`VY(v3 ), `VX(v11),`VY(v11)) |
             line4(pix_x,pix_y, `VX(v4 ),`VY(v4 ), `VX(v12),`VY(v12)) |
             line4(pix_x,pix_y, `VX(v5 ),`VY(v5 ), `VX(v13),`VY(v13)) |
             line4(pix_x,pix_y, `VX(v6 ),`VY(v6 ), `VX(v14),`VY(v14)) |
             line4(pix_x,pix_y, `VX(v7 ),`VY(v7 ), `VX(v15),`VY(v15));

  wire edge_any   = e_x | e_y | e_z | e_w;
  wire edge_wonly = e_w; // lime highlight for 4D struts

  // vertex dots = draw all 16 vertices
  wire dot_pix =
    dot2(pix_x,pix_y, `VX(v0 ),`VY(v0 )) |
    dot2(pix_x,pix_y, `VX(v1 ),`VY(v1 )) |
    dot2(pix_x,pix_y, `VX(v2 ),`VY(v2 )) |
    dot2(pix_x,pix_y, `VX(v3 ),`VY(v3 )) |
    dot2(pix_x,pix_y, `VX(v4 ),`VY(v4 )) |
    dot2(pix_x,pix_y, `VX(v5 ),`VY(v5 )) |
    dot2(pix_x,pix_y, `VX(v6 ),`VY(v6 )) |
    dot2(pix_x,pix_y, `VX(v7 ),`VY(v7 )) |
    dot2(pix_x,pix_y, `VX(v8 ),`VY(v8 )) |
    dot2(pix_x,pix_y, `VX(v9 ),`VY(v9 )) |
    dot2(pix_x,pix_y, `VX(v10),`VY(v10)) |
    dot2(pix_x,pix_y, `VX(v11),`VY(v11)) |
    dot2(pix_x,pix_y, `VX(v12),`VY(v12)) |
    dot2(pix_x,pix_y, `VX(v13),`VY(v13)) |
    dot2(pix_x,pix_y, `VX(v14),`VY(v14)) |
    dot2(pix_x,pix_y, `VX(v15),`VY(v15));

  // Color rules, same as before:
  //   dots        -> white
  //   w-edges     -> lime (glowy green/yellow)
  //   other edges -> cyan
  //   background  -> black
  wire [1:0] R_pix = dot_pix      ? 2'b11 :
                     edge_wonly   ? 2'b01 :
                     edge_any     ? 2'b00 :
                                   2'b00;

  wire [1:0] G_pix = dot_pix      ? 2'b11 :
                     edge_wonly   ? 2'b11 :
                     edge_any     ? 2'b11 :
                                   2'b00;

  wire [1:0] B_pix = dot_pix      ? 2'b11 :
                     edge_wonly   ? 2'b00 :
                     edge_any     ? 2'b10 :
                                   2'b00;

  // drive pixel RGB during active video
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      R <= 2'b00; G <= 2'b00; B <= 2'b00;
    end else if (video_active) begin
      R <= R_pix;
      G <= G_pix;
      B <= B_pix;
    end else begin
      R <= 2'b00; G <= 2'b00; B <= 2'b00;
    end
  end

endmodule


// ============================================================
// sin/cos lookup table (64 steps), signed Q1.7
// cos(angle) = sin(angle+16)
// ============================================================
module sincos64(
  input  wire [5:0] idx,
  output reg  signed [7:0] cos_q,
  output reg  signed [7:0] sin_q
);
  reg signed [7:0] s[0:63];
  initial begin
    s[ 0]=   0; s[ 1]=  13; s[ 2]=  25; s[ 3]=  38;
    s[ 4]=  49; s[ 5]=  60; s[ 6]=  70; s[ 7]=  79;
    s[ 8]=  87; s[ 9]=  94; s[10]= 100; s[11]= 105;
    s[12]= 109; s[13]= 112; s[14]= 114; s[15]= 115;
    s[16]= 116; s[17]= 115; s[18]= 114; s[19]= 112;
    s[20]= 109; s[21]= 105; s[22]= 100; s[23]=  94;
    s[24]=  87; s[25]=  79; s[26]=  70; s[27]=  60;
    s[28]=  49; s[29]=  38; s[30]=  25; s[31]=  13;
    s[32]=   0; s[33]= -13; s[34]= -25; s[35]= -38;
    s[36]= -49; s[37]= -60; s[38]= -70; s[39]= -79;
    s[40]= -87; s[41]= -94; s[42]= -100; s[43]= -105;
    s[44]= -109; s[45]= -112; s[46]= -114; s[47]= -115;
    s[48]= -116; s[49]= -115; s[50]= -114; s[51]= -112;
    s[52]= -109; s[53]= -105; s[54]= -100; s[55]=  -94;
    s[56]=  -87; s[57]=  -79; s[58]=  -70; s[59]=  -60;
    s[60]=  -49; s[61]=  -38; s[62]=  -25; s[63]=  -13;
  end

  wire [5:0] ic = idx + 6'd16;
  always @* begin
    sin_q = s[idx];
    cos_q = s[ic];
  end
endmodule
