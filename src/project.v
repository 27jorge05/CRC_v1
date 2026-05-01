/*
 * Copyright (c) 2024 Jorge Luis Chuquimia Parra
 * SPDX-License-Identifier: Apache-2.0
 * CRC_FIFO: Motor CRC-32 con FIFO de 16 bytes x 2 canales
 */

`default_nettype none

module tt_um_27jorge05_crc_fifo(
  input  wire [7:0] ui_in,
  output wire [7:0] uo_out,
  input  wire [7:0] uio_in,
  output wire [7:0] uio_out,
  output wire [7:0] uio_oe,
  input  wire       ena,
  input  wire       clk,
  input  wire       rst_n
);

  // VGA signals
  wire hsync;
  wire vsync;
  wire [1:0] R;
  wire [1:0] G;
  wire [1:0] B;
  wire video_active;
  wire [9:0] pix_x;
  wire [9:0] pix_y;

  assign uo_out  = {hsync, B[0], G[0], R[0], vsync, B[1], G[1], R[1]};
  assign uio_out = 8'b0;
  assign uio_oe  = 8'b0;

  // Interfaz control
  wire       wr     = ui_in[0];
  wire [3:0] addr   = ui_in[5:2];
  wire       enable = ui_in[6];
  wire       ch_sel = ui_in[7];

  hvsync_generator hvsync_gen(
    .clk(clk),
    .reset(~rst_n),
    .hsync(hsync),
    .vsync(vsync),
    .display_on(video_active),
    .hpos(pix_x),
    .vpos(pix_y)
  );

  // =========================================================
  // FIFO 16 bytes — Canal 0
  // =========================================================
  reg [7:0] fifo0 [0:15];
  reg [3:0] fifo0_wr_ptr;
  reg [3:0] fifo0_rd_ptr;
  wire      fifo0_empty = (fifo0_wr_ptr == fifo0_rd_ptr);
  wire      fifo0_full  = ((fifo0_wr_ptr + 4'd1) == fifo0_rd_ptr);
  wire [3:0] fifo0_count = fifo0_wr_ptr - fifo0_rd_ptr;

  // FIFO 16 bytes — Canal 1
  reg [7:0] fifo1 [0:15];
  reg [3:0] fifo1_wr_ptr;
  reg [3:0] fifo1_rd_ptr;
  wire      fifo1_empty = (fifo1_wr_ptr == fifo1_rd_ptr);
  wire      fifo1_full  = ((fifo1_wr_ptr + 4'd1) == fifo1_rd_ptr);
  wire [3:0] fifo1_count = fifo1_wr_ptr - fifo1_rd_ptr;

  // =========================================================
  // CRC-32 combinacional
  // Usamos assign directo sin función para evitar BLKSEQ warning
  // =========================================================
  // Paso intermedio: XOR inicial
  wire [31:0] crc0_xor = crc0_reg ^ {24'b0, fifo0[fifo0_rd_ptr]};
  wire [31:0] crc1_xor = crc1_reg ^ {24'b0, fifo1[fifo1_rd_ptr]};

  // Árbol CRC desenrollado 8 iteraciones — canal 0
  wire [31:0] c0_s0 = crc0_xor[0] ? (crc0_xor >> 1) ^ 32'hEDB88320 : crc0_xor >> 1;
  wire [31:0] c0_s1 = c0_s0[0]    ? (c0_s0    >> 1) ^ 32'hEDB88320 : c0_s0    >> 1;
  wire [31:0] c0_s2 = c0_s1[0]    ? (c0_s1    >> 1) ^ 32'hEDB88320 : c0_s1    >> 1;
  wire [31:0] c0_s3 = c0_s2[0]    ? (c0_s2    >> 1) ^ 32'hEDB88320 : c0_s2    >> 1;
  wire [31:0] c0_s4 = c0_s3[0]    ? (c0_s3    >> 1) ^ 32'hEDB88320 : c0_s3    >> 1;
  wire [31:0] c0_s5 = c0_s4[0]    ? (c0_s4    >> 1) ^ 32'hEDB88320 : c0_s4    >> 1;
  wire [31:0] c0_s6 = c0_s5[0]    ? (c0_s5    >> 1) ^ 32'hEDB88320 : c0_s5    >> 1;
  wire [31:0] c0_s7 = c0_s6[0]    ? (c0_s6    >> 1) ^ 32'hEDB88320 : c0_s6    >> 1;

  // Árbol CRC desenrollado — canal 1
  wire [31:0] c1_s0 = crc1_xor[0] ? (crc1_xor >> 1) ^ 32'hEDB88320 : crc1_xor >> 1;
  wire [31:0] c1_s1 = c1_s0[0]    ? (c1_s0    >> 1) ^ 32'hEDB88320 : c1_s0    >> 1;
  wire [31:0] c1_s2 = c1_s1[0]    ? (c1_s1    >> 1) ^ 32'hEDB88320 : c1_s1    >> 1;
  wire [31:0] c1_s3 = c1_s2[0]    ? (c1_s2    >> 1) ^ 32'hEDB88320 : c1_s2    >> 1;
  wire [31:0] c1_s4 = c1_s3[0]    ? (c1_s3    >> 1) ^ 32'hEDB88320 : c1_s3    >> 1;
  wire [31:0] c1_s5 = c1_s4[0]    ? (c1_s4    >> 1) ^ 32'hEDB88320 : c1_s4    >> 1;
  wire [31:0] c1_s6 = c1_s5[0]    ? (c1_s5    >> 1) ^ 32'hEDB88320 : c1_s5    >> 1;
  wire [31:0] c1_s7 = c1_s6[0]    ? (c1_s6    >> 1) ^ 32'hEDB88320 : c1_s6    >> 1;

  // =========================================================
  // FSM
  // =========================================================
  localparam IDLE     = 2'b00;
  localparam PROCESS  = 2'b01;
  localparam FINALIZE = 2'b10;
  localparam DONE     = 2'b11;

  reg [31:0] crc0_reg;
  reg        crc0_done;
  reg [1:0]  fsm0_state;

  reg [31:0] crc1_reg;
  reg        crc1_done;
  reg [1:0]  fsm1_state;

  // FIFO Write canal 0
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) fifo0_wr_ptr <= 4'b0;
    else if (wr && enable && !ch_sel && !fifo0_full) begin
      fifo0[fifo0_wr_ptr] <= uio_in;
      fifo0_wr_ptr <= fifo0_wr_ptr + 4'd1;
    end
  end

  // FIFO Write canal 1
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) fifo1_wr_ptr <= 4'b0;
    else if (wr && enable && ch_sel && !fifo1_full) begin
      fifo1[fifo1_wr_ptr] <= uio_in;
      fifo1_wr_ptr <= fifo1_wr_ptr + 4'd1;
    end
  end

  // FSM + CRC canal 0
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      crc0_reg     <= 32'hFFFFFFFF;
      crc0_done    <= 1'b0;
      fifo0_rd_ptr <= 4'b0;
      fsm0_state   <= IDLE;
    end else begin
      case (fsm0_state)
        IDLE: begin
          crc0_done <= 1'b0;
          if (!fifo0_empty && enable) begin
            crc0_reg   <= 32'hFFFFFFFF;
            fsm0_state <= PROCESS;
          end
        end
        PROCESS: begin
          if (!fifo0_empty) begin
            crc0_reg     <= c0_s7;           // resultado combinacional
            fifo0_rd_ptr <= fifo0_rd_ptr + 4'd1;
          end else begin
            fsm0_state <= FINALIZE;
          end
        end
        FINALIZE: begin
          crc0_reg   <= ~crc0_reg;
          crc0_done  <= 1'b1;
          fsm0_state <= DONE;
        end
        DONE: begin
          if (wr && enable && !ch_sel) begin
            crc0_done  <= 1'b0;
            fsm0_state <= IDLE;
          end
        end
      endcase
    end
  end

  // FSM + CRC canal 1
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      crc1_reg     <= 32'hFFFFFFFF;
      crc1_done    <= 1'b0;
      fifo1_rd_ptr <= 4'b0;
      fsm1_state   <= IDLE;
    end else begin
      case (fsm1_state)
        IDLE: begin
          crc1_done <= 1'b0;
          if (!fifo1_empty && enable) begin
            crc1_reg   <= 32'hFFFFFFFF;
            fsm1_state <= PROCESS;
          end
        end
        PROCESS: begin
          if (!fifo1_empty) begin
            crc1_reg     <= c1_s7;
            fifo1_rd_ptr <= fifo1_rd_ptr + 4'd1;
          end else begin
            fsm1_state <= FINALIZE;
          end
        end
        FINALIZE: begin
          crc1_reg   <= ~crc1_reg;
          crc1_done  <= 1'b1;
          fsm1_state <= DONE;
        end
        DONE: begin
          if (wr && enable && ch_sel) begin
            crc1_done  <= 1'b0;
            fsm1_state <= IDLE;
          end
        end
      endcase
    end
  end

  wire irq = crc0_done | crc1_done | fifo0_full | fifo1_full;

  // =========================================================
  // VGA visualización
  // =========================================================
  reg [7:0] frame_counter;
  always @(posedge vsync or negedge rst_n) begin
    if (!rst_n) frame_counter <= 8'b0;
    else        frame_counter <= frame_counter + 8'd1;
  end

  // Barras FIFO escaladas: count max=15, escala *40 → max 600px
  wire [9:0] bar0_width = {6'b0, fifo0_count} * 10'd40;
  wire [9:0] bar1_width = {6'b0, fifo1_count} * 10'd40;

  wire bar0_active = (pix_y >= 10'd100 && pix_y < 10'd160) &&
                     (pix_x < bar0_width) && video_active;
  wire bar1_active = (pix_y >= 10'd220 && pix_y < 10'd280) &&
                     (pix_x < bar1_width) && video_active;

  // Grid CRC bits
  wire [4:0] crc_bit_idx = pix_x[9:5];
  wire [4:0] crc_cell_x  = pix_x[4:0];
  wire crc_grid_active   = (pix_y >= 10'd340 && pix_y < 10'd460) &&
                           (pix_x < 10'd640) &&
                           (crc_cell_x >= 5'd2 && crc_cell_x < 5'd18) &&
                           video_active;

  wire [31:0] crc_show   = ch_sel ? crc1_reg : crc0_reg;
  wire        crc_bit_on = crc_show[crc_bit_idx];

  wire [9:0] scan_line   = {2'b0, frame_counter[5:0], 2'b0};
  wire       scanner_act = (pix_y == (scan_line & 10'd479)) && video_active;

  wire [1:0] fsm_cell = pix_x[9:8];
  wire       fsm_zone = (pix_y >= 10'd290 && pix_y < 10'd330) && video_active;

  // Color logic
  reg [1:0] pix_R, pix_G, pix_B;

  always @(*) begin
    pix_R = 2'b00; pix_G = 2'b00; pix_B = 2'b00;

    if (!video_active) begin
      pix_R = 2'b00; pix_G = 2'b00; pix_B = 2'b00;

    end else if (pix_y < 10'd80) begin
      pix_R = 2'b00; pix_G = pix_x[8:7]; pix_B = 2'b11;

    end else if (bar0_active) begin
      pix_R = 2'b00; pix_G = 2'b11; pix_B = 2'b01;

    end else if (pix_y >= 10'd100 && pix_y < 10'd160) begin
      pix_R = 2'b00; pix_G = 2'b01; pix_B = 2'b00;

    end else if (bar1_active) begin
      pix_R = 2'b00; pix_G = 2'b11; pix_B = 2'b11;

    end else if (pix_y >= 10'd220 && pix_y < 10'd280) begin
      pix_R = 2'b00; pix_G = 2'b00; pix_B = 2'b01;

    end else if (fsm_zone) begin
      case (fsm_cell)
        2'd0: case (fsm0_state)
          IDLE:     begin pix_R=2'b01; pix_G=2'b01; pix_B=2'b01; end
          PROCESS:  begin pix_R=2'b00; pix_G=2'b11; pix_B=2'b00; end
          FINALIZE: begin pix_R=2'b11; pix_G=2'b11; pix_B=2'b00; end
          default:  begin pix_R=2'b00; pix_G=2'b00; pix_B=2'b11; end
        endcase
        2'd1: case (fsm1_state)
          IDLE:     begin pix_R=2'b01; pix_G=2'b01; pix_B=2'b01; end
          PROCESS:  begin pix_R=2'b00; pix_G=2'b11; pix_B=2'b00; end
          FINALIZE: begin pix_R=2'b11; pix_G=2'b11; pix_B=2'b00; end
          default:  begin pix_R=2'b00; pix_G=2'b00; pix_B=2'b11; end
        endcase
        2'd2: begin
          pix_R = irq ? 2'b11 : 2'b00;
          pix_G = 2'b00; pix_B = 2'b00;
        end
        default: begin
          pix_R = 2'b00;
          pix_G = enable ? 2'b11 : 2'b00;
          pix_B = enable ? 2'b11 : 2'b00;
        end
      endcase

    end else if (crc_grid_active) begin
      if (crc_bit_on) begin
        pix_R = 2'b11; pix_G = 2'b10; pix_B = 2'b00;
      end else begin
        pix_R = 2'b00; pix_G = 2'b00; pix_B = 2'b10;
      end

    end else if (scanner_act) begin
      pix_R = 2'b11; pix_G = 2'b11; pix_B = 2'b11;

    end else begin
      pix_R = 2'b00;
      pix_G = (pix_y[7:6] == 2'b00) ? 2'b01 : 2'b00;
      pix_B = 2'b00;
    end
  end

  assign R = pix_R;
  assign G = pix_G;
  assign B = pix_B;

  wire _unused_ok = &{ena, addr, uio_in};

endmodule
