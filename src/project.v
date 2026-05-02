/*
 * Copyright (c) 2024 Jorge Luis Chuquimia Parra
 * SPDX-License-Identifier: Apache-2.0
 *
 * Proyecto  : CRC_FIFO — Motor CRC-32 con FIFO de 16 bytes
 * Autor     : Jorge Luis Chuquimia Parra
 * GitHub    : 27jorge05
 *
 * Descripcion:
 *   Motor CRC-32 (IEEE 802.3, polinomio 0xEDB88320 reflejado)
 *   con buffer FIFO de 16 bytes y visualizacion VGA 640x480.
 *   Sync VGA integrado directamente — un solo dominio de reloj.
 *
 * Pines:
 *   ui_in[0]   = wr       escribe uio_in en FIFO
 *   ui_in[1]   = rd       habilita lectura de registro por uio
 *   ui_in[5:2] = addr     seleccion de registro (0=status, 1-4=CRC bytes)
 *   ui_in[6]   = enable   habilita motor CRC
 *   ui_in[7]   = rst_crc  reset suave del motor
 *   uio[7:0]   = data     bus bidireccional
 *
 * Restricciones Tiny Tapeout tile 1x1:
 *   - Un solo dominio de reloj (clk 25MHz)
 *   - Sin Metal 5
 *   - Sin modulos externos (sync integrado)
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

  // ==========================================================
  // VGA timing — integrado, sin modulo externo
  // 640x480 @ 60Hz con reloj de 25MHz
  // H total: 800 ciclos (640 visible + 160 blanking)
  // V total: 525 lineas (480 visible + 45 blanking)
  // ==========================================================
  reg [9:0] h_count; // contador horizontal 0-799
  reg [9:0] v_count; // contador vertical   0-524

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      h_count <= 10'b0;
      v_count <= 10'b0;
    end else begin
      if (h_count == 10'd799) begin
        h_count <= 10'b0;
        if (v_count == 10'd524)
          v_count <= 10'b0;
        else
          v_count <= v_count + 10'd1;
      end else begin
        h_count <= h_count + 10'd1;
      end
    end
  end

  // Senales VGA derivadas de los contadores
  wire hsync        = ~(h_count >= 10'd656 && h_count < 10'd752);
  wire vsync        = ~(v_count >= 10'd490 && v_count < 10'd492);
  wire video_active = (h_count < 10'd640) && (v_count < 10'd480);
  wire [9:0] pix_x  = h_count;
  wire [9:0] pix_y  = v_count;

  wire [1:0] R;
  wire [1:0] G;
  wire [1:0] B;

  // TinyVGA PMOD — orden fijo por conector fisico
  assign uo_out = {hsync, B[0], G[0], R[0], vsync, B[1], G[1], R[1]};

  // ==========================================================
  // Pines de control
  // ==========================================================
  wire        wr      = ui_in[0];
  wire        rd      = ui_in[1];
  wire [3:0]  addr    = ui_in[5:2];
  wire        enable  = ui_in[6];
  wire        rst_crc = ui_in[7];

  reg [7:0] uio_out_reg;
  reg [7:0] uio_oe_reg;
  assign uio_out = uio_out_reg;
  assign uio_oe  = uio_oe_reg;

  // ==========================================================
  // FIFO circular 16 bytes
  // ==========================================================
  reg [7:0] fifo [0:15];
  reg [3:0] wr_ptr;
  reg [3:0] rd_ptr;

  wire       fifo_empty = (wr_ptr == rd_ptr);
  wire       fifo_full  = ((wr_ptr + 4'd1) == rd_ptr);
  wire [3:0] fifo_count = wr_ptr - rd_ptr;

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n || rst_crc) begin
      wr_ptr <= 4'b0;
    end else if (wr && enable && !fifo_full) begin
      fifo[wr_ptr] <= uio_in;
      wr_ptr        <= wr_ptr + 4'd1;
    end
  end

  // ==========================================================
  // CRC-32 combinacional desenrollado
  // Polinomio reflejado 0xEDB88320 (IEEE 802.3)
  // Ciclo A: XOR byte + pasos 0-3
  // Ciclo B: pasos 4-7 sin nuevo XOR
  // ==========================================================
  wire [31:0] crc_in_a = crc_reg ^ {24'b0, fifo[rd_ptr]};
  wire [31:0] crc_in_b = crc_reg;

  wire [31:0] a0 = crc_in_a[0] ? (crc_in_a >> 1) ^ 32'hEDB88320 : (crc_in_a >> 1);
  wire [31:0] a1 = a0[0]       ? (a0       >> 1) ^ 32'hEDB88320 : (a0       >> 1);
  wire [31:0] a2 = a1[0]       ? (a1       >> 1) ^ 32'hEDB88320 : (a1       >> 1);
  wire [31:0] a3 = a2[0]       ? (a2       >> 1) ^ 32'hEDB88320 : (a2       >> 1);

  wire [31:0] b0 = crc_in_b[0] ? (crc_in_b >> 1) ^ 32'hEDB88320 : (crc_in_b >> 1);
  wire [31:0] b1 = b0[0]       ? (b0       >> 1) ^ 32'hEDB88320 : (b0       >> 1);
  wire [31:0] b2 = b1[0]       ? (b1       >> 1) ^ 32'hEDB88320 : (b1       >> 1);
  wire [31:0] b3 = b2[0]       ? (b2       >> 1) ^ 32'hEDB88320 : (b2       >> 1);

  // ==========================================================
  // FSM — IDLE > PROCESS > FINALIZE > DONE
  // ==========================================================
  localparam IDLE     = 2'b00;
  localparam PROCESS  = 2'b01;
  localparam FINALIZE = 2'b10;
  localparam DONE     = 2'b11;

  reg [31:0] crc_reg;
  reg        crc_done;
  reg        half;
  reg [1:0]  fsm_state;

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n || rst_crc) begin
      crc_reg   <= 32'hFFFFFFFF;
      crc_done  <= 1'b0;
      rd_ptr    <= 4'b0;
      fsm_state <= IDLE;
      half      <= 1'b0;
    end else begin
      case (fsm_state)
        IDLE: begin
          crc_done <= 1'b0;
          half     <= 1'b0;
          if (!fifo_empty && enable) begin
            crc_reg   <= 32'hFFFFFFFF;
            fsm_state <= PROCESS;
          end
        end
        PROCESS: begin
          if (!fifo_empty) begin
            if (!half) begin
              crc_reg <= a3;
              half    <= 1'b1;
            end else begin
              crc_reg <= b3;
              half    <= 1'b0;
              rd_ptr  <= rd_ptr + 4'd1;
            end
          end else begin
            fsm_state <= FINALIZE;
          end
        end
        FINALIZE: begin
          crc_reg   <= ~crc_reg;
          crc_done  <= 1'b1;
          fsm_state <= DONE;
        end
        DONE: begin
          if (wr && enable) begin
            crc_done  <= 1'b0;
            fsm_state <= IDLE;
          end
        end
      endcase
    end
  end

  // ==========================================================
  // Lectura de registros
  // ==========================================================
  wire irq = crc_done | fifo_full;

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      uio_out_reg <= 8'b0;
      uio_oe_reg  <= 8'b0;
    end else if (rd && enable) begin
      uio_oe_reg <= 8'hFF;
      case (addr)
        4'd0:    uio_out_reg <= {3'b0, irq, fifo_count};
        4'd1:    uio_out_reg <= crc_reg[7:0];
        4'd2:    uio_out_reg <= crc_reg[15:8];
        4'd3:    uio_out_reg <= crc_reg[23:16];
        4'd4:    uio_out_reg <= crc_reg[31:24];
        default: uio_out_reg <= 8'b0;
      endcase
    end else begin
      uio_oe_reg  <= 8'b0;
      uio_out_reg <= 8'b0;
    end
  end

  // ==========================================================
  // Animacion VGA — divisor de reloj en dominio clk
  // 25MHz / 2^18 = ~95Hz
  // ==========================================================
  reg [17:0] clk_div;
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) clk_div <= 18'b0;
    else        clk_div <= clk_div + 18'd1;
  end
  wire [5:0] frame_ctr = clk_div[17:12];

  // ==========================================================
  // Zonas VGA
  // ==========================================================
  wire [9:0] bar_w = {6'b0, fifo_count} * 10'd40;
  wire bar_on = (pix_y >= 10'd100) && (pix_y < 10'd160) &&
                (pix_x < bar_w) && video_active;

  wire [4:0] bit_idx = pix_x[9:5];
  wire [4:0] cell_x  = pix_x[4:0];
  wire grid_on = (pix_y >= 10'd260) && (pix_y < 10'd460) &&
                 (pix_x < 10'd640) &&
                 (cell_x >= 5'd2) && (cell_x < 5'd18) &&
                 video_active;
  wire bit_on = crc_reg[bit_idx];

  wire [9:0] scan_y = {4'b0, frame_ctr} << 3;
  wire scan_on = (pix_y == scan_y) && video_active;

  wire [1:0] fsm_blk = pix_x[9:8];
  wire fsm_on = (pix_y >= 10'd180) && (pix_y < 10'd240) && video_active;

  // ==========================================================
  // Logica de color combinacional
  // ==========================================================
  reg [1:0] pR, pG, pB;

  always @(*) begin
    pR = 2'b00; pG = 2'b00; pB = 2'b00;

    if (!video_active) begin
      pR = 2'b00; pG = 2'b00; pB = 2'b00;

    end else if (pix_y < 10'd80) begin
      pR = 2'b00; pG = pix_x[8:7]; pB = 2'b11;

    end else if (bar_on) begin
      pR = 2'b00; pG = 2'b11; pB = 2'b01;

    end else if ((pix_y >= 10'd100) && (pix_y < 10'd160)) begin
      pR = 2'b00; pG = 2'b01; pB = 2'b00;

    end else if (fsm_on) begin
      case (fsm_blk)
        2'd0: case (fsm_state)
          IDLE:     begin pR=2'b01; pG=2'b01; pB=2'b01; end
          PROCESS:  begin pR=2'b00; pG=2'b11; pB=2'b00; end
          FINALIZE: begin pR=2'b11; pG=2'b11; pB=2'b00; end
          default:  begin pR=2'b00; pG=2'b00; pB=2'b11; end
        endcase
        2'd1: begin
          pR = irq ? 2'b11 : 2'b01;
          pG = 2'b00; pB = 2'b00;
        end
        2'd2: begin
          pR = 2'b00;
          pG = enable ? 2'b11 : 2'b00;
          pB = enable ? 2'b11 : 2'b00;
        end
        default: begin
          pR = rst_crc ? 2'b11 : 2'b00;
          pG = 2'b00;
          pB = rst_crc ? 2'b11 : 2'b00;
        end
      endcase

    end else if (grid_on) begin
      if (bit_on) begin
        pR = 2'b11; pG = 2'b10; pB = 2'b00;
      end else begin
        pR = 2'b00; pG = 2'b00; pB = 2'b10;
      end

    end else if (scan_on) begin
      pR = 2'b11; pG = 2'b11; pB = 2'b11;

    end else begin
      pR = 2'b00;
      pG = (pix_y[7:6] == 2'b00) ? 2'b01 : 2'b00;
      pB = 2'b00;
    end
  end

  assign R = pR;
  assign G = pG;
  assign B = pB;

  wire _unused_ok = &{ena, uio_in};

endmodule