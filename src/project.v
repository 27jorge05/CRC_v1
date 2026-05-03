/*
 * Copyright (c) 2024 Jorge Luis Chuquimia Parra
 * SPDX-License-Identifier: Apache-2.0
 *
 * Proyecto  : CRC_FIFO — Motor CRC-32 con FIFO de 16 bytes
 * Autor     : Jorge Luis Chuquimia Parra
 * GitHub    : 27jorge05
 *
 * Descripcion:
 *   Motor CRC-32 (IEEE 802.3, 0xEDB88320) con FIFO de 16 bytes.
 *   Visualizacion VGA 640x480 en tiempo real:
 *     - Fila   0- 79 : header azul con degradado verde
 *     - Fila 100-159 : barra de ocupacion FIFO
 *     - Fila 180-239 : indicadores FSM / IRQ / Enable / rst_crc
 *     - Fila 260-339 : grid bits 15:0  del CRC (16 celdas de 32px)
 *     - Fila 360-439 : grid bits 31:16 del CRC (16 celdas de 32px)
 *     - Scanner      : linea blanca animada (toda la pantalla)
 *   Un solo dominio de reloj (clk). Sin posedge vsync.
 *
 * Pines:
 *   ui_in[0]   = wr       escribe uio_in en FIFO
 *   ui_in[1]   = rd       habilita lectura de registro por uio
 *   ui_in[5:2] = addr     registro (0=status, 1-4=CRC bytes)
 *   ui_in[6]   = enable   habilita motor CRC
 *   ui_in[7]   = rst_crc  reset suave del motor
 *   uio[7:0]   = data     bus bidireccional
 */

`default_nettype none

module tt_um_27jorge05_crc_fifo(
  input  wire [7:0] ui_in,    // Dedicated inputs
  output wire [7:0] uo_out,   // Dedicated outputs
  input  wire [7:0] uio_in,   // IOs: Input path
  output wire [7:0] uio_out,  // IOs: Output path
  output wire [7:0] uio_oe,   // IOs: Enable path (active high: 0=input, 1=output)
  input  wire       ena,      // always 1 when the design is powered
  input  wire       clk,      // clock
  input  wire       rst_n     // reset_n - low to reset
);

  // ==========================================================
  // VGA signals
  // ==========================================================
  wire hsync;
  wire vsync;
  wire [1:0] R;
  wire [1:0] G;
  wire [1:0] B;
  wire video_active;
  wire [9:0] pix_x;
  wire [9:0] pix_y;

  // TinyVGA PMOD - orden fijo por conector fisico
  assign uo_out = {hsync, B[0], G[0], R[0], vsync, B[1], G[1], R[1]};

  // hvsync_generator - modulo externo en hvsync_generator.v
  hvsync_generator hvsync_gen(
    .clk(clk),
    .reset(~rst_n),
    .hsync(hsync),
    .vsync(vsync),
    .display_on(video_active),
    .hpos(pix_x),
    .vpos(pix_y)
  );

  // ==========================================================
  // Pines de control
  // ==========================================================
  wire        wr      = ui_in[0];
  wire        rd      = ui_in[1];
  wire [3:0]  addr    = ui_in[5:2];
  wire        enable  = ui_in[6];
  wire        rst_crc = ui_in[7];

  // Bus uio bidireccional
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
    if (!rst_n) begin
      wr_ptr <= 4'b0;
    end else begin
      if (rst_crc) begin
        wr_ptr <= 4'b0;
      end else if (wr && enable && !fifo_full) begin
        fifo[wr_ptr] <= uio_in;   // uio_in usado aqui, NO va en _unused_ok
        wr_ptr        <= wr_ptr + 4'd1;
      end
    end
  end

  // ==========================================================
  // CRC-32 combinacional — 8 pasos, 1 byte por ciclo
  // Polinomio reflejado 0xEDB88320 (IEEE 802.3)
  // Cada paso: LSB=1 -> (c>>1) XOR poly; LSB=0 -> (c>>1)
  // ==========================================================
  wire [31:0] crc_in = crc_reg ^ {24'b0, fifo[rd_ptr]};

  wire [31:0] s0 = crc_in[0] ? (crc_in >> 1) ^ 32'hEDB88320 : (crc_in >> 1);
  wire [31:0] s1 = s0[0]     ? (s0     >> 1) ^ 32'hEDB88320 : (s0     >> 1);
  wire [31:0] s2 = s1[0]     ? (s1     >> 1) ^ 32'hEDB88320 : (s1     >> 1);
  wire [31:0] s3 = s2[0]     ? (s2     >> 1) ^ 32'hEDB88320 : (s2     >> 1);
  wire [31:0] s4 = s3[0]     ? (s3     >> 1) ^ 32'hEDB88320 : (s3     >> 1);
  wire [31:0] s5 = s4[0]     ? (s4     >> 1) ^ 32'hEDB88320 : (s4     >> 1);
  wire [31:0] s6 = s5[0]     ? (s5     >> 1) ^ 32'hEDB88320 : (s5     >> 1);
  wire [31:0] s7 = s6[0]     ? (s6     >> 1) ^ 32'hEDB88320 : (s6     >> 1);

  // ==========================================================
  // FSM — IDLE > PROCESS > FINALIZE > DONE
  // Reset async: negedge rst_n
  // Reset suave: rst_crc sincrono (dentro del else)
  // SIN posedge vsync — un solo dominio de reloj
  // ==========================================================
  localparam IDLE     = 2'b00;
  localparam PROCESS  = 2'b01;
  localparam FINALIZE = 2'b10;
  localparam DONE     = 2'b11;

  reg [31:0] crc_reg;
  reg        crc_done;
  reg [1:0]  fsm_state;

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      crc_reg   <= 32'hFFFFFFFF;
      crc_done  <= 1'b0;
      rd_ptr    <= 4'b0;
      fsm_state <= IDLE;
    end else begin
      if (rst_crc) begin
        crc_reg   <= 32'hFFFFFFFF;
        crc_done  <= 1'b0;
        rd_ptr    <= 4'b0;
        fsm_state <= IDLE;
      end else begin
        case (fsm_state)
          IDLE: begin
            crc_done <= 1'b0;
            if (!fifo_empty && enable) begin
              crc_reg   <= 32'hFFFFFFFF;
              fsm_state <= PROCESS;
            end
          end
          PROCESS: begin
            if (!fifo_empty) begin
              crc_reg <= s7;
              rd_ptr  <= rd_ptr + 4'd1;
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
  end

  // ==========================================================
  // Lectura de registros via uio
  // addr 0 = status {3b0, irq, fifo_count}
  // addr 1 = CRC byte 0 LSB
  // addr 2 = CRC byte 1
  // addr 3 = CRC byte 2
  // addr 4 = CRC byte 3 MSB
  // ==========================================================
  wire irq = crc_done | fifo_full;

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      uio_out_reg <= 8'b0;
      uio_oe_reg  <= 8'b0;
    end else begin
      if (rd && enable) begin
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
  end

  // ==========================================================
  // Divisor de reloj para animacion VGA
  // 25MHz / 2^12 = ~6100 incrementos/seg para frame_ctr
  // frame_ctr wrappea en 60 -> scan_y max = 59*8 = 472 < 480
  // SIN posedge vsync — un solo dominio de reloj
  // ==========================================================
  reg [17:0] clk_div;
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) clk_div <= 18'b0;
    else        clk_div <= clk_div + 18'd1;
  end

  wire [5:0] frame_ctr = clk_div[17:12];

  // Limitar frame_mod a 0-59 para que scan_y nunca supere 472 (< 480)
  wire [5:0] frame_mod = (frame_ctr >= 6'd60) ? (frame_ctr - 6'd60) : frame_ctr;
  wire [9:0] scan_y    = {4'b0, frame_mod} << 3;   // max = 59*8 = 472

  // ==========================================================
  // Zonas de visualizacion VGA (640x480)
  //
  // fila   0- 79 : header azul con degradado verde
  // fila 100-159 : barra de ocupacion FIFO
  // fila 180-239 : indicadores FSM / IRQ / Enable / rst_crc
  // fila 260-339 : grid bits 15:0  del CRC (16 celdas x 32px = 512px)
  // fila 360-439 : grid bits 31:16 del CRC (16 celdas x 32px = 512px)
  // scanner      : linea blanca animada (toda la pantalla)
  // ==========================================================

  // --- Barra FIFO: max 15*40 = 600px < 640, sin desbordamiento ---
  wire [9:0] bar_w = {6'b0, fifo_count} * 10'd40;
  wire bar_on = (pix_y >= 10'd100) && (pix_y < 10'd160) &&
                (pix_x < bar_w) && video_active;

  // --- Grid CRC: 2 filas de 16 bits, celdas de 32px ---
  // Fila A: y=260-339 -> bits 15:0
  // Fila B: y=360-439 -> bits 31:16
  // pix_x[8:5] = columna 0-15 (32px por celda, 16*32 = 512px < 640)
  // pix_x[4:0] = posicion dentro de celda (margen interior 2-17px)
  wire in_grid_r1 = (pix_y >= 10'd260) && (pix_y < 10'd340) && video_active;
  wire in_grid_r2 = (pix_y >= 10'd360) && (pix_y < 10'd440) && video_active;
  wire [3:0] col_idx  = pix_x[8:5];                            // columna 0-15
  wire [4:0] bit_idx  = in_grid_r2 ? (5'd16 + {1'b0, col_idx}) // bits 16-31
                                   : {1'b0, col_idx};           // bits  0-15
  wire [4:0] cell_x   = pix_x[4:0];                            // offset en celda
  wire grid_on = (in_grid_r1 || in_grid_r2) &&
                 (pix_x < 10'd512) &&                          // solo 16 celdas validas
                 (cell_x >= 5'd2) && (cell_x < 5'd18);
  wire bit_on  = crc_reg[bit_idx];

  // --- Scanner ---
  wire scan_on = (pix_y == scan_y) && video_active;

  // --- Indicadores FSM: 4 bloques de 160px (0-159, 160-319, 320-479, 480-639) ---
  wire [1:0] fsm_blk = pix_x[9:8];
  wire fsm_on = (pix_y >= 10'd180) && (pix_y < 10'd240) && video_active;

  // ==========================================================
  // Logica de color — combinacional pura, sin latches
  // Defaults al inicio del always(*) evitan inferencia de latches
  // ==========================================================
  reg [1:0] pR, pG, pB;

  always @(*) begin
    // Defaults — fondo negro fuera de todas las zonas
    pR = 2'b00; pG = 2'b00; pB = 2'b00;

    if (!video_active) begin
      pR = 2'b00; pG = 2'b00; pB = 2'b00;

    end else if (pix_y < 10'd80) begin
      // Header: azul solido con degradado verde segun posicion X
      pR = 2'b00; pG = pix_x[8:7]; pB = 2'b11;

    end else if (bar_on) begin
      // Barra FIFO llena: verde brillante con toque azul
      pR = 2'b00; pG = 2'b11; pB = 2'b01;

    end else if ((pix_y >= 10'd100) && (pix_y < 10'd160)) begin
      // Fondo de la barra FIFO (parte vacia): verde oscuro
      pR = 2'b00; pG = 2'b01; pB = 2'b00;

    end else if (fsm_on) begin
      // Bloque 0: color segun estado FSM
      // Bloque 1: IRQ (rojo si activo)
      // Bloque 2: Enable (cian si activo)
      // Bloque 3: rst_crc (magenta si activo)
      case (fsm_blk)
        2'd0: case (fsm_state)
          IDLE:     begin pR=2'b01; pG=2'b01; pB=2'b01; end  // gris
          PROCESS:  begin pR=2'b00; pG=2'b11; pB=2'b00; end  // verde
          FINALIZE: begin pR=2'b11; pG=2'b11; pB=2'b00; end  // amarillo
          default:  begin pR=2'b00; pG=2'b00; pB=2'b11; end  // azul (DONE)
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
      // Grid CRC: naranja si bit=1, azul oscuro si bit=0
      if (bit_on) begin
        pR = 2'b11; pG = 2'b10; pB = 2'b00;   // naranja
      end else begin
        pR = 2'b00; pG = 2'b00; pB = 2'b10;   // azul oscuro
      end

    end else if (scan_on) begin
      // Scanner: linea blanca animada
      pR = 2'b11; pG = 2'b11; pB = 2'b11;

    end else begin
      // Fondo general: toque verde muy tenue en la parte superior
      pR = 2'b00;
      pG = (pix_y[7:6] == 2'b00) ? 2'b01 : 2'b00;
      pB = 2'b00;
    end
  end

  assign R = pR;
  assign G = pG;
  assign B = pB;

  // ena no se usa en logica; uio_in SI se usa (FIFO write), no va aqui
  wire _unused_ok = &{ena, 1'b0};

endmodule