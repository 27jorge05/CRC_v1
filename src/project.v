/*
 * Copyright (c) 2024 Jorge Luis Chuquimia Parra
 * SPDX-License-Identifier: Apache-2.0
 *
 * Proyecto  : CRC_FIFO — Motor CRC-32 con FIFO de 8 bytes
 * Autor     : Jorge Luis Chuquimia Parra
 * GitHub    : 27jorge05
 *
 * Descripcion:
 *   Motor CRC-32 (IEEE 802.3, 0xEDB88320) con FIFO de 8 bytes.
 *   CRC procesado 1 bit por ciclo (8 ciclos/byte) para minimizar
 *   el area combinacional y caber en 1x1 tile IHP sg13g2.
 *   Visualizacion VGA en tiempo real. Un solo dominio de reloj.
 *
 * FSM: IDLE -> LOAD -> BITS(x8) -> [LOAD | FINALIZE] -> DONE
 *
 * Pines:
 *   ui_in[0]   = wr       escribe uio_in en FIFO
 *   ui_in[1]   = rd       habilita lectura de registro por uio
 *   ui_in[5:2] = addr     registro (0=status, 1-4=CRC bytes)
 *   ui_in[6]   = enable   habilita motor CRC
 *   ui_in[7]   = rst_crc  reset suave del motor
 *   uio[7:0]   = data     bus bidireccional
 *
 * Registro 0 (status): {4'b0, irq, fifo_count[2:0]}
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
  // FIFO circular 8 bytes (reducido de 16 para caber en 1x1)
  // Punteros de 3 bits — aritmetica modulo 8 natural
  // ==========================================================
  reg [7:0] fifo [0:7];
  reg [2:0] wr_ptr;
  reg [2:0] rd_ptr;

  wire       fifo_empty = (wr_ptr == rd_ptr);
  wire       fifo_full  = ((wr_ptr + 3'd1) == rd_ptr);
  wire [2:0] fifo_count = wr_ptr - rd_ptr;

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      wr_ptr <= 3'b0;
    end else begin
      if (rst_crc) begin
        wr_ptr <= 3'b0;
      end else if (wr && enable && !fifo_full) begin
        fifo[wr_ptr] <= uio_in;   // uio_in se usa aqui — no va en _unused_ok
        wr_ptr       <= wr_ptr + 3'd1;
      end
    end
  end

  // ==========================================================
  // CRC-32 — 1 bit por ciclo, 8 ciclos por byte
  // Polinomio reflejado 0xEDB88320 (IEEE 802.3)
  // Un solo nivel combinacional: ~32 celdas vs ~500 del metodo 8-bit
  //
  // Paso unico:  LSB=1 -> (w>>1) XOR 0xEDB88320
  //              LSB=0 -> (w>>1)
  // ==========================================================
  wire [31:0] crc_step = crc_work[0]
                          ? (crc_work >> 1) ^ 32'hEDB88320
                          : (crc_work >> 1);

  // ==========================================================
  // FSM — IDLE -> LOAD -> BITS -> [LOAD | FINALIZE] -> DONE
  //
  // LOAD    (1 ciclo) : XOR byte actual con crc_reg en crc_work,
  //                     avanza rd_ptr, reinicia bit_cnt
  // BITS    (8 ciclos): desplaza crc_work 1 bit por ciclo;
  //                     al terminar guarda en crc_reg
  // FINALIZE(1 ciclo) : complementa crc_reg, levanta crc_done
  // DONE              : espera nuevo dato para reiniciar
  // ==========================================================
  localparam IDLE     = 3'd0;
  localparam LOAD     = 3'd1;
  localparam BITS     = 3'd2;
  localparam FINALIZE = 3'd3;
  localparam DONE     = 3'd4;

  reg [31:0] crc_reg;    // resultado CRC acumulado
  reg [31:0] crc_work;   // registro de trabajo durante BITS
  reg        crc_done;
  reg [2:0]  bit_cnt;    // contador de bits dentro del byte actual (0-7)
  reg [2:0]  fsm_state;

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      crc_reg   <= 32'hFFFFFFFF;
      crc_work  <= 32'h0;
      crc_done  <= 1'b0;
      rd_ptr    <= 3'b0;
      bit_cnt   <= 3'b0;
      fsm_state <= IDLE;
    end else begin
      if (rst_crc) begin
        crc_reg   <= 32'hFFFFFFFF;
        crc_work  <= 32'h0;
        crc_done  <= 1'b0;
        rd_ptr    <= 3'b0;
        bit_cnt   <= 3'b0;
        fsm_state <= IDLE;
      end else begin
        case (fsm_state)

          IDLE: begin
            crc_done <= 1'b0;
            if (!fifo_empty && enable) begin
              crc_reg   <= 32'hFFFFFFFF;  // inicializar CRC al empezar mensaje
              fsm_state <= LOAD;
            end
          end

          LOAD: begin
            // XOR el byte FIFO con el CRC actual; avanzar rd_ptr
            crc_work  <= crc_reg ^ {24'b0, fifo[rd_ptr]};
            rd_ptr    <= rd_ptr + 3'd1;
            bit_cnt   <= 3'd0;
            fsm_state <= BITS;
          end

          BITS: begin
            if (bit_cnt == 3'd7) begin
              // Ultimo bit del byte: guardar resultado en crc_reg
              crc_reg   <= crc_step;
              bit_cnt   <= 3'd0;
              // Si hay mas bytes continuar, sino finalizar
              fsm_state <= fifo_empty ? FINALIZE : LOAD;
            end else begin
              crc_work <= crc_step;
              bit_cnt  <= bit_cnt + 3'd1;
            end
          end

          FINALIZE: begin
            crc_reg   <= ~crc_reg;    // complemento final IEEE 802.3
            crc_done  <= 1'b1;
            fsm_state <= DONE;
          end

          DONE: begin
            // Esperar nuevo dato para reiniciar el motor
            if (wr && enable) begin
              crc_done  <= 1'b0;
              fsm_state <= IDLE;
            end
          end

          default: fsm_state <= IDLE;

        endcase
      end
    end
  end

  // ==========================================================
  // Lectura de registros via uio
  // addr 0 = status: {4'b0, irq, fifo_count[2:0]}
  // addr 1 = CRC byte 0 (LSB)
  // addr 2 = CRC byte 1
  // addr 3 = CRC byte 2
  // addr 4 = CRC byte 3 (MSB)
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
          4'd0:    uio_out_reg <= {4'b0, irq, fifo_count};
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
  // frame_ctr wrappea en 60 -> scan_y max = 59*8 = 472 < 480
  // ==========================================================
  reg [17:0] clk_div;
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) clk_div <= 18'b0;
    else        clk_div <= clk_div + 18'd1;
  end

  wire [5:0] frame_ctr = clk_div[17:12];
  wire [5:0] frame_mod = (frame_ctr >= 6'd60) ? (frame_ctr - 6'd60) : frame_ctr;
  wire [9:0] scan_y    = {4'b0, frame_mod} << 3;   // max = 59*8 = 472

  // ==========================================================
  // Zonas de visualizacion VGA (640x480)
  //
  // fila   0- 79 : header azul con degradado verde
  // fila 100-159 : barra de ocupacion FIFO (8 slots x 80px = 640px max)
  // fila 180-239 : indicadores FSM / IRQ / Enable / rst_crc
  // fila 260-339 : grid bits 15:0  del CRC (16 celdas x 32px = 512px)
  // fila 360-439 : grid bits 31:16 del CRC (16 celdas x 32px = 512px)
  // scanner      : linea blanca animada
  // ==========================================================

  // --- Barra FIFO: 8 slots x 80px = 640px max, sin desbordamiento ---
  wire [9:0] bar_w = {7'b0, fifo_count} * 10'd80;
  wire bar_on = (pix_y >= 10'd100) && (pix_y < 10'd160) &&
                (pix_x < bar_w) && video_active;

  // --- Grid CRC: 2 filas de 16 bits, celdas de 32px ---
  // Fila A: y=260-339 -> bits 15:0
  // Fila B: y=360-439 -> bits 31:16
  wire in_grid_r1 = (pix_y >= 10'd260) && (pix_y < 10'd340) && video_active;
  wire in_grid_r2 = (pix_y >= 10'd360) && (pix_y < 10'd440) && video_active;
  wire [3:0] col_idx = pix_x[8:5];
  wire [4:0] bit_idx = in_grid_r2 ? (5'd16 + {1'b0, col_idx}) : {1'b0, col_idx};
  wire [4:0] cell_x  = pix_x[4:0];
  wire grid_on = (in_grid_r1 || in_grid_r2) &&
                 (pix_x < 10'd512) &&
                 (cell_x >= 5'd2) && (cell_x < 5'd18);
  wire bit_on  = crc_reg[bit_idx];

  // --- Scanner ---
  wire scan_on = (pix_y == scan_y) && video_active;

  // --- Indicadores FSM: 4 bloques de 160px ---
  wire [1:0] fsm_blk = pix_x[9:8];
  wire fsm_on = (pix_y >= 10'd180) && (pix_y < 10'd240) && video_active;

  // ==========================================================
  // Logica de color — combinacional pura, sin latches
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
          IDLE:     begin pR=2'b01; pG=2'b01; pB=2'b01; end  // gris
          LOAD:     begin pR=2'b11; pG=2'b11; pB=2'b00; end  // amarillo
          BITS:     begin pR=2'b00; pG=2'b11; pB=2'b00; end  // verde
          FINALIZE: begin pR=2'b11; pG=2'b10; pB=2'b00; end  // naranja
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
      if (bit_on) begin
        pR = 2'b11; pG = 2'b10; pB = 2'b00;   // naranja
      end else begin
        pR = 2'b00; pG = 2'b00; pB = 2'b10;   // azul oscuro
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

  // ena no se usa en logica; uio_in SI se usa (FIFO write)
  wire _unused_ok = &{ena, 1'b0};

endmodule