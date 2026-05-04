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
 *   CRC procesado 1 bit por ciclo (8 ciclos/byte).
 *   Visualizacion VGA simplificada para caber en 1x1 tile IHP sg13g2:
 *     - Header solido azul
 *     - Barra FIFO (shift, sin multiplicador)
 *     - Panel FSM/IRQ/Enable/rst_crc
 *     - Display 8 bits del CRC (indexacion estatica 8:1, no mux 32:1)
 *   Un solo dominio de reloj. Sin scanner (elimina clk_div 18-bit).
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

  reg [7:0] uio_out_reg;
  reg [7:0] uio_oe_reg;
  assign uio_out = uio_out_reg;
  assign uio_oe  = uio_oe_reg;

  // ==========================================================
  // FIFO circular 8 bytes — punteros 3-bit, modulo 8 natural
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
        fifo[wr_ptr] <= uio_in;
        wr_ptr       <= wr_ptr + 3'd1;
      end
    end
  end

  // ==========================================================
  // CRC-32 — 1 bit por ciclo, 8 ciclos por byte
  // Polinomio reflejado 0xEDB88320 (IEEE 802.3)
  // Un solo nivel combinacional: mux 32-bit + XOR condicional
  // ==========================================================
  wire [31:0] crc_step = crc_work[0]
                          ? (crc_work >> 1) ^ 32'hEDB88320
                          : (crc_work >> 1);

  // ==========================================================
  // FSM — IDLE -> LOAD -> BITS -> [LOAD | FINALIZE] -> DONE
  // ==========================================================
  localparam IDLE     = 3'd0;
  localparam LOAD     = 3'd1;
  localparam BITS     = 3'd2;
  localparam FINALIZE = 3'd3;
  localparam DONE     = 3'd4;

  reg [31:0] crc_reg;
  reg [31:0] crc_work;
  reg        crc_done;
  reg [2:0]  bit_cnt;
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
              crc_reg   <= 32'hFFFFFFFF;
              fsm_state <= LOAD;
            end
          end

          LOAD: begin
            crc_work  <= crc_reg ^ {24'b0, fifo[rd_ptr]};
            rd_ptr    <= rd_ptr + 3'd1;
            bit_cnt   <= 3'd0;
            fsm_state <= BITS;
          end

          BITS: begin
            if (bit_cnt == 3'd7) begin
              crc_reg   <= crc_step;
              bit_cnt   <= 3'd0;
              fsm_state <= fifo_empty ? FINALIZE : LOAD;
            end else begin
              crc_work <= crc_step;
              bit_cnt  <= bit_cnt + 3'd1;
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

          default: fsm_state <= IDLE;

        endcase
      end
    end
  end

  // ==========================================================
  // Lectura de registros via uio
  // addr 0 = {4'b0, irq, fifo_count[2:0]}
  // addr 1-4 = bytes CRC LSB->MSB
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
  // VGA — Zonas simplificadas (menos comparadores, sin scanner,
  //        sin multiplicador, indexacion estatica del CRC)
  //
  // y   0- 79 : header azul solido
  // y  90-149 : barra FIFO  — bar_w = fifo_count << 6 (count*64px)
  // y 160-219 : panel FSM/IRQ/Enable/rst_crc  (4 bloques x 160px)
  // y 230-309 : display CRC byte bajo (crc_reg[7:0])
  //             8 celdas de 80px, indexacion pix_x[9:7] — mux 8:1
  // resto      : fondo negro
  // ==========================================================

  // --- Barra FIFO: shift en lugar de multiplicador ---
  // count*64 = count<<6, max = 7*64 = 448px < 640, sin desbordamiento
  // {1'b0, fifo_count, 6'b0} = concatenacion directa = count*64 sin multiplicador
  wire [9:0] bar_w  = {1'b0, fifo_count, 6'b0};
  wire       bar_on = (pix_y >= 10'd90)  && (pix_y < 10'd150) &&
                      (pix_x < bar_w)    && video_active;

  // --- Panel FSM/IRQ: 4 bloques de 160px (pix_x[9:8]) ---
  wire [1:0] fsm_blk = pix_x[9:8];
  wire       fsm_on  = (pix_y >= 10'd160) && (pix_y < 10'd220) && video_active;

  // --- Display CRC byte bajo ---
  // pix_x[9:7] = columna 0-7 (80px por celda, 8 celdas = 640px)
  // Indexacion estatica: mux 8:1, no mux 32:1
  // Margen interior: pix_x[6:0] entre 5 y 60
  wire [4:0] crc_col  = {2'b0, pix_x[9:7]};  // 5 bits para indexar crc_reg[31:0]
  wire [6:0] crc_cell = pix_x[6:0];
  wire       crc_on   = (pix_y >= 10'd230) && (pix_y < 10'd310) &&
                        (crc_cell >= 7'd5)  && (crc_cell < 7'd65) &&
                        video_active;
  wire       bit_on   = crc_reg[crc_col];  // mux 8:1, no 32:1

  // ==========================================================
  // Logica de color — combinacional pura, sin latches
  // Defaults al inicio garantizan que no hay latch inferido
  // ==========================================================
  reg [1:0] pR, pG, pB;

  always @(*) begin
    pR = 2'b00; pG = 2'b00; pB = 2'b00;

    if (!video_active) begin
      // Fuera de ventana activa: negro
      pR = 2'b00; pG = 2'b00; pB = 2'b00;

    end else if (pix_y < 10'd80) begin
      // Header: azul solido (sin degradado -> sin comparador pix_x)
      pR = 2'b00; pG = 2'b00; pB = 2'b11;

    end else if (bar_on) begin
      // Barra FIFO ocupada: verde brillante
      pR = 2'b00; pG = 2'b11; pB = 2'b01;

    end else if ((pix_y >= 10'd90) && (pix_y < 10'd150)) begin
      // Fondo FIFO vacio: verde oscuro
      pR = 2'b00; pG = 2'b01; pB = 2'b00;

    end else if (fsm_on) begin
      // Bloque 0: estado FSM
      // Bloque 1: IRQ
      // Bloque 2: Enable
      // Bloque 3: rst_crc
      case (fsm_blk)
        2'd0: case (fsm_state)
          IDLE:     begin pR=2'b01; pG=2'b01; pB=2'b01; end // gris
          LOAD:     begin pR=2'b11; pG=2'b11; pB=2'b00; end // amarillo
          BITS:     begin pR=2'b00; pG=2'b11; pB=2'b00; end // verde
          FINALIZE: begin pR=2'b11; pG=2'b10; pB=2'b00; end // naranja
          default:  begin pR=2'b00; pG=2'b00; pB=2'b11; end // azul DONE
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

    end else if (crc_on) begin
      // Display CRC[7:0]: naranja=1, azul oscuro=0
      if (bit_on) begin
        pR = 2'b11; pG = 2'b10; pB = 2'b00;
      end else begin
        pR = 2'b00; pG = 2'b00; pB = 2'b10;
      end

    end else begin
      // Fondo general: negro puro
      pR = 2'b00; pG = 2'b00; pB = 2'b00;
    end
  end

  assign R = pR;
  assign G = pG;
  assign B = pB;

  // ena no se usa; uio_in SI se usa en FIFO write
  wire _unused_ok = &{ena, 1'b0};

endmodule
