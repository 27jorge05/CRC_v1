/*
 * Copyright (c) 2024 Jorge Luis Chuquimia Parra
 * SPDX-License-Identifier: Apache-2.0
 * CRC_FIFO: Motor CRC-32 con FIFO de 16 bytes
 * Verificado: sin doble declaracion, CRC correcto, compatible VGA playground
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

  // Control signals
  wire wr     = ui_in[0];
  wire enable = ui_in[6];

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
  // FIFO 16 bytes — punteros 4 bits
  // =========================================================
  reg [7:0] fifo     [0:15];   // 16 x 8 bits = 128 flip-flops
  reg [3:0] wr_ptr;             // puntero escritura
  reg [3:0] rd_ptr;             // puntero lectura  ← UNA SOLA declaracion
  wire      fifo_empty = (wr_ptr == rd_ptr);
  wire      fifo_full  = ((wr_ptr + 4'd1) == rd_ptr);
  wire [3:0] fifo_count = wr_ptr - rd_ptr;

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      wr_ptr <= 4'b0;
    end else if (wr && enable && !fifo_full) begin
      fifo[wr_ptr] <= uio_in;
      wr_ptr <= wr_ptr + 4'd1;
    end
  end

  // =========================================================
  // CRC-32 — 4 pasos por ciclo, 2 ciclos por byte
  //
  // Polinomio reflejado: 0xEDB88320
  //
  // Ciclo A (half=0): incorpora byte, procesa bits[3:0]
  //   entrada = crc_reg XOR {24'b0, byte_actual}
  //   aplica 4 pasos CRC → guarda en crc_reg
  //
  // Ciclo B (half=1): procesa bits[7:4] (sin XOR — byte ya incorporado)
  //   entrada = crc_reg (ya tiene XOR del ciclo A)
  //   aplica 4 pasos CRC → guarda en crc_reg, avanza rd_ptr
  //
  // Resultado: 1 byte procesado cada 2 ciclos de reloj
  // =========================================================
  reg [31:0] crc_reg;
  reg        crc_done;
  reg        half;       // 0=primera mitad byte, 1=segunda mitad

  // Wire para la entrada del primer ciclo: XOR con byte actual
  wire [31:0] crc_in_a = crc_reg ^ {24'b0, fifo[rd_ptr]};
  // Wire para la entrada del segundo ciclo: crc_reg directo (sin XOR)
  wire [31:0] crc_in_b = crc_reg;

  // 4 pasos CRC desde crc_in_a (ciclo A)
  wire [31:0] a0 = crc_in_a[0] ? (crc_in_a >> 1) ^ 32'hEDB88320 : crc_in_a >> 1;
  wire [31:0] a1 = a0[0]       ? (a0       >> 1) ^ 32'hEDB88320 : a0       >> 1;
  wire [31:0] a2 = a1[0]       ? (a1       >> 1) ^ 32'hEDB88320 : a1       >> 1;
  wire [31:0] a3 = a2[0]       ? (a2       >> 1) ^ 32'hEDB88320 : a2       >> 1;

  // 4 pasos CRC desde crc_in_b (ciclo B)
  wire [31:0] b0 = crc_in_b[0] ? (crc_in_b >> 1) ^ 32'hEDB88320 : crc_in_b >> 1;
  wire [31:0] b1 = b0[0]       ? (b0       >> 1) ^ 32'hEDB88320 : b0       >> 1;
  wire [31:0] b2 = b1[0]       ? (b1       >> 1) ^ 32'hEDB88320 : b1       >> 1;
  wire [31:0] b3 = b2[0]       ? (b2       >> 1) ^ 32'hEDB88320 : b2       >> 1;

  // =========================================================
  // FSM — IDLE → PROCESS → FINALIZE → DONE
  // =========================================================
  localparam IDLE     = 2'b00;
  localparam PROCESS  = 2'b01;
  localparam FINALIZE = 2'b10;
  localparam DONE     = 2'b11;

  reg [1:0] fsm_state;

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      crc_reg    <= 32'hFFFFFFFF;
      crc_done   <= 1'b0;
      rd_ptr     <= 4'b0;
      fsm_state  <= IDLE;
      half       <= 1'b0;
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
              // Ciclo A: XOR + bits[3:0]
              crc_reg <= a3;
              half    <= 1'b1;
            end else begin
              // Ciclo B: bits[7:4] del mismo byte (sin XOR)
              crc_reg <= b3;
              half    <= 1'b0;
              rd_ptr  <= rd_ptr + 4'd1;  // avanza al siguiente byte
            end
          end else begin
            fsm_state <= FINALIZE;
          end
        end

        FINALIZE: begin
          crc_reg   <= ~crc_reg;   // inversión final IEEE 802.3
          crc_done  <= 1'b1;
          fsm_state <= DONE;
        end

        DONE: begin
          // Nuevo dato reinicia el motor
          if (wr && enable) begin
            crc_done  <= 1'b0;
            fsm_state <= IDLE;
          end
        end

      endcase
    end
  end

  wire irq = crc_done | fifo_full;

  // =========================================================
  // VGA — Visualización del estado CRC
  // =========================================================
  reg [7:0] frame_ctr;
  always @(posedge vsync or negedge rst_n) begin
    if (!rst_n) frame_ctr <= 8'b0;
    else        frame_ctr <= frame_ctr + 8'd1;
  end

  // --- Barra FIFO (fila 100–160) ---
  // fifo_count [3:0] max=15, escala x40 → max 600px (< 640) sin desbordamiento
  wire [9:0] bar_w    = {6'b0, fifo_count} * 10'd40;
  wire bar_on         = (pix_y >= 10'd100 && pix_y < 10'd160) &&
                        (pix_x < bar_w) && video_active;

  // --- Grid bits CRC (fila 260–460): 32 bloques de 20px ---
  // pix_x max 639, crc_bit_idx = pix_x[9:5] → max = 639>>5 = 19
  // solo mostramos bits 0-19 sin problema; bits 20-31 no aparecen en pantalla
  wire [4:0] bit_idx  = pix_x[9:5];           // 0-19 en zona visible
  wire [4:0] cell_x   = pix_x[4:0];           // 0-31 dentro de cada bloque
  wire grid_on        = (pix_y >= 10'd260 && pix_y < 10'd460) &&
                        (pix_x < 10'd640) &&
                        (cell_x >= 5'd2 && cell_x < 5'd18) &&
                        video_active;
  wire bit_on         = crc_reg[bit_idx];

  // --- Scanner horizontal animado ---
  // frame_ctr[5:0] * 8 → rango 0–504, nunca desborda 10 bits
  wire [9:0] scan_y   = {4'b0, frame_ctr[5:0]} << 3;
  wire scan_on        = (pix_y == scan_y) && video_active;

  // --- Indicadores FSM (fila 180–240): 4 bloques de 160px ---
  wire [1:0] fsm_blk  = pix_x[9:8];
  wire fsm_on         = (pix_y >= 10'd180 && pix_y < 10'd240) && video_active;

  // =========================================================
  // Lógica de color
  // =========================================================
  reg [1:0] pR, pG, pB;

  always @(*) begin
    pR = 2'b00; pG = 2'b00; pB = 2'b00;

    if (!video_active) begin
      pR = 2'b00; pG = 2'b00; pB = 2'b00;

    // Header azul degradado (fila 0–80)
    end else if (pix_y < 10'd80) begin
      pR = 2'b00;
      pG = pix_x[8:7];
      pB = 2'b11;

    // Barra FIFO — verde brillante
    end else if (bar_on) begin
      pR = 2'b00; pG = 2'b11; pB = 2'b01;

    // Fondo barra (zona vacía)
    end else if (pix_y >= 10'd100 && pix_y < 10'd160) begin
      pR = 2'b00; pG = 2'b01; pB = 2'b00;

    // Indicadores FSM
    end else if (fsm_on) begin
      case (fsm_blk)
        2'd0: begin   // Color según estado FSM
          case (fsm_state)
            IDLE:     begin pR=2'b01; pG=2'b01; pB=2'b01; end // gris
            PROCESS:  begin pR=2'b00; pG=2'b11; pB=2'b00; end // verde
            FINALIZE: begin pR=2'b11; pG=2'b11; pB=2'b00; end // amarillo
            default:  begin pR=2'b00; pG=2'b00; pB=2'b11; end // azul=DONE
          endcase
        end
        2'd1: begin   // IRQ — rojo si activo
          pR = irq ? 2'b11 : 2'b01;
          pG = 2'b00; pB = 2'b00;
        end
        2'd2: begin   // Enable — cyan si activo
          pR = 2'b00;
          pG = enable ? 2'b11 : 2'b00;
          pB = enable ? 2'b11 : 2'b00;
        end
        default: begin
          pR = 2'b00; pG = 2'b00; pB = 2'b00;
        end
      endcase

    // Grid bits CRC — ámbar=1, azul oscuro=0
    end else if (grid_on) begin
      if (bit_on) begin
        pR = 2'b11; pG = 2'b10; pB = 2'b00;
      end else begin
        pR = 2'b00; pG = 2'b00; pB = 2'b10;
      end

    // Scanner blanco animado
    end else if (scan_on) begin
      pR = 2'b11; pG = 2'b11; pB = 2'b11;

    // Fondo general
    end else begin
      pR = 2'b00;
      pG = (pix_y[7:6] == 2'b00) ? 2'b01 : 2'b00;
      pB = 2'b00;
    end
  end

  assign R = pR;
  assign G = pG;
  assign B = pB;

  // Suprimir unused: rd, addr, ch_sel, uio_in data bus
  wire _unused_ok = &{ena, uio_in, ui_in[7:1]};

endmodule