/*
 * Copyright (c) 2024 Jorge Luis Chuquimia Parra
 * SPDX-License-Identifier: Apache-2.0
 *
 * Proyecto  : CRC_FIFO — Motor CRC-32 con FIFO de 16 bytes
 * Autor     : Jorge Luis Chuquimia Parra
 * GitHub    : 27jorge05
 *
 * Descripcion:
 *   Motor de verificacion de integridad CRC-32 (polinomio IEEE 802.3,
 *   0xEDB88320 reflejado) con buffer FIFO de 16 bytes. El motor procesa
 *   4 bits por ciclo de reloj (2 ciclos por byte) usando logica
 *   combinacional desenrollada (sin funciones ni loops).
 *
 *   Salida VGA 640x480 @ 60Hz que muestra en tiempo real:
 *     - Barra de ocupacion del buffer FIFO
 *     - Indicadores de estado FSM (IDLE/PROCESS/FINALIZE/DONE)
 *     - Indicadores IRQ y Enable
 *     - Grid de 32 bits mostrando el valor actual del registro CRC
 *     - Linea scanner animada
 *
 * Interfaz de pines:
 *   ui_in[0]   = wr       — escribe uio_in en FIFO (pulso alto)
 *   ui_in[1]   = rd       — habilita salida de registro por uio_out
 *   ui_in[5:2] = addr     — seleccion de registro a leer
 *                           0 = status {irq, fifo_count}
 *                           1 = CRC byte 0 (LSB)
 *                           2 = CRC byte 1
 *                           3 = CRC byte 2
 *                           4 = CRC byte 3 (MSB)
 *   ui_in[6]   = enable   — habilita el motor CRC
 *   ui_in[7]   = rst_crc  — reset suave del motor (sin reset global)
 *   uio[7:0]   = data     — bus bidireccional: entrada al escribir,
 *                           salida al leer registros CRC
 *
 * Arquitectura interna:
 *   - FIFO circular de 16 bytes (wr_ptr / rd_ptr de 4 bits)
 *   - CRC-32 desenrollado: 4 etapas combinacionales por ciclo
 *     Ciclo A: XOR con byte + pasos 0-3
 *     Ciclo B: pasos 4-7 (sin nuevo XOR)
 *   - FSM de 4 estados: IDLE -> PROCESS -> FINALIZE -> DONE
 *   - Divisor de reloj para animacion VGA (sin segundo dominio)
 *   - Un unico dominio de reloj: clk 25MHz
 *
 * Restricciones Tiny Tapeout:
 *   - Tile 1x1 (~1400 celdas disponibles)
 *   - Sin Metal 5
 *   - Un solo dominio de reloj
 *   - 24 pines I/O (8 in + 8 out + 8 bidir)
 */

`default_nettype none

module tt_um_27jorge05_crc_fifo(
  input  wire [7:0] ui_in,    // Entradas dedicadas
  output wire [7:0] uo_out,   // Salidas dedicadas
  input  wire [7:0] uio_in,   // IOs: camino de entrada
  output wire [7:0] uio_out,  // IOs: camino de salida
  output wire [7:0] uio_oe,   // IOs: habilitacion (1=salida, 0=entrada)
  input  wire       ena,      // Siempre 1 cuando el diseno esta activo
  input  wire       clk,      // Reloj principal 25 MHz
  input  wire       rst_n     // Reset activo-bajo
);

  // ==========================================================
  // Senales VGA internas
  // ==========================================================
  wire hsync;
  wire vsync;
  wire video_active;
  wire [9:0] pix_x;
  wire [9:0] pix_y;
  wire [1:0] R;
  wire [1:0] G;
  wire [1:0] B;

  // TinyVGA PMOD — orden de bits fijo por el conector fisico
  assign uo_out = {hsync, B[0], G[0], R[0], vsync, B[1], G[1], R[1]};

  // ==========================================================
  // Decodificacion de pines de control
  // ==========================================================
  wire        wr      = ui_in[0]; // write strobe
  wire        rd      = ui_in[1]; // read strobe
  wire [3:0]  addr    = ui_in[5:2]; // direccion de registro
  wire        enable  = ui_in[6]; // habilita motor CRC
  wire        rst_crc = ui_in[7]; // reset suave del motor

  // Bus bidireccional uio
  reg [7:0] uio_out_reg;
  reg [7:0] uio_oe_reg;
  assign uio_out = uio_out_reg;
  assign uio_oe  = uio_oe_reg;

  // ==========================================================
  // Generador HSync/VSync
  // Modulo externo — no modificar
  // ==========================================================
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
  // FIFO circular de 16 bytes
  //
  // Estructura: array de 16 registros de 8 bits
  // Punteros  : wr_ptr (escritura) y rd_ptr (lectura), 4 bits c/u
  // Lleno     : wr_ptr + 1 == rd_ptr
  // Vacio     : wr_ptr == rd_ptr
  // Ocupacion : wr_ptr - rd_ptr (aritmetica modular 4 bits)
  // ==========================================================
  reg [7:0] fifo [0:15];
  reg [3:0] wr_ptr;
  reg [3:0] rd_ptr;

  wire       fifo_empty = (wr_ptr == rd_ptr);
  wire       fifo_full  = ((wr_ptr + 4'd1) == rd_ptr);
  wire [3:0] fifo_count = wr_ptr - rd_ptr;

  // Escritura en FIFO
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n || rst_crc) begin
      wr_ptr <= 4'b0;
    end else if (wr && enable && !fifo_full) begin
      fifo[wr_ptr] <= uio_in;
      wr_ptr        <= wr_ptr + 4'd1;
    end
  end

  // ==========================================================
  // Motor CRC-32 — logica combinacional desenrollada
  //
  // Polinomio reflejado IEEE 802.3: 0xEDB88320
  // Equivale al polinomio normal 0x04C11DB7 procesado LSB-first
  //
  // Cada "paso" CRC es:
  //   si bit[0] == 1: crc = (crc >> 1) XOR 0xEDB88320
  //   si bit[0] == 0: crc = (crc >> 1)
  //
  // Se aplican 4 pasos por ciclo de reloj:
  //   Ciclo A (half=0): entrada = crc_reg XOR byte_actual
  //                     aplica pasos 0,1,2,3 -> resultado a3
  //   Ciclo B (half=1): entrada = crc_reg (ya tiene XOR)
  //                     aplica pasos 4,5,6,7 -> resultado b3
  //   Total: 8 pasos = 1 byte completo en 2 ciclos
  // ==========================================================

  // Entradas de cada cadena combinacional
  wire [31:0] crc_in_a = crc_reg ^ {24'b0, fifo[rd_ptr]}; // ciclo A
  wire [31:0] crc_in_b = crc_reg;                          // ciclo B

  // Cadena A — pasos 0 al 3
  wire [31:0] a0 = crc_in_a[0] ? (crc_in_a >> 1) ^ 32'hEDB88320
                                : (crc_in_a >> 1);
  wire [31:0] a1 = a0[0]       ? (a0       >> 1) ^ 32'hEDB88320
                                : (a0       >> 1);
  wire [31:0] a2 = a1[0]       ? (a1       >> 1) ^ 32'hEDB88320
                                : (a1       >> 1);
  wire [31:0] a3 = a2[0]       ? (a2       >> 1) ^ 32'hEDB88320
                                : (a2       >> 1);

  // Cadena B — pasos 4 al 7
  wire [31:0] b0 = crc_in_b[0] ? (crc_in_b >> 1) ^ 32'hEDB88320
                                : (crc_in_b >> 1);
  wire [31:0] b1 = b0[0]       ? (b0       >> 1) ^ 32'hEDB88320
                                : (b0       >> 1);
  wire [31:0] b2 = b1[0]       ? (b1       >> 1) ^ 32'hEDB88320
                                : (b1       >> 1);
  wire [31:0] b3 = b2[0]       ? (b2       >> 1) ^ 32'hEDB88320
                                : (b2       >> 1);

  // ==========================================================
  // FSM de control — 4 estados
  //
  // IDLE     : espera datos en FIFO y enable=1
  // PROCESS  : procesa bytes (2 ciclos por byte)
  // FINALIZE : invierte el resultado (estandar IEEE 802.3)
  // DONE     : resultado listo, espera nueva escritura
  // ==========================================================
  localparam IDLE     = 2'b00;
  localparam PROCESS  = 2'b01;
  localparam FINALIZE = 2'b10;
  localparam DONE     = 2'b11;

  reg [31:0] crc_reg;    // registro CRC actual
  reg        crc_done;   // flag: resultado listo
  reg        half;       // flag: primera/segunda mitad del byte
  reg [1:0]  fsm_state;  // estado actual de la FSM

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n || rst_crc) begin
      crc_reg   <= 32'hFFFFFFFF; // valor inicial CRC estandar
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
              // Ciclo A: incorpora byte, procesa bits 0-3
              crc_reg <= a3;
              half    <= 1'b1;
            end else begin
              // Ciclo B: procesa bits 4-7, avanza puntero
              crc_reg <= b3;
              half    <= 1'b0;
              rd_ptr  <= rd_ptr + 4'd1;
            end
          end else begin
            // FIFO vaciada — finalizar
            fsm_state <= FINALIZE;
          end
        end

        FINALIZE: begin
          // Inversion final obligatoria segun IEEE 802.3
          crc_reg   <= ~crc_reg;
          crc_done  <= 1'b1;
          fsm_state <= DONE;
        end

        DONE: begin
          // Nueva escritura reinicia el motor
          if (wr && enable) begin
            crc_done  <= 1'b0;
            fsm_state <= IDLE;
          end
        end

      endcase
    end
  end

  // ==========================================================
  // Interfaz de lectura de registros
  // Cuando rd=1 y enable=1, uio_out expone el registro en addr
  // uio_oe = 0xFF activa los pines como salida
  // ==========================================================
  wire irq = crc_done | fifo_full; // interrupcion: resultado listo o FIFO llena

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      uio_out_reg <= 8'b0;
      uio_oe_reg  <= 8'b0;
    end else if (rd && enable) begin
      uio_oe_reg <= 8'hFF; // pines como salida
      case (addr)
        4'd0:    uio_out_reg <= {3'b0, irq, fifo_count}; // status
        4'd1:    uio_out_reg <= crc_reg[7:0];             // CRC LSB
        4'd2:    uio_out_reg <= crc_reg[15:8];
        4'd3:    uio_out_reg <= crc_reg[23:16];
        4'd4:    uio_out_reg <= crc_reg[31:24];           // CRC MSB
        default: uio_out_reg <= 8'b0;
      endcase
    end else begin
      uio_oe_reg  <= 8'b0; // pines como entrada (reposo)
      uio_out_reg <= 8'b0;
    end
  end

  // ==========================================================
  // Divisor de reloj para animacion VGA
  //
  // Un UNICO dominio de reloj — sin posedge vsync
  // 25MHz / 2^18 = ~95 Hz -> frame_ctr cambia ~95 veces/seg
  // frame_ctr[5:0] * 8 = scanner se mueve 8px por frame
  // Rango: 0 a 504 — nunca desborda 10 bits (max 1023)
  // ==========================================================
  reg [17:0] clk_div;
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) clk_div <= 18'b0;
    else        clk_div <= clk_div + 18'd1;
  end

  wire [5:0] frame_ctr = clk_div[17:12]; // 6 bits de animacion

  // ==========================================================
  // Zonas de visualizacion VGA (640x480 @ 60Hz)
  //
  // fila   0- 79 : header azul con degradado verde
  // fila 100-159 : barra de ocupacion FIFO (verde)
  // fila 180-239 : indicadores FSM en 4 bloques de 160px
  //                bloque 0: estado FSM (gris/verde/amarillo/azul)
  //                bloque 1: IRQ (rojo si activo)
  //                bloque 2: Enable (cyan si activo)
  //                bloque 3: rst_crc (magenta si activo)
  // fila 260-459 : grid de 32 bits del registro CRC
  //                cada bit ocupa 20px de ancho
  //                ambar = bit 1, azul oscuro = bit 0
  // scanner      : linea blanca que baja ~8px por frame
  // resto        : fondo negro con tono verde tenue (arriba)
  // ==========================================================

  // Barra FIFO: ancho = fifo_count * 40px, max = 15*40 = 600 < 640
  wire [9:0] bar_w = {6'b0, fifo_count} * 10'd40;
  wire bar_on = (pix_y >= 10'd100) && (pix_y < 10'd160) &&
                (pix_x < bar_w) && video_active;

  // Grid CRC: bloques de 20px, margen interior 2px a cada lado
  wire [4:0] bit_idx = pix_x[9:5];       // que bit del CRC (0-19 visibles)
  wire [4:0] cell_x  = pix_x[4:0];       // posicion dentro del bloque
  wire grid_on = (pix_y >= 10'd260) && (pix_y < 10'd460) &&
                 (pix_x < 10'd640) &&
                 (cell_x >= 5'd2) && (cell_x < 5'd18) &&
                 video_active;
  wire bit_on = crc_reg[bit_idx];

  // Scanner: posicion Y = frame_ctr * 8, max = 63*8 = 504 < 640
  wire [9:0] scan_y = {4'b0, frame_ctr} << 3;
  wire scan_on = (pix_y == scan_y) && video_active;

  // Indicadores FSM: 4 bloques horizontales de 160px
  wire [1:0] fsm_blk = pix_x[9:8];
  wire fsm_on = (pix_y >= 10'd180) && (pix_y < 10'd240) && video_active;

  // ==========================================================
  // Logica de color — combinacional pura (always @(*))
  // Prioridad de zonas (if-else if en orden):
  //   1. fuera de zona visible → negro
  //   2. header
  //   3. barra FIFO activa
  //   4. fondo barra FIFO
  //   5. indicadores FSM
  //   6. grid CRC
  //   7. scanner
  //   8. fondo general
  // ==========================================================
  reg [1:0] pR, pG, pB;

  always @(*) begin
    pR = 2'b00; pG = 2'b00; pB = 2'b00;

    if (!video_active) begin
      pR = 2'b00; pG = 2'b00; pB = 2'b00;

    end else if (pix_y < 10'd80) begin
      // Header: azul fijo con degradado verde segun pix_x
      pR = 2'b00;
      pG = pix_x[8:7];
      pB = 2'b11;

    end else if (bar_on) begin
      // Barra FIFO ocupada — verde brillante
      pR = 2'b00; pG = 2'b11; pB = 2'b01;

    end else if ((pix_y >= 10'd100) && (pix_y < 10'd160)) begin
      // Fondo barra FIFO vacia — verde oscuro
      pR = 2'b00; pG = 2'b01; pB = 2'b00;

    end else if (fsm_on) begin
      // Indicadores FSM — 4 bloques
      case (fsm_blk)
        2'd0: begin
          case (fsm_state)
            IDLE:     begin pR=2'b01; pG=2'b01; pB=2'b01; end // gris
            PROCESS:  begin pR=2'b00; pG=2'b11; pB=2'b00; end // verde
            FINALIZE: begin pR=2'b11; pG=2'b11; pB=2'b00; end // amarillo
            default:  begin pR=2'b00; pG=2'b00; pB=2'b11; end // azul=DONE
          endcase
        end
        2'd1: begin // IRQ
          pR = irq ? 2'b11 : 2'b01;
          pG = 2'b00; pB = 2'b00;
        end
        2'd2: begin // Enable
          pR = 2'b00;
          pG = enable ? 2'b11 : 2'b00;
          pB = enable ? 2'b11 : 2'b00;
        end
        default: begin // rst_crc
          pR = rst_crc ? 2'b11 : 2'b00;
          pG = 2'b00;
          pB = rst_crc ? 2'b11 : 2'b00;
        end
      endcase

    end else if (grid_on) begin
      // Grid CRC: ambar si bit=1, azul oscuro si bit=0
      if (bit_on) begin
        pR = 2'b11; pG = 2'b10; pB = 2'b00;
      end else begin
        pR = 2'b00; pG = 2'b00; pB = 2'b10;
      end

    end else if (scan_on) begin
      // Scanner animado — linea blanca
      pR = 2'b11; pG = 2'b11; pB = 2'b11;

    end else begin
      // Fondo general: negro con leve tono verde en zona alta
      pR = 2'b00;
      pG = (pix_y[7:6] == 2'b00) ? 2'b01 : 2'b00;
      pB = 2'b00;
    end
  end

  assign R = pR;
  assign G = pG;
  assign B = pB;

  // Suprimir warnings de senales no usadas
  wire _unused_ok = &{ena, uio_in};

endmodule