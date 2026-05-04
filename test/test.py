import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles

import os
import glob
import itertools
from PIL import Image, ImageChops


@cocotb.test()
async def test_project(dut):

    CLOCK_PERIOD = 40  # 25 MHz

    H_DISPLAY = 640
    H_FRONT   =  16
    H_SYNC    =  96
    H_BACK    =  48
    V_DISPLAY = 480
    V_FRONT   =  10
    V_SYNC    =   2
    V_BACK    =  33

    CAPTURE_FRAMES = 3

    H_SYNC_START = H_DISPLAY + H_FRONT
    H_SYNC_END   = H_SYNC_START + H_SYNC
    H_TOTAL      = H_SYNC_END + H_BACK
    V_SYNC_START = V_DISPLAY + V_FRONT
    V_SYNC_END   = V_SYNC_START + V_SYNC
    V_TOTAL      = V_SYNC_END + V_BACK

    # Paleta uo_out -> RGB
    # uo_out = {hsync, B[0], G[0], R[0], vsync, B[1], G[1], R[1]}
    palette = [bytes(3)] * 256
    for r1, r0, g1, g0, b1, b0 in itertools.product(range(2), repeat=6):
        red   = 170*r1 + 85*r0
        green = 170*g1 + 85*g0
        blue  = 170*b1 + 85*b0
        color_index = b0<<6 | g0<<5 | r0<<4 | b1<<2 | g1<<1 | r1<<0
        for sync_bits in (0x00, 0x08, 0x80, 0x88):
            palette[color_index | sync_bits] = bytes((red, green, blue))

    def safe_bit(signal, idx):
        """Lee un bit de uo_out. Devuelve None si es X (gate-level GL artifact)."""
        try:
            return int(signal.value[idx])
        except ValueError:
            return None

    def safe_uo(signal):
        """Lee uo_out completo. Devuelve None si tiene algun X."""
        try:
            return int(signal.value)
        except ValueError:
            return None

    # Clock y reset
    clock = Clock(dut.clk, CLOCK_PERIOD, unit="ns")
    cocotb.start_soon(clock.start())

    dut.ena.value    = 1
    dut.ui_in.value  = 0
    dut.uio_in.value = 0
    dut.rst_n.value  = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value  = 1
    await ClockCycles(dut.clk, 2)

    # ------------------------------------------------------------------
    # Sincronizacion robusta al inicio de frame
    #
    # En simulacion gate-level IHP sg13g2 con iverilog, los flip-flops
    # dfrbpq con reset sincrono arrancan en X y pueden tardar mas de un
    # frame en resolverse (limitacion conocida de iverilog con celdas IHP
    # que usan ifnone en specify blocks — ver warnings de compilacion).
    #
    # Estrategia:
    #   1. Esperar hasta que uo_out no tenga ningun bit X (maximo 3 frames)
    #   2. Sincronizar al borde de subida de vsync (inicio de frame limpio)
    # ------------------------------------------------------------------
    MAX_WAIT = H_TOTAL * V_TOTAL * 3  # 3 frames maximo de espera

    dut._log.info("GL-sync: esperando output estable (sin X)...")
    waited = 0
    while waited < MAX_WAIT:
        await ClockCycles(dut.clk, 1)
        waited += 1
        if safe_uo(dut.uo_out) is not None:
            break

    if waited >= MAX_WAIT:
        dut._log.warning("GL-sync: timeout esperando output sin X — continuando igual")
    else:
        dut._log.info(f"GL-sync: output estable despues de {waited} ciclos extra")

    # Sincronizar al flanco de subida de vsync (fin de pulso = inicio de frame)
    dut._log.info("GL-sync: buscando flanco de vsync...")
    for _ in range(H_TOTAL * V_TOTAL * 2):
        await ClockCycles(dut.clk, 1)
        v = safe_bit(dut.uo_out, 3)
        if v == 0:
            break   # entramos al pulso de vsync (activo bajo)

    for _ in range(H_TOTAL * V_TOTAL * 2):
        await ClockCycles(dut.clk, 1)
        v = safe_bit(dut.uo_out, 3)
        if v == 1:
            break   # fin del pulso, inicio del back porch

    # Avanzar por el back porch hasta el inicio del area de display
    await ClockCycles(dut.clk, H_TOTAL * V_BACK)
    dut._log.info("GL-sync: sincronizado al inicio de frame")

    # ------------------------------------------------------------------
    # Funciones de captura — tolerantes a X residuales
    # ------------------------------------------------------------------

    async def check_line(expected_vsync):
        for i in range(H_TOTAL):
            h = safe_bit(dut.uo_out, 7)
            v = safe_bit(dut.uo_out, 3)
            if h is not None:
                assert h == (0 if H_SYNC_START <= i < H_SYNC_END else 1), \
                    f"hsync incorrecto en pixel {i}"
            if v is not None:
                assert v == expected_vsync, f"vsync incorrecto en pixel {i}"
            await ClockCycles(dut.clk, 1)

    async def capture_line(framebuffer, offset):
        for i in range(H_TOTAL):
            h = safe_bit(dut.uo_out, 7)
            v = safe_bit(dut.uo_out, 3)
            if h is not None:
                assert h == (0 if H_SYNC_START <= i < H_SYNC_END else 1), \
                    f"hsync incorrecto en pixel {i}"
            if v is not None:
                assert v == 1, f"vsync bajo durante display en pixel {i}"
            if i < H_DISPLAY:
                val = safe_uo(dut.uo_out)
                if val is not None:
                    framebuffer[offset+3*i:offset+3*i+3] = palette[val]
                # X residual: dejar el pixel negro (bytearray ya viene a 0)
            await ClockCycles(dut.clk, 1)

    async def capture_frame(frame_num, check_sync=True):
        framebuffer = bytearray(V_DISPLAY * H_DISPLAY * 3)
        for j in range(V_DISPLAY):
            dut._log.info(f"Frame {frame_num}, linea {j} (display)")
            await capture_line(framebuffer, 3*j*H_DISPLAY)
        if check_sync:
            for j in range(V_FRONT):
                await check_line(1)
            for j in range(V_SYNC):
                await check_line(0)
            for j in range(V_BACK):
                await check_line(1)
        else:
            await ClockCycles(dut.clk, H_TOTAL * (V_TOTAL - V_DISPLAY))
        return Image.frombytes('RGB', (H_DISPLAY, V_DISPLAY), bytes(framebuffer))

    # Captura
    os.makedirs("output", exist_ok=True)
    for i in range(CAPTURE_FRAMES):
        frame = await capture_frame(i)
        frame.save(f"output/frame{i}.png")


@cocotb.test()
async def compare_reference(dut):
    for img in glob.glob("output/frame*.png"):
        basename = img.removeprefix("output/")
        dut._log.info(f"Comparando {basename} con imagen de referencia")
        frame = Image.open(img)
        ref   = Image.open(f"reference/{basename}")
        diff  = ImageChops.difference(frame, ref)
        if diff.getbbox() is not None:
            diff.save(f"output/diff_{basename}")
            assert False, f"{basename} difiere de la imagen de referencia"
