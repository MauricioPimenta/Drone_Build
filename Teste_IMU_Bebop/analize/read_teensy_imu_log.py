#!/usr/bin/env python3
import struct
import sys
from pathlib import Path

import numpy as np
import pandas as pd


HEADER_FMT = "<4sHH"   # magic[4], uint16 ver, uint16 struct_size
HEADER_SIZE = struct.calcsize(HEADER_FMT)

# Sample: uint32 t_us + 6 floats
SAMPLE_FMT = "<Iffffff"
SAMPLE_SIZE = struct.calcsize(SAMPLE_FMT)

COLUMNS = ["t_us", "ax", "ay", "az", "gx", "gy", "gz"]


def read_log(path: Path) -> pd.DataFrame:
    data = path.read_bytes()
    if len(data) < HEADER_SIZE:
        raise RuntimeError("Arquivo pequeno demais para conter header.")

    magic, ver, sz = struct.unpack_from(HEADER_FMT, data, 0)

    if magic != b"IMUF":
        raise RuntimeError(f"Magic inválido: {magic!r} (esperado b'IMUF')")

    if ver != 1:
        print(f"[WARN] Versão do log = {ver}, esperado 1. Tentando ler mesmo assim...")

    if sz != SAMPLE_SIZE:
        raise RuntimeError(
            f"Tamanho da struct no arquivo = {sz}, mas o script espera {SAMPLE_SIZE}.\n"
            f"Você mudou a struct no firmware?"
        )

    payload = data[HEADER_SIZE:]
    n = len(payload) // SAMPLE_SIZE
    if n == 0:
        return pd.DataFrame(columns=COLUMNS)

    payload = payload[: n * SAMPLE_SIZE]  # descarta resto incompleto

    # Lê tudo como numpy estruturado (rápido)
    dtype = np.dtype([
        ("t_us",  "<u4"),
        ("ax",    "<f4"),
        ("ay",    "<f4"),
        ("az",    "<f4"),
        ("gx",    "<f4"),
        ("gy",    "<f4"),
        ("gz",    "<f4"),
    ])

    arr = np.frombuffer(payload, dtype=dtype, count=n)

    df = pd.DataFrame({k: arr[k] for k in COLUMNS})

    # Tempo em segundos e dt (pra você checar frequência/jitter)
    df["t_s"] = df["t_us"] * 1e-6
    df["dt_s"] = df["t_s"].diff()
    df["fs_hz_inst"] = 1.0 / df["dt_s"]
    return df


def main():
    if len(sys.argv) < 2:
        print("Uso: read_teensy_imu_log.py LOG000.BIN [out.csv]")
        sys.exit(1)

    in_path = Path(sys.argv[1])
    df = read_log(in_path)

    print(df.head())
    print("\n--- Stats ---")
    if len(df) > 2:
        dt = df["dt_s"].dropna()
        print(f"Amostras: {len(df)}")
        print(f"dt médio: {dt.mean():.6f} s  => fs ~ {1.0/dt.mean():.1f} Hz")
        print(f"dt min/max: {dt.min():.6f} / {dt.max():.6f} s")
    else:
        print("Poucas amostras.")

    if len(sys.argv) >= 3:
        out_path = Path(sys.argv[2])
        df.to_csv(out_path, index=False)
        print(f"\nCSV salvo em: {out_path}")


if __name__ == "__main__":
    main()
