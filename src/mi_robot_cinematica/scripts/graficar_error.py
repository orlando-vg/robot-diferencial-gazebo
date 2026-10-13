#!/usr/bin/env python3
"""
graficar_error.py — Gráficas de la comparación entre odometrías.

    python3 graficar_error.py            (lee ~/error_odom.csv)

Genera ~/error_odom.png con:
  1) Trayectoria (x, y) de las dos odometrías
  2) Error de posición en el tiempo
  3) Error de orientación en el tiempo
"""
import csv
import math
import os

import matplotlib.pyplot as plt

ruta_csv = os.path.expanduser('~/error_odom.csv')
ruta_png = os.path.expanduser('~/error_odom.png')

# ===== Leer el CSV =====
t, xp, yp, xr, yr, epos, eth = [], [], [], [], [], [], []
with open(ruta_csv) as f:
    for fila in csv.DictReader(f):
        t.append(float(fila['t']))
        xp.append(float(fila['x_plugin']))
        yp.append(float(fila['y_plugin']))
        xr.append(float(fila['x_ruedas']))
        yr.append(float(fila['y_ruedas']))
        epos.append(float(fila['e_pos']) * 1000.0)            # m -> mm
        eth.append(math.degrees(float(fila['eth'])))          # rad -> grados

t0 = t[0]
t = [ti - t0 for ti in t]                                     # tiempo desde el inicio [s]

# ===== Resumen numérico =====
rms = math.sqrt(sum(e * e for e in epos) / len(epos))
print(f'Muestras: {len(t)}  |  duración: {t[-1]:.1f} s')
print(f'Error de posición  -> RMS: {rms:.3f} mm  |  máximo: {max(epos):.3f} mm')
print(f'Error de orientación -> máximo: {max(abs(e) for e in eth):.4f} °')

# ===== Gráficas =====
fig, ax = plt.subplots(1, 3, figsize=(16, 5))

# 1) Trayectorias
ax[0].plot(xp, yp, label='/odom (plugin DiffDrive)', linewidth=3)
ax[0].plot(xr, yr, '--', label='/odom_ruedas (nodo propio)', linewidth=1.5)
ax[0].set_xlabel('x [m]')
ax[0].set_ylabel('y [m]')
ax[0].set_title('Trayectoria en el frame odom')
ax[0].axis('equal')
ax[0].grid(True)
ax[0].legend()

# 2) Error de posición
ax[1].plot(t, epos)
ax[1].set_xlabel('tiempo [s]')
ax[1].set_ylabel('error de posición [mm]')
ax[1].set_title(f'Error de posición (RMS = {rms:.2f} mm)')
ax[1].grid(True)

# 3) Error de orientación
ax[2].plot(t, eth)
ax[2].set_xlabel('tiempo [s]')
ax[2].set_ylabel('error de orientación [°]')
ax[2].set_title('Error de orientación')
ax[2].grid(True)

plt.tight_layout()
plt.savefig(ruta_png, dpi=150)
print(f'Gráfica guardada en {ruta_png}')
plt.show()