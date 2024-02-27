#!/usr/bin/env python3

import tkinter as tk

def salir_del_bucle():
    global continuar_bucle
    continuar_bucle = False

def bucle_principal():
    global continuar_bucle
    while continuar_bucle:
        # Código del bucle principal
        pass

# Crear la ventana de Tkinter
ventana = tk.Tk()

# Crear un botón para salir
boton_salir = tk.Button(ventana, text="Salir", command=salir_del_bucle)
boton_salir.pack()

# Variable para controlar el bucle
continuar_bucle = True

# Iniciar el bucle principal
bucle_principal()

# Iniciar el bucle de eventos de Tkinter
ventana.mainloop()
