#!/usr/bin/env python3

from tkinter import *
import customtkinter

customtkinter.set_appearance_mode("dark")

customtkinter.set_default_color_theme("green")


root = customtkinter.CTk()

root.geometry('500x500')

def button_event():
    print("button pressed")

button = customtkinter.CTkButton(master=root, width=120, height=32, border_width=0, corner_radius=8, text="Pause", command=button_event)
button.place(relx=0.5, rely=0.5, anchor=CENTER)

def combobox_callback(choice):
    print("combobox dropdown clicked:", choice)

combobox = customtkinter.CTkComboBox(master=root,
                                     values=["StandStill","Maintenance mode", "Manual mode", "Automatic mode"], width=150,
                                     command=combobox_callback)
combobox.pack(padx=20, pady=10)
combobox.set("StandStill")  # set initial value

root.mainloop()
