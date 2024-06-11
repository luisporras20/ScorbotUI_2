import tkinter as tk
from tkinter import Toplevel, font

# Variable global para contar cuántos botones se han presionado
buttons_pressed_count = 0

# Función para manejar los eventos de clic en cada botón en la interfaz mini
def button_pressed(button):
    global buttons_pressed_count
    buttons_pressed_count += 1
    button.config(state=tk.DISABLED)  # Deshabilita el botón después de ser presionado
    if buttons_pressed_count == 3:
        mini_root.destroy()  # Cierra la interfaz mini
        show_main_interface()  # Muestra la interfaz principal
    elif buttons_pressed_count == 1:
        button2.config(state=tk.NORMAL)  # Habilita el botón 2 cuando se presiona el botón 1
    elif buttons_pressed_count == 2:
        button3.config(state=tk.NORMAL)  # Habilita el botón 3 cuando se presiona el botón 2

# Función para mostrar la interfaz principal
def show_main_interface():
    # Crear la ventana principal de la interfaz principal
    root = tk.Tk()
    root.title("Interfaz Principal")
    bold_font = font.Font(weight="bold", size=10)

    # Crear y agregar los botones a la ventana principal
    button1_main = tk.Button(root, text="Button 1 Main", font=bold_font)
    button1_main.pack(pady=10)

    button2_main = tk.Button(root, text="Button 2 Main", font=bold_font)
    button2_main.pack(pady=10)

    button3_main = tk.Button(root, text="Button 3 Main", font=bold_font)
    button3_main.pack(pady=10)

    # Ejecutar el bucle principal de la aplicación
    root.mainloop()

# Crear y configurar la ventana principal de la interfaz mini
mini_root = tk.Tk()
mini_root.title("Interfaz Mini")
mini_root.geometry("300x200")

# Botones en la interfaz mini
button1 = tk.Button(mini_root, text="Button 1", command=lambda: button_pressed(button1))
button1.pack(pady=10)

button2 = tk.Button(mini_root, text="Button 2", command=lambda: button_pressed(button2), state=tk.DISABLED)
button2.pack(pady=10)

button3 = tk.Button(mini_root, text="Button 3", command=lambda: button_pressed(button3), state=tk.DISABLED)
button3.pack(pady=10)

# Ejecutar el bucle principal de la aplicación para la interfaz mini
mini_root.mainloop()
