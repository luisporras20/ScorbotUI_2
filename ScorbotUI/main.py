import tkinter as tk
from tkinter import Toplevel, font, Listbox, Scrollbar, Frame, Label, PhotoImage, Button, Entry
import os
from os import listdir
from os.path import isfile, join
from PIL import Image, ImageTk
import tempfile
import tkinter as tk
import subprocess
from tkinter import filedialog
from tkinter import messagebox
import threading
import serial
import time
import rclpy
from builtin_interfaces.msg import Duration
import shutil
import numpy as np
from math import radians, cos, sin, atan2, sqrt, degrees

from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
global ser2
ser2 = serial.Serial('/dev/ttyACM2', 1000000)

wrist_roll2hotend = np.array([[cos(radians(-90)),  0, sin(radians(-90)), -0.049],
                              [0,  1,          0, -0.0137],
                              [-sin(radians(-90)), 0, cos(radians(-90)), 0.132],
                              [0, 0,       0,           1]])

hotend2wrist_roll = np.linalg.inv(wrist_roll2hotend)

base2station = np.array([[1,  0, 0, 0.315],
                         [0,  1, 0, 0.52],
                         [0,  0, 1, 0.245],
                         [0,  0, 0, 1]])
global gcode_filename
gcode_filename = '/home/ras-rpi/CE3PRO_Pieza1.gcode'
print(gcode_filename)
global xglobal
xglobal = 0.139
global yglobal
yglobal = 0.176
global zglobal
zglobal = 0.65
global theta1global
theta1global = -1.527657638698873
global theta2global
theta2global = 2.072261577664386
global theta3global
theta3global = -2.040044236880632
global theta4global
theta4global = 1.67
global theta5global
temperatura_cama = 0
temperatura_extrusor = 0
temperatura_seteada_cama = 0
temperatura_seteada_extrusor = 0

temperaturas_seteadas = {
    "cama": 0,
    "extrusor": 0
}

theta5global = 0.0
offset_slide_base = 0.17
link_1 = 0.450
link_2 = 0.220
link_3 = 0.220
a_2 = 0.025
slide_base = 0.0
d_1 = offset_slide_base + slide_base
offset_slide_base = 0.17 #Cuidado que aquí depende de qué posición puede o no alcanzar el robot, medirlo al revés como dijo él jajaja
# Variable global para contar cuántos botones se han presionado
buttons_pressed_count = 0
# Funciones para manejar los eventos de clic en cada botón
def lower_control():
    port = '/dev/ttyACM0'  # Puerto serial en Linux
    baudrate = 115200  # Velocidad de transmisión

    # Crear objeto de puerto serial
    ser = serial.Serial(port, baudrate)

    # data = "c\n"
    # ser.write(data.encode())

    # time.sleep(10)

    data = "h\n"
    ser.write(data.encode())

    # Cerrar el puerto serial
    ser.close()
def controladores():
        comandos = [
        "cd /home/ras-rpi/robot_ws/",
        ". install/setup.bash",  
        "ros2 launch ros2_control_demo_example_1 rrbot.launch.py"
]

        comando_terminal = f"x-terminal-emulator -e 'bash -c \""
        for comando in comandos:
            comando_terminal += f"{comando}; "
        comando_terminal += "bash\"'"
        subprocess.run(comando_terminal, shell=True)
def upper_control():
    # Configurar el puerto serial
    port = '/dev/ttyACM1'  # Puerto serial en Linux
    baudrate = 115200  # Velocidad de transmisión

    # Crear objeto de puerto serial
    ser = serial.Serial(port, baudrate)

    # data = "c\n"
    # ser.write(data.encode())

    # time.sleep(10)

    data = "h\n"
    ser.write(data.encode())

    # Cerrar el puerto serial
    ser.close()
def ejecutarambosLower():
    lower_control()
    button_pressed(button1)
def ejecutarambosUpper():
    upper_control()
    button_pressed(button2)
def insertar_archivo():
    archivo_origen = filedialog.askopenfilename()
    if archivo_origen:
        print("Archivo seleccionado:", archivo_origen)
        ruta_destino = "/home/ras-rpi/Codigosg"
        nombre_archivo = os.path.basename(archivo_origen)
        archivo_destino = os.path.join(ruta_destino, nombre_archivo)
        try:
            shutil.copy(archivo_origen, archivo_destino)
            print("Archivo guardado en:", archivo_destino)
            create_or_destroy_window("print_window")
        except Exception as e:
            print("Error al guardar el archivo:", e)
def open_print_window():
    # Cerrar todas las subventanas abiertas
    for widget in root.winfo_children():
        if isinstance(widget, Toplevel):
            widget.destroy()
    window = Toplevel(root)
    window.title("Print Window")
    window.geometry("800x450")
    window.resizable(False, False)

    # Ruta de archivos
    path = r'/home/ras-rpi/Codigosg'
    
    # Frame para la Listbox y los botones
    frame = tk.Frame(window)
    frame.pack(fill='both', expand=True)

    # Lista para almacenar nombres de archivos
    listbox = Listbox(frame)  # Texto en negrita
    listbox.pack(side='left', fill='both', expand=True)
    
    # Frame para los botones
    button_frame = tk.Frame(frame)
    button_frame.pack(side='right', fill='y', padx=10, pady=10)

    # Botones
    controladores_button = tk.Button(button_frame, text="Controladores", font=bold_font, command=lambda: controladores())
    controladores_button.pack(fill='x', pady=20)

    select_button = tk.Button(button_frame, text="Seleccionar", font=bold_font, command=lambda: select_file(listbox))
    select_button.pack(fill='x', pady=20)

    temperatura_button = tk.Button(button_frame, text="Temperatura Extrusor", font=bold_font, command=calentar_extrusor)
    temperatura_button.pack(fill='x', pady=20)

    temperaturaCama_button = tk.Button(button_frame, text="Temperatura Cama", font=bold_font, command=calentar_superficie)
    temperaturaCama_button.pack(fill='x', pady=20)

    print_button = tk.Button(button_frame, text="Imprimir", font=bold_font, command=start_trajectory)
    print_button.pack(fill='x', pady=20)
    
    nuevo_button = tk.Button(button_frame, text="Insertar Archivo", font=bold_font, command=insertar_archivo)
    nuevo_button.pack(fill='x', pady=20)
    
    # Leer los archivos de una carpeta específica y añadirlos a la Listbox
    files = os.listdir(path)
    for file in files:
        if os.path.isfile(os.path.join(path, file)):
            listbox.insert(tk.END, file)

def calentar_superficie():
                            

                            
                        global temperatura_cama

                        try:
                                data2 = "c\n"
                                ser2.write(data2.encode())
                                print("Letra 'c' enviada correctamente.")
                        except serial.SerialException as e:
                                print(f"Error al abrir puerto serial: {e}")
                        except Exception as e:
                                print(f"Error general: {e}")

                        controlTemp2 = tk.Toplevel()
                        controlTemp2.title("Cama de impresion")
                        controlTemp2.geometry("400x300")

                        temperatura_label2 = tk.Label(controlTemp2, text="Temperatura:")
                        temperatura_label2.pack(pady=10)

                        barra_temperatura2 = tk.Scale(controlTemp2, from_=0, to=70, orient="horizontal", length=300)
                        barra_temperatura2.set(temperatura_cama)  # Establecer la posición del slider al valor guardado
                        barra_temperatura2.pack(pady=10)

                        dato_seteado_label2 = tk.Label(controlTemp2, text="", font=("Helvetica", 12))
                        dato_seteado_label2.pack(pady=10)
                        
                        dato_seteado_label4 = tk.Label(controlTemp2, text="", font=("Helvetica", 12))
                        dato_seteado_label4.pack(pady=10)
                        
                        dato_seteado_label4.config(text=f"Temperatura seteada: {temperatura_cama} grados")

                        def enviar_temperatura2():
                                global temperatura_cama
                                temperatura_cama = int(barra_temperatura2.get())
                                dato_seteado_label4.config(text=f"Temperatura seteada: {temperatura_cama} grados")
                                mensaje2 = str(temperatura_cama).encode('latin-1')
                                ser2.write(mensaje2)

                        #def actualizar_entry2(event):
                        #    temperatura2 = str(int(barra_temperatura2.get()))
                        #    dato_seteado_label2.config(text=f"Temperatura actual: {temperatura2} grados")

                       # barra_temperatura2.bind("<Motion>", actualizar_entry2)

                        enviar_button2 = tk.Button(controlTemp2, text="Enviar a Arduino", command=enviar_temperatura2)
                        enviar_button2.pack(pady=10)

                        def monitor_temperatura2():
                                 while True:
                                         if not controlTemp2.winfo_exists():
                                                    break
                                         try:
                                            lineBytes2 = ser2.readline()
                                            line2 = lineBytes2.decode('latin-1').strip()
                                            if line2.startswith("C"):
                                                        temp2 = line2[1:]
                                                        if dato_seteado_label2.winfo_exists():
                                                            dato_seteado_label2.config(text=f"Temperatura actual: {temp2} grados")
                                         except serial.SerialException as e:
                                                print(f"Error de lectura del puerto serial: {e}")
                                         controlTemp2.update()

                        threading.Thread(target=monitor_temperatura2, daemon=True).start()


def calentar_extrusor():


                           
                            global temperatura_extrusor
                            try:
                                #ser = serial.Serial('/dev/ttyACM2', 1000000)
                                data = "e\n"
                               
                                ser2.write(data.encode())
                                print("Letra 'e' enviada correctamente.")
                            except serial.SerialException as e:
                                        print(f"Error al abrir puerto serial: {e}")
                            except Exception as e:
                                print(f"Error general: {e}")
                            

                            #ser = serial.Serial('/dev/ttyACM0', 1000000)
                            #data = "e\n"
                            #ser.write(data.encode())
                           
                            controlTemp = tk.Toplevel()
                            
                            controlTemp.title("Extrusor")
                            controlTemp.geometry("400x300")

                            temperatura_label = tk.Label(controlTemp, text="Temperatura:")
                            temperatura_label.pack(pady=10)
                            
                            

                            barra_temperatura = tk.Scale(controlTemp, from_=0, to=220, orient="horizontal", length=300)
                            barra_temperatura.set(temperatura_extrusor) 
                            barra_temperatura.pack(pady=10)
                            
                            
                            dato_seteado_label = tk.Label(controlTemp, text="", font=("Helvetica", 12))
                            dato_seteado_label.pack(pady=10)
                            
                            dato_seteado_label3 = tk.Label(controlTemp, text="", font=("Helvetica", 12))
                            dato_seteado_label3.pack(pady=10)
                            
                            dato_seteado_label3.config(text=f"Temperatura seteada: {temperatura_extrusor} grados")
                            # Configurar la conexión serial con Arduino (ajusta el puerto según tu configuración)
                           

                            # Iniciar el monitoreo constante de la temperatura recibida
                           
                            
                            def enviar_temperatura():
                                global temperatura_extrusor
                                temperatura_extrusor = str(int(barra_temperatura.get()))  # Convertir el valor a entero
                                dato_seteado_label3.config(text=f"Temperatura seteada: {temperatura_extrusor} grados")
                                #print(f"Temperatura enviada: {temperatura} grados")

                                # Enviar la temperatura al Arduino
                                mensaje = temperatura_extrusor.encode('latin-1')
                                ser2.write(mensaje)
                                
                                #print(f"Enviando temperatura al Arduino: {temperatura} grados")
                                
                                
                                
                            #def actualizar_entry(event):
                                # Actualizar la entrada de temperatura al mover la barra
                            #    temperatura = str(int(barra_temperatura.get()))
                             #   dato_seteado_label.config(text=f"Temperatura actual: {temperatura} grados")
                                
                            #barra_temperatura.bind("<Motion>", actualizar_entry)
                            
                            enviar_button = tk.Button(controlTemp, text="Enviar a Arduino", command=enviar_temperatura)
                            enviar_button.pack(pady=10)
                            

                            def monitor_temperatura():
                                # Esta función puede ejecutarse como un hilo aparte para no bloquear la interfaz gráfica
                                while True:
                                         if not controlTemp.winfo_exists():
                                                    break
                                         try:
                                            lineBytes = ser2.readline()
                                            line = lineBytes.decode('latin-1').strip()
                                            if line.startswith("E"):
                                                        temp = line[1:]
                                                        if dato_seteado_label.winfo_exists():
                                                            dato_seteado_label.config(text=f"Temperatura actual: {temp} grados")
                                         except serial.SerialException as e:
                                                print(f"Error de lectura del puerto serial: {e}")
                                         controlTemp.update()

                            threading.Thread(target=monitor_temperatura, daemon=True).start()






def start_trajectory():
    if not gcode_filename:
        messagebox.showerror("Error", "Por favor selecciona un archivo GCode primero.")
        return
    
    # Crear un hilo para ejecutar la tarea en segundo plano sin bloquear la interfaz gráfica
    thread = threading.Thread(target=execute_trajectory)
    thread.start()
def start_trajectory2():
    if not gcode_filename:
        messagebox.showerror("Error", "Por favor selecciona un archivo GCode primero.")
        return
    
    # Crear un hilo para ejecutar la tarea en segundo plano sin bloquear la interfaz gráfica
    thread = threading.Thread(target=execute_trajectory2)
    thread.start()
def start_trajectory3():
    if not gcode_filename:
        messagebox.showerror("Error", "Por favor selecciona un archivo GCode primero.")
        return
    
    # Crear un hilo para ejecutar la tarea en segundo plano sin bloquear la interfaz gráfica
    thread = threading.Thread(target=execute_trajectory3)
    thread.start()
def start_trajectory4():
    if not gcode_filename:
        messagebox.showerror("Error", "Por favor selecciona un archivo GCode primero.")
        return
    
    # Crear un hilo para ejecutar la tarea en segundo plano sin bloquear la interfaz gráfica
    thread = threading.Thread(target=execute_trajectory4)
    thread.start()
def start_trajectory5():
    if not gcode_filename:
        messagebox.showerror("Error", "Por favor selecciona un archivo GCode primero.")
        return
    
    # Crear un hilo para ejecutar la tarea en segundo plano sin bloquear la interfaz gráfica
    thread = threading.Thread(target=execute_trajectory5)
    thread.start()
def start_trajectory6():
    if not gcode_filename:
        messagebox.showerror("Error", "Por favor selecciona un archivo GCode primero.")
        return
    
    # Crear un hilo para ejecutar la tarea en segundo plano sin bloquear la interfaz gráfica
    thread = threading.Thread(target=execute_trajectory6)
    thread.start()
def start_trajectory7():
    if not gcode_filename:
        messagebox.showerror("Error", "Por favor selecciona un archivo GCode primero.")
        return
    
    # Crear un hilo para ejecutar la tarea en segundo plano sin bloquear la interfaz gráfica
    thread = threading.Thread(target=execute_trajectory7)
    thread.start()
def start_trajectory8():
    if not gcode_filename:
        messagebox.showerror("Error", "Por favor selecciona un archivo GCode primero.")
        return
    
    # Crear un hilo para ejecutar la tarea en segundo plano sin bloquear la interfaz gráfica
    thread = threading.Thread(target=execute_trajectory8)
    thread.start()
def start_trajectory9():
    if not gcode_filename:
        messagebox.showerror("Error", "Por favor selecciona un archivo GCode primero.")
        return
    
    # Crear un hilo para ejecutar la tarea en segundo plano sin bloquear la interfaz gráfica
    thread = threading.Thread(target=execute_trajectory9)
    thread.start()
def start_trajectory10():
    if not gcode_filename:
        messagebox.showerror("Error", "Por favor selecciona un archivo GCode primero.")
        return
    
    # Crear un hilo para ejecutar la tarea en segundo plano sin bloquear la interfaz gráfica
    thread = threading.Thread(target=execute_trajectory10)
    thread.start()
def start_trajectory11():
    if not gcode_filename:
        messagebox.showerror("Error", "Por favor selecciona un archivo GCode primero.")
        return
    
    # Crear un hilo para ejecutar la tarea en segundo plano sin bloquear la interfaz gráfica
    thread = threading.Thread(target=execute_trajectory11)
    thread.start()
def start_trajectory12():
    if not gcode_filename:
        messagebox.showerror("Error", "Por favor selecciona un archivo GCode primero.")
        return
    
    # Crear un hilo para ejecutar la tarea en segundo plano sin bloquear la interfaz gráfica
    thread = threading.Thread(target=execute_trajectory12)
    thread.start()
def start_trajectory13():
    if not gcode_filename:
        messagebox.showerror("Error", "Por favor selecciona un archivo GCode primero.")
        return
    
    # Crear un hilo para ejecutar la tarea en segundo plano sin bloquear la interfaz gráfica
    thread = threading.Thread(target=execute_trajectory13)
    thread.start()
def start_trajectory14():
    if not gcode_filename:
        messagebox.showerror("Error", "Por favor selecciona un archivo GCode primero.")
        return
    
    # Crear un hilo para ejecutar la tarea en segundo plano sin bloquear la interfaz gráfica
    thread = threading.Thread(target=execute_trajectory14)
    thread.start()
def start_trajectory15():
    if not gcode_filename:
        messagebox.showerror("Error", "Por favor selecciona un archivo GCode primero.")
        return
    
    # Crear un hilo para ejecutar la tarea en segundo plano sin bloquear la interfaz gráfica
    thread = threading.Thread(target=execute_trajectory15)
    thread.start()
def start_trajectory16():
    if not gcode_filename:
        messagebox.showerror("Error", "Por favor selecciona un archivo GCode primero.")
        return
    
    # Crear un hilo para ejecutar la tarea en segundo plano sin bloquear la interfaz gráfica
    thread = threading.Thread(target=execute_trajectory16)
    thread.start()
def start_trajectory17():
    if not gcode_filename:
        messagebox.showerror("Error", "Por favor selecciona un archivo GCode primero.")
        return
    
    # Crear un hilo para ejecutar la tarea en segundo plano sin bloquear la interfaz gráfica
    thread = threading.Thread(target=execute_trajectory17)
    thread.start()
def start_trajectory18():
    if not gcode_filename:
        messagebox.showerror("Error", "Por favor selecciona un archivo GCode primero.")
        return
    
    # Crear un hilo para ejecutar la tarea en segundo plano sin bloquear la interfaz gráfica
    thread = threading.Thread(target=execute_trajectory18)
    thread.start()
def start_trajectory19():
    if not gcode_filename:
        messagebox.showerror("Error", "Por favor selecciona un archivo GCode primero.")
        return
    
    # Crear un hilo para ejecutar la tarea en segundo plano sin bloquear la interfaz gráfica
    thread = threading.Thread(target=execute_trajectory19)
    thread.start()
def execute_trajectory():
    try:
        main()
    except Exception as e:
        messagebox.showerror("Error", f"Ocurrió un error al ejecutar la trayectoria: {str(e)}")
def execute_trajectory2():
    try:
        main2()
    except Exception as e:
        messagebox.showerror("Error", f"Ocurrió un error al ejecutar la trayectoria: {str(e)}")
def execute_trajectory3():
    try:
        main3()
    except Exception as e:
        messagebox.showerror("Error", f"Ocurrió un error al ejecutar la trayectoria: {str(e)}")
def execute_trajectory4():
    try:
        main4()
    except Exception as e:
        messagebox.showerror("Error", f"Ocurrió un error al ejecutar la trayectoria: {str(e)}")
def execute_trajectory5():
    try:
        main5()
    except Exception as e:
        messagebox.showerror("Error", f"Ocurrió un error al ejecutar la trayectoria: {str(e)}")
def execute_trajectory6():
    try:
        main6()
    except Exception as e:
        messagebox.showerror("Error", f"Ocurrió un error al ejecutar la trayectoria: {str(e)}")
def execute_trajectory7():
    try:
        main7()
    except Exception as e:
        messagebox.showerror("Error", f"Ocurrió un error al ejecutar la trayectoria: {str(e)}")
def execute_trajectory8():
    try:
        main8()
    except Exception as e:
        messagebox.showerror("Error", f"Ocurrió un error al ejecutar la trayectoria: {str(e)}")
def execute_trajectory9():
    try:
        main9()
    except Exception as e:
        messagebox.showerror("Error", f"Ocurrió un error al ejecutar la trayectoria: {str(e)}")
def execute_trajectory10():
    try:
        main10()
    except Exception as e:
        messagebox.showerror("Error", f"Ocurrió un error al ejecutar la trayectoria: {str(e)}")
def execute_trajectory11():
    try:
        main11()
    except Exception as e:
        messagebox.showerror("Error", f"Ocurrió un error al ejecutar la trayectoria: {str(e)}")
def execute_trajectory12():
    try:
        main12()
    except Exception as e:
        messagebox.showerror("Error", f"Ocurrió un error al ejecutar la trayectoria: {str(e)}")
def execute_trajectory13():
    try:
        main13()
    except Exception as e:
        messagebox.showerror("Error", f"Ocurrió un error al ejecutar la trayectoria: {str(e)}")
def execute_trajectory14():
    try:
        main14()
    except Exception as e:
        messagebox.showerror("Error", f"Ocurrió un error al ejecutar la trayectoria: {str(e)}")
def execute_trajectory15():
    try:
        main15()
    except Exception as e:
        messagebox.showerror("Error", f"Ocurrió un error al ejecutar la trayectoria: {str(e)}")
def execute_trajectory16():
    try:
        main16()
    except Exception as e:
        messagebox.showerror("Error", f"Ocurrió un error al ejecutar la trayectoria: {str(e)}")
def execute_trajectory17():
    try:
        main17()
    except Exception as e:
        messagebox.showerror("Error", f"Ocurrió un error al ejecutar la trayectoria: {str(e)}")
def execute_trajectory18():
    try:
        main18()
    except Exception as e:
        messagebox.showerror("Error", f"Ocurrió un error al ejecutar la trayectoria: {str(e)}")
def execute_trajectory19():
    try:
        main19()
    except Exception as e:
        messagebox.showerror("Error", f"Ocurrió un error al ejecutar la trayectoria: {str(e)}")
def post_process_coordenates(data, resolution, lower_limit, upper_limit):
    columns = [[] for _ in range(len(data[0]))]

    # Append the first point to the respective column list
    for value, column in zip(data[0], columns):
        column.append(value)

    # Iterate over the rows in the data array
    for i in range(len(data) - 1):
        point1 = data[i]
        point2 = data[i + 1]

        x1, y1, z1, roll1, pitch1, yaw1, e1, time1 = point1
        x2, y2, z2, roll2, pitch2, yaw2, e2, time2 = point2

        # Calculate the Euclidean distance between the two points
        distance = np.sqrt((x2 - x1)**2 + (y2 - y1)**2 + (z2 - z1)**2)

        if distance > upper_limit:
            # Calculate the number of intermediate points based on the spacing
            num_points = int(distance / resolution)

            # Calculate the increments for each column
            x_increment = (x2 - x1) / num_points
            y_increment = (y2 - y1) / num_points
            z_increment = (z2 - z1) / num_points
            roll_increment = (roll2 - roll1) / num_points
            pitch_increment = (pitch2 - pitch1) / num_points
            yaw_increment = (yaw2 - yaw1) / num_points
            extruder_increment = (e2 - e1) / num_points
            time_increment = (time2 - time1) / num_points

            # Iterate and interpolate values for each column
            for j in range(num_points):
                x = x1 + j * x_increment
                y = y1 + j * y_increment
                z = z1 + j * z_increment
                roll = roll1 + j * roll_increment
                pitch = pitch1 + j * pitch_increment
                yaw = yaw1 + j * yaw_increment
                e = e1 + j * extruder_increment
                t = time1 + j * time_increment

                # Append the interpolated values to the respective column list
                if j != 0:
                    columns[0].append(x)
                    columns[1].append(y)
                    columns[2].append(z)
                    columns[3].append(roll)
                    columns[4].append(pitch)
                    columns[5].append(yaw)
                    columns[6].append(e)
                    columns[7].append(t)
        elif distance >= lower_limit:
            # Append the original point to the respective column list
            for value, column in zip(point1, columns):
                column.append(value)

    # Add the last point of the original data
    for value, column in zip(data[-1], columns):
        column.append(value)

    # Transpose the column lists to get the interpolated data in columns
    return np.transpose(columns)

def extract_values_from_gcode(filename):
    data = []
    # Initial values for X, Y, Z, E, R, P, Y, NANOSEC
    last_values = [0, 0, 0, 0, 0, 0, 0]
    time_sum_sec = 0  # Variable to store the cumulative sum of time_nanosec

    with open(filename, 'r') as file:
        print("Processing GCode...")
        for line in file:
            if line.startswith('G1'):
                words = line.split()
                x = next(
                    (float(word[1:]) for word in words if word.startswith('X')), last_values[0])
                y = next(
                    (float(word[1:]) for word in words if word.startswith('Y')), last_values[1])
                z = next(
                    (float(word[1:]) for word in words if word.startswith('Z')), last_values[2])
                e = next(
                    (float(word[1:]) for word in words if word.startswith('E')), last_values[3])
                f = next(
                    (float(word[1:]) for word in words if word.startswith('F')), last_values[4])

                if f != last_values[4]:
                    f /= 60 * 1000  # Divide F value by 60
                if x != last_values[0]:
                    x /= 1000
                if y != last_values[1]:
                    y /= 1000
                if z != last_values[2]:
                    z /= 1000

                if last_values == [0, 0, 0, 0, 0, 0, 0]:
                    last_values = [x, y, z, e, f, 180, -90]
                    continue

                distance = sqrt(
                    (x - last_values[0]) ** 2 + (y - last_values[1]) ** 2 + (z - last_values[2]) ** 2)
                time_sum_sec += (distance / f)
                atan2_base = -degrees(atan2(x + base2station[0,3], y - 0.170 + base2station[1,3])) - 90

                last_values = [x, y, z, e, f, 180, atan2_base]
                row = np.array(last_values[:3] + [0] + last_values[5:] + last_values[3:4] + [
                               time_sum_sec], dtype=np.float64)  # Create the modified row as a NumPy array

                data.append(row)  # Append the modified row to data

    return np.array(data)
def inverse_kinematics_scorbot(position_goal, rotation_goal, extruder_pos, sec_time_between_points, nanosec_time_between_points, wrist):
    global hotend2wrist_roll, base2station
    rotation = [radians(rotation_goal[0]),
                radians(rotation_goal[1]),
                radians(rotation_goal[2])]
    position_t = np.array([[position_goal[0]],
                           [position_goal[1]],
                           [position_goal[2]]])

    scale_perception = np.array([0, 0, 0, 1])

    Rx = np.array([[1, 0, 0],
                   [0, cos(rotation[0]), -sin(rotation[0])],
                   [0, sin(rotation[0]), cos(rotation[0])]])

    Ry = np.array([[cos(rotation[1]), 0, sin(rotation[1])],
                   [0, 1, 0],
                   [-sin(rotation[1]), 0, cos(rotation[1])]])

    Rz = np.array([[cos(rotation[2]), -sin(rotation[2]), 0],
                   [sin(rotation[2]), cos(rotation[2]), 0],
                   [0, 0, 1]])

    R = (Rz.dot(Ry)).dot(Rx)
    T = np.vstack((np.hstack((R, position_t)), scale_perception))
    arm_transform = np.matmul(base2station, np.matmul(
        T, hotend2wrist_roll)) if wrist else T

    link_1 = 0.450
    link_2 = 0.220
    link_3 = 0.220
    a_2 = 0.025

    nx = arm_transform[0, 0]
    ny = arm_transform[1, 0]
    sx = arm_transform[0, 1]
    sy = arm_transform[1, 1]
    ax = arm_transform[0, 2]
    ay = arm_transform[1, 2]
    az = arm_transform[2, 2]

    offset_slide_base = 0.17
    slide_base = 0.0

    d_1 = offset_slide_base + slide_base

    x = arm_transform[0, 3]
    y = arm_transform[1, 3] - d_1
    z = arm_transform[2, 3] - link_1

    theta_1 = atan2(-x, y)
    theta_5 = atan2(nx*cos(theta_1) + ny*sin(theta_1),
                    sx*cos(theta_1) + sy*sin(theta_1))

    w = sqrt(x**2 + y**2) - a_2
    cos_theta_3 = (z**2 + w**2 - link_2**2 - link_3**2)/(2*link_2*link_3)

    if cos_theta_3 > 1:
        print("\n OUT OF WORKSPACE ROBOT! BY THETA3\n")
        return -1.0, -1.0, -1.0, -1.0, -1.0, -1.0
    else:
        sin_theta_3 = -sqrt(1-cos_theta_3**2)

    theta_3 = atan2(sin_theta_3, cos_theta_3)

    k1 = link_2 + link_3*cos_theta_3
    k2 = link_3*sin_theta_3

    theta_2 = atan2(z, w) - atan2(k2, k1)

    theta_4 = atan2(az, -ax*sin(theta_1) + ay*cos(theta_1)) - \
        (theta_2 + theta_3)

    np.set_printoptions(precision=5, suppress=True)
    result = [slide_base, theta_1, theta_2, theta_3, theta_4, theta_5,
              extruder_pos, sec_time_between_points, nanosec_time_between_points, ]
    return result
def inverse_kinematics_scorbot2(position_goal, rotation_goal, extruder_pos, sec_time_between_points, nanosec_time_between_points, wrist):
    global hotend2wrist_roll, base2station
    global theta1global
    global theta2global
    global theta3global
    global theta4global
    global theta5global
    rotation = [(rotation_goal[0]),
                (rotation_goal[1]),
                (rotation_goal[2])]
    position_t = np.array([[position_goal[0]],
                           [position_goal[1]],
                           [position_goal[2]]])

    scale_perception = np.array([0, 0, 0, 1])

    Rx = np.array([[1, 0, 0],
                   [0, cos(rotation[0]), -sin(rotation[0])],
                   [0, sin(rotation[0]), cos(rotation[0])]])

    Ry = np.array([[cos(rotation[1]), 0, sin(rotation[1])],
                   [0, 1, 0],
                   [-sin(rotation[1]), 0, cos(rotation[1])]])

    Rz = np.array([[cos(rotation[2]), -sin(rotation[2]), 0],
                   [sin(rotation[2]), cos(rotation[2]), 0],
                   [0, 0, 1]])
    x1 = position_goal[0]
    y1 = position_goal[1]
    z1 = position_goal[2]
    if((0.16>x1>-0.14)and(0.35>y1>0.0)and(z1<0.58)):
                            messagebox.showwarning("Error","Error:Estás chocando con los límites del espacio del robot")
                            return -1.0, -1.0, -1.0, -1.0, -1.0, -1.0
    R = (Rz.dot(Ry)).dot(Rx)
    print(R)
    T = np.vstack((np.hstack((R, position_t)), scale_perception))
    print(T)
    arm_transform = T

    link_1 = 0.450
    link_2 = 0.220
    link_3 = 0.220
    a_2 = 0.025

    nx = arm_transform[0, 0]
    ny = arm_transform[1, 0]
    sx = arm_transform[0, 1]
    sy = arm_transform[1, 1]
    ax = arm_transform[0, 2]
    ay = arm_transform[1, 2]
    az = arm_transform[2, 2]

    offset_slide_base = 0.17
    slide_base = 0.0

    d_1 = offset_slide_base + slide_base

    x = arm_transform[0, 3]
    y = arm_transform[1, 3] - d_1
    z = arm_transform[2, 3] - link_1

    theta_1 = atan2(-x, y)
    theta_5 = atan2(nx*cos(theta_1) + ny*sin(theta_1),
                    sx*cos(theta_1) + sy*sin(theta_1))

    w = sqrt(x**2 + y**2) - a_2
    cos_theta_3 = (z**2 + w**2 - link_2**2 - link_3**2)/(2*link_2*link_3)

    if cos_theta_3 > 1:
        messagebox.showwarning("Error","Valor imposible en los límites físicos de las articulaciones")
        theta_1 = theta1global
        theta_2 = theta2global
        theta_3 = theta3global
        theta_4 = theta4global
        theta_5 = theta5global
        result = [slide_base, theta_1, theta_2, theta_3, theta_4, theta_5,
              extruder_pos, sec_time_between_points, nanosec_time_between_points, ]
        return result
    else:
        sin_theta_3 = -sqrt(1-cos_theta_3**2)

    theta_3 = atan2(sin_theta_3, cos_theta_3)

    k1 = link_2 + link_3*cos_theta_3
    k2 = link_3*sin_theta_3

    theta_2 = atan2(z, w) - atan2(k2, k1)

    theta_4 = atan2(az, -ax*sin(theta_1) + ay*cos(theta_1)) - \
        (theta_2 + theta_3)
    theta_4 = 1.67
    theta_5 = 0.0
    if (theta_1 <= -2.03 or theta_1 >= 0.8) or \
                            (theta_2 <= -0.323 or theta_2 >= 2.08) or \
                            (theta_3 <= -2.4 or theta_3 >= 0.0) or \
                            (theta_4 <= 0 or theta_4 >= 1.7) or \
                            (theta_5 <= -0.52 or theta_5 >= 0.52):
                             messagebox.showwarning("Error","Valor imposible en los límites físicos de las articulaciones")
                             print(theta3)
                             theta_1 = theta1global
                             theta_2 = theta2global
                             theta_3 = theta3global
                             theta_4 = theta4global
                             theta_5 = theta5global
                             result = [slide_base, theta_1, theta_2, theta_3, theta_4, theta_5,
              extruder_pos, sec_time_between_points, nanosec_time_between_points, ]
                             return result
    
    theta1global = theta_1
    theta2global = theta_2
    theta3global = theta_3
    theta4global = theta_4
    theta5global = theta_5
    np.set_printoptions(precision=5, suppress=True)
    result = [slide_base, theta_1, theta_2, theta_3, theta_4, theta_5,
              extruder_pos, sec_time_between_points, nanosec_time_between_points, ]
    global xglobal
    xglobal = x1
    print("Valor global de X1 al mandar coordenadas manuales")
    print(xglobal)
    global yglobal
    yglobal = y1
    print("Valor global de Y1 al mandar coordenadas manuales")
    print(yglobal)
    global zglobal
    zglobal = z1
    print("Valor global de z1 al mandar coordenadas manuales")
    print (zglobal)
    return result
def create_or_destroy_window(window_type):
    global root  # Asegúrate de tener acceso a la ventana principal

    # Cerrar todas las subventanas abiertas
    for widget in root.winfo_children():
        if isinstance(widget, Toplevel):
            widget.destroy()

    if window_type == "print_window":
        open_print_window()
    elif window_type == "manual_move_window":
        open_manual_move_window()
    elif window_type == "info_window":
        open_info_window()
    elif window_type == "settings_window":
        open_settings_window()  
# Funciones para los comandos de los botones (debes definirlas)
def select_file(listbox):
    global gcode_filename
    selected_file = listbox.get(listbox.curselection())
    print(f"Archivo seleccionado: {selected_file}")
    print(selected_file)
    ubicacion_destino = "/home/ras-rpi/Codigosg"
    gcode_filename = os.path.join(ubicacion_destino,os.path.basename(selected_file))
    print(gcode_filename)
    gcode_filename = f'{gcode_filename}'
    print(gcode_filename)

def preheat():
    print("Precalentamiento iniciado...")

def print_file(listbox):
    selected_file = listbox.get(listbox.curselection())
    print(f"Imprimiendo archivo: {selected_file}")
    
    #Lectura de archivos en una ruta especificada """ VARIABLE ROOT """
    #archivos = [a for a in listdir(root) if isfile(join(root, a))]

    window = Toplevel()
    window.title("Print Window")
    
     # Configurar el Scrollbar
    scrollbar = Scrollbar(window)
    scrollbar.pack(side='right', fill='y')

    # Crear el Listbox y adjuntar el Scrollbar
    listbox = Listbox(window, yscrollcommand=scrollbar.set)
    listbox.pack(side='left', fill='both', expand=True)
    scrollbar.config(command=listbox.yview)

    # Leer los archivos de la carpeta especificada y añadirlos al Listbox
    path = '/home/ras-rpi' # Reemplaza con la ruta a tu carpeta
    files = os.listdir(path)
    for file in files:
        listbox.insert('end', file)

    #tk.Label(window, text="Print Window").pack()
def open_manual_move_window():
    
    
    # Cerrar todas las subventanas abiertas
    for widget in root.winfo_children():
        if isinstance(widget, Toplevel):
            widget.destroy() 
    window = Toplevel(root)
    window.geometry("800x250")
    window.resizable(False, False)
    window.configure(bg='gray')

    left_frame = Frame(window, bg='gray', relief='groove', bd=2)
    left_frame.pack(side='left', fill='both', expand=True, padx=10)

    # Centrar y hacer concéntricos los elementos en top_left_frame
    top_left_frame = Frame(left_frame, bg='gray', relief='groove', bd=2)
    top_left_frame.pack(side='top', fill='both', expand=True, pady=10)

    # Configurar filas y columnas para expansión
    top_left_frame.grid_rowconfigure(0, weight=1)
    top_left_frame.grid_rowconfigure(1, weight=1)
    top_left_frame.grid_rowconfigure(2, weight=1)
    top_left_frame.grid_columnconfigure(0, weight=1)
    top_left_frame.grid_columnconfigure(1, weight=1)
    top_left_frame.grid_columnconfigure(2, weight=1)

    bottom_left_frame = Frame(left_frame, bg='gray', relief='groove', bd=2)
    bottom_left_frame.pack(side='bottom', fill='both', expand=True)

    # Marco derecho
    right_frame = Frame(window, bg='gray', relief='groove', bd=2)
    right_frame.pack(side='right', fill='both', expand=True)

    # Subframe para los botones
    buttons_frame = Frame(right_frame, bg='gray')
    buttons_frame.pack(side='left', fill='both', expand=True, padx=10)

    # Subframe para la imagen y el botón a su derecha
    image_frame = Frame(right_frame, bg='gray')
    image_frame.pack(side='left', fill='both', expand=True, padx=10)

     # X
    global entry_x
    label_x = Label(top_left_frame, text="X:", bg='gray', anchor='w')
    label_x.grid(row=0, column=0, padx=5, pady=5, ipadx=10, ipady=5)
    entry_x = Entry(top_left_frame, width=10)
    entry_x.grid(row=0, column=1, padx=5, pady=5, ipadx=10, ipady=5)

    # Y
    global entry_y
    label_y = Label(top_left_frame, text="Y:", bg='gray', anchor='w')
    label_y.grid(row=1, column=0, padx=5, pady=5, ipadx=10, ipady=5)
    entry_y = Entry(top_left_frame, width=10)
    entry_y.grid(row=1, column=1, padx=5, pady=5, ipadx=10, ipady=5)

    # Z
    global entry_z
    label_z = Label(top_left_frame, text="Z:", bg='gray', anchor='w')
    label_z.grid(row=2, column=0, padx=5, pady=5, ipadx=10, ipady=5)
    entry_z = Entry(top_left_frame, width=10)
    entry_z.grid(row=2, column=1, padx=5, pady=5, ipadx=10, ipady=5)


    #botón home
    button_home = Button(top_left_frame, text="Home", width=8, command = start_trajectory3)
    button_home.grid(row=1, column=2, padx=3, pady=3, ipadx=1, ipady=1)
    # Botón Posicionar
    button_posicionar = Button(top_left_frame, text="Posicionar", width=8, command = start_trajectory2)
    button_posicionar.grid(row=2, column=2, padx=3, pady=3, ipadx=1, ipady=1)  # Columna 2 para el botón "Posicionar"



    # Crear un frame adicional para los botones en bottom_left_frame
    buttons_container = Frame(bottom_left_frame, bg='gray')
    buttons_container.pack(expand=True)

    # Botones en buttons_container usando grid
    button_width = 4  # Ancho en caracteres
    button_height = 1  # Altura en líneas de texto

    button_1_1 = Button(buttons_container, text="X+", width=button_width, height=button_height, command = start_trajectory4)
    button_1_1.grid(row=0, column=0, padx=5, pady=5)

    button_1_2 = Button(buttons_container, text="Y+", width=button_width, height=button_height,  command = start_trajectory6)
    button_1_2.grid(row=0, column=1, padx=5, pady=5)

    button_1_3 = Button(buttons_container, text="Z+", width=button_width, height=button_height, command = start_trajectory8)
    button_1_3.grid(row=0, column=2, padx=5, pady=5)

    button_2_1 = Button(buttons_container, text="X-", width=button_width, height=button_height, command = start_trajectory5)
    button_2_1.grid(row=1, column=0, padx=5, pady=5)

    button_2_2 = Button(buttons_container, text="Y-", width=button_width, height=button_height, command = start_trajectory7)
    button_2_2.grid(row=1, column=1, padx=5, pady=5)

    button_2_3 = Button(buttons_container, text="Z-", width=button_width, height=button_height, command = start_trajectory9)
    button_2_3.grid(row=1, column=2, padx=5, pady=5)

    # Crear botón y etiqueta juntos en su propio Frame
    button_label_frame_space = Frame(buttons_frame, bg='gray')
    button_label_frame_space.pack(pady=5)
    Label(button_label_frame_space, text='\b', fg='gray', bg='gray').pack(side='left')

    button_label_frame = Frame(buttons_frame, bg='gray')
    button_label_frame.pack(pady=5)
    Label(button_label_frame, text="Muñeca", bg='gray').pack(side='left')
    Button(button_label_frame, text='+', width=button_width, height=button_height, command = start_trajectory18).pack(side='left')

    button_label_frame2 = Frame(buttons_frame, bg='gray')
    button_label_frame2.pack(pady=5)
    Label(button_label_frame2, text="Codo 3 ", bg='gray').pack(side='left')
    Button(button_label_frame2, text='+', width=button_width, height=button_height, command = start_trajectory16).pack(side='left')

    button_label_frame3 = Frame(buttons_frame, bg='gray')
    button_label_frame3.pack(pady=5)
    Label(button_label_frame3, text="Codo 2 ", bg='gray').pack(side='left')
    Button(button_label_frame3, text='+', width=button_width, height=button_height, command = start_trajectory14).pack(side='left')

    button_label_frame4 = Frame(buttons_frame, bg='gray')
    button_label_frame4.pack(pady=5)
    Label(button_label_frame4, text="Codo 1 ", bg='gray').pack(side='left')
    Button(button_label_frame4, text='+', width=button_width, height=button_height,command = start_trajectory12).pack(side='left')

    button_label_frame5 = Frame(buttons_frame, bg='gray')
    button_label_frame5.pack(pady=5)
    Label(button_label_frame5, text="    Base  ", bg='gray').pack(side='left')
    Button(button_label_frame5, text='+', width=button_width, height=button_height, command = start_trajectory10).pack(side='left')

    # Carga y muestra la imagen en image_frame
    photo = PhotoImage(file="./scorbotimage.png")  
    label_with_image = Label(image_frame, image=photo, bg='gray')
    label_with_image.pack(side='left', pady=5)
    label_with_image.photo = photo

    # Botón a la derecha de la imagen
    space = Frame(image_frame, bg='gray')
    space.pack(pady=5)
    Label(space, text='\b', fg='gray', bg='gray').pack(side='left')
   

    button_m1 = Frame(image_frame, bg='gray')
    button_m1.pack(pady=5)
    Button(button_m1, text='-', width=button_width, height=button_height,command = start_trajectory19).pack(side='left', padx=5)

    button_m2 = Frame(image_frame, bg='gray')
    button_m2.pack(pady=5)
    Button(button_m2, text='-', width=button_width, height=button_height, command = start_trajectory17).pack(side='left', padx=5)

    button_m3 = Frame(image_frame, bg='gray')
    button_m3.pack(pady=5)
    Button(button_m3, text='-', width=button_width, height=button_height, command = start_trajectory15).pack(side='left', padx=5)

    button_m4 = Frame(image_frame, bg='gray')
    button_m4.pack(pady=5)
    Button(button_m4, text='-', width=button_width, height=button_height, command = start_trajectory13).pack(side='left', padx=5)

    button_m5 = Frame(image_frame, bg='gray')
    button_m5.pack(pady=5)
    Button(button_m5, text='-', width=button_width, height=button_height, command = start_trajectory11).pack(side='left', padx=5)

    root.mainloop()

    


def open_info_window():
    for widget in root.winfo_children():
        if isinstance(widget, Toplevel):
            widget.destroy()
    window = Toplevel(root)
    window.title("Info Window")
    window.geometry("300x200")
    window.resizable(False, False)

    # Especificaciones técnicas del robot
    specs = {
        "Modelo": "Scorbot UAO",
        "Versión": "2.0 con Impresion",
        "Temperatura cama": "60°-80°",
        "Temperatura extrusor":"150°-300°",
        "Peso Máximo": "100 kg",
    }

    # Crear y colocar las etiquetas de texto en la ventana
    for i, (spec, value) in enumerate(specs.items()):
        label_text = f"{spec}: {value}"
        label = Label(window, text=label_text, anchor="center", justify=tk.CENTER)
        label.pack(fill='x', padx=10, pady=10)  # Ajusta el relleno según necesites

def open_settings_window():
    for widget in root.winfo_children():
        if isinstance(widget, Toplevel):
            widget.destroy()
    window = Toplevel(root)
    window.title("Settings Window")
    tk.Label(window, text="Settings Window").pack()
def show_main_interface():
    
    # Crear archivo temporal al abrir la interfaz principal
    tempfile_path = os.path.join(tempfile.gettempdir(), "ARCHIVO_TEMPORTAL")
    if not os.path.exists(tempfile_path):
        with open(tempfile_path, "w") as temp_file:
            temp_file.write("Contenido del archivo temporal")
    # Crear la ventana principal
    global root
    root = tk.Tk()
    root.title("Scorbot User Interface")
    global bold_font
    bold_font = font.Font(weight="bold", size=10)

    # Configura el tamaño mínimo de la ventana si es necesario
    root.minsize(770, 300)  # Ancho de 2 botones x alto de 2 botones

    # Configurar el peso de las filas y columnas para que los widgets se expandan correctamente
    root.grid_rowconfigure(0, weight=1)
    root.grid_rowconfigure(1, weight=1)
    root.grid_columnconfigure(0, weight=1)
    root.grid_columnconfigure(1, weight=1)

    # Crear y agregar los botones a la ventana principal
    print_button = tk.Button(root, text="Print", command=lambda: create_or_destroy_window("print_window"), font=bold_font)
    print_button.grid(row=0, column=0, sticky='nsew', padx=10, pady=10)

    manual_move_button = tk.Button(root, text="Manual move", command=open_manual_move_window, font=bold_font)
    manual_move_button.grid(row=0, column=1, sticky='nsew', padx=10, pady=10)

    info_button = tk.Button(root, text="Info", command=open_info_window, font=bold_font)
    info_button.grid(row=1, column=0, sticky='nsew', padx=10, pady=5)

    settings_button = tk.Button(root, text="Settings", command=open_settings_window, font=bold_font)
    settings_button.grid(row=1, column=1, sticky='nsew', padx=10, pady=5)
    root.mainloop()
    mini_root.destroy()
def button_pressed(button):
    global buttons_pressed_count
    buttons_pressed_count += 1
    button.config(state=tk.DISABLED)  # Deshabilita el botón después de ser presionado
    if buttons_pressed_count == 3:
        controladores()
        mini_root.destroy()  # Cierra la interfaz mini
        show_main_interface()  # Muestra la interfaz principal
        
    elif buttons_pressed_count == 1:
        button2.config(state=tk.NORMAL)  # Habilita el botón 2 cuando se presiona el botón 1
        lower_control()
    elif buttons_pressed_count == 2:
        upper_control()
        button3.config(state=tk.NORMAL)  # Habilita el botón 3 cuando se presiona el botón 2
def mostrar():
    x = entry_x.get()
    print(x)
class ScorbotActionClient(Node):

    def __init__(self):
        super().__init__('scorbot_action_client')
        self._action_client = ActionClient(
            self, FollowJointTrajectory, '/joint_trajectory_position_controller/follow_joint_trajectory')
        self.goal_msg = FollowJointTrajectory.Goal()
        self.goal_msg.trajectory.joint_names = ["slide_base_joint", "body_joint", "shoulder_joint", "elbow_joint",
                                                "wrist_joint", "roll_wrist_joint", "extruder_screw_joint"]

    def calculateTrajectory(self):
        points_xyz_rpy = extract_values_from_gcode(gcode_filename)
        # print(points_xyz_rpy)

        post_process_xyz_rpy = post_process_coordenates(
            points_xyz_rpy, 0.001, 0.0005, 0.003)
        print(post_process_xyz_rpy)
        trajectory_points = []
        for data_point in post_process_xyz_rpy:
            position = [float(i) for i in data_point[0:3]]
            rotation = [float(i) for i in data_point[3:6]]
            extruder_pos = float(data_point[6])
            absolute_time = data_point[7]
            sec_time_between_points = int(absolute_time)
            nanosec_time_between_points = int(
                (absolute_time - sec_time_between_points) * 1e9)
            result = inverse_kinematics_scorbot(
                position, rotation, extruder_pos, sec_time_between_points, nanosec_time_between_points, True)
            trajectory_points.append(result)
        data = "a\n"
        ser2.write(data.encode())
        for positions in trajectory_points:
            trajectory_point = JointTrajectoryPoint()
            trajectory_point.positions = positions[:7]
            # print(positions[7:])
            trajectory_point.time_from_start = Duration(
                sec=int(positions[7:8][0]), nanosec=int(positions[8:9][0]))
            self.goal_msg.trajectory.points.append(trajectory_point)
    def calculateTrajectory2(self):
        x1 = float(entry_x.get())
        y1 = float(entry_y.get())
        z1 = float(entry_z.get())
        thetax1 = 0.0
        post_process_xyz_rpy = [[x1, y1, z1, thetax1, 0, 0, 0]]  # Lista de listas
        puntos_de_trayectoria = []

        for punto_de_dato in post_process_xyz_rpy:
            posicion = punto_de_dato[:3]  # Extrae la posición del punto de dato
            rotacion = punto_de_dato[3:6]  # Extrae la rotación del punto de dato
            print(posicion)
            print(rotacion)
            posicion_extrusor = punto_de_dato[6]
            tiempo_absoluto = 0  # Puedes ajustar este valor según sea necesario
            resultado = inverse_kinematics_scorbot2(
                posicion, rotacion, 0.0, 3.0, int(3*1e9), True)
            print(resultado)
            puntos_de_trayectoria.append(resultado)
            
        for positions in puntos_de_trayectoria:
            trajectory_point = JointTrajectoryPoint()
            trajectory_point.positions = positions[:7]
            trajectory_point.time_from_start = Duration(
                sec=int(positions[7:8][0]), nanosec=int(positions[8:9][0]))
            self.goal_msg.trajectory.points.append(trajectory_point)
    def calculateTrajectory3(self):
        x1 = 0.29
        y1 = 0.47
        z1 = 0.44
        thetax1 = 0.0
        post_process_xyz_rpy = [[x1, y1, z1, thetax1, 0, 0, 0]]  # Lista de listas
        puntos_de_trayectoria = []

        for punto_de_dato in post_process_xyz_rpy:
            posicion = punto_de_dato[:3]  # Extrae la posición del punto de dato
            rotacion = punto_de_dato[3:6]  # Extrae la rotación del punto de dato
            print(posicion)
            print(rotacion)
            posicion_extrusor = punto_de_dato[6]
            tiempo_absoluto = 0  # Puedes ajustar este valor según sea necesario
            resultado = inverse_kinematics_scorbot2(
                posicion, rotacion, 0.0, 3.0, int(3*1e9), True)
            print(resultado)
            puntos_de_trayectoria.append(resultado)

        for positions in puntos_de_trayectoria:
            trajectory_point = JointTrajectoryPoint()
            trajectory_point.positions = positions[:7]
            trajectory_point.time_from_start = Duration(
                sec=int(positions[7:8][0]), nanosec=int(positions[8:9][0]))
            self.goal_msg.trajectory.points.append(trajectory_point)
    def calculateTrajectory4(self):
        x1 = xglobal + 0.005
        print("X1 GLOBAL ACTUAL")
        print(x1)
        y1 = yglobal
        z1 = zglobal
        thetax1 = 0.0
        post_process_xyz_rpy = [[x1, y1, z1, thetax1, 0, 0, 0]]  # Lista de listas
        puntos_de_trayectoria = []

        for punto_de_dato in post_process_xyz_rpy:
            posicion = punto_de_dato[:3]  # Extrae la posición del punto de dato
            rotacion = punto_de_dato[3:6]  # Extrae la rotación del punto de dato
            print(posicion)
            print(rotacion)
            posicion_extrusor = punto_de_dato[6]
            tiempo_absoluto = 0  # Puedes ajustar este valor según sea necesario
            resultado = inverse_kinematics_scorbot2(
                posicion, rotacion, 0.0, 1.0, int(1*1e9), True)
            print(resultado)
            puntos_de_trayectoria.append(resultado)

        for positions in puntos_de_trayectoria:
            trajectory_point = JointTrajectoryPoint()
            trajectory_point.positions = positions[:7]
            trajectory_point.time_from_start = Duration(
                sec=int(positions[7:8][0]), nanosec=int(positions[8:9][0]))
            self.goal_msg.trajectory.points.append(trajectory_point)
    def calculateTrajectory5(self):
        x1 = xglobal - 0.005
        print("X1 GLOBAL ACTUAL")
        print(x1)
        y1 = yglobal
        z1 = zglobal
        thetax1 = 0.0
        post_process_xyz_rpy = [[x1, y1, z1, thetax1, 0, 0, 0]]  # Lista de listas
        puntos_de_trayectoria = []

        for punto_de_dato in post_process_xyz_rpy:
            posicion = punto_de_dato[:3]  # Extrae la posición del punto de dato
            rotacion = punto_de_dato[3:6]  # Extrae la rotación del punto de dato
            print(posicion)
            print(rotacion)
            posicion_extrusor = punto_de_dato[6]
            tiempo_absoluto = 0  # Puedes ajustar este valor según sea necesario
            resultado = inverse_kinematics_scorbot2(
                posicion, rotacion, 0.0, 1.0, int(1*1e9), True)
            print(resultado)
            puntos_de_trayectoria.append(resultado)

        for positions in puntos_de_trayectoria:
            trajectory_point = JointTrajectoryPoint()
            trajectory_point.positions = positions[:7]
            trajectory_point.time_from_start = Duration(
                sec=int(positions[7:8][0]), nanosec=int(positions[8:9][0]))
            self.goal_msg.trajectory.points.append(trajectory_point)
    def calculateTrajectory6(self):
        x1 = xglobal
        print("X1 GLOBAL ACTUAL")
        print(x1)
        y1 = yglobal + 0.005
        z1 = zglobal
        thetax1 = 0.0
        post_process_xyz_rpy = [[x1, y1, z1, thetax1, 0, 0, 0]]  # Lista de listas
        puntos_de_trayectoria = []

        for punto_de_dato in post_process_xyz_rpy:
            posicion = punto_de_dato[:3]  # Extrae la posición del punto de dato
            rotacion = punto_de_dato[3:6]  # Extrae la rotación del punto de dato
            print(posicion)
            print(rotacion)
            posicion_extrusor = punto_de_dato[6]
            tiempo_absoluto = 0  # Puedes ajustar este valor según sea necesario
            resultado = inverse_kinematics_scorbot2(
                posicion, rotacion, 0.0, 1.0, int(1*1e9), True)
            print(resultado)
            puntos_de_trayectoria.append(resultado)

        for positions in puntos_de_trayectoria:
            trajectory_point = JointTrajectoryPoint()
            trajectory_point.positions = positions[:7]
            trajectory_point.time_from_start = Duration(
                sec=int(positions[7:8][0]), nanosec=int(positions[8:9][0]))
            self.goal_msg.trajectory.points.append(trajectory_point)
    def calculateTrajectory7(self):
        x1 = xglobal
        print("X1 GLOBAL ACTUAL")
        print(x1)
        y1 = yglobal - 0.005
        z1 = zglobal
        thetax1 = 0.0
        post_process_xyz_rpy = [[x1, y1, z1, thetax1, 0, 0, 0]]  # Lista de listas
        puntos_de_trayectoria = []

        for punto_de_dato in post_process_xyz_rpy:
            posicion = punto_de_dato[:3]  # Extrae la posición del punto de dato
            rotacion = punto_de_dato[3:6]  # Extrae la rotación del punto de dato
            print(posicion)
            print(rotacion)
            posicion_extrusor = punto_de_dato[6]
            tiempo_absoluto = 0  # Puedes ajustar este valor según sea necesario
            resultado = inverse_kinematics_scorbot2(
                posicion, rotacion, 0.0, 1.0, int(1*1e9), True)
            print(resultado)
            puntos_de_trayectoria.append(resultado)

        for positions in puntos_de_trayectoria:
            trajectory_point = JointTrajectoryPoint()
            trajectory_point.positions = positions[:7]
            trajectory_point.time_from_start = Duration(
                sec=int(positions[7:8][0]), nanosec=int(positions[8:9][0]))
            self.goal_msg.trajectory.points.append(trajectory_point)
    def calculateTrajectory8(self):
        x1 = xglobal
        print("X1 GLOBAL ACTUAL")
        print(x1)
        y1 = yglobal 
        z1 = zglobal + 0.005
        thetax1 = 0.0
        post_process_xyz_rpy = [[x1, y1, z1, thetax1, 0, 0, 0]]  # Lista de listas
        puntos_de_trayectoria = []

        for punto_de_dato in post_process_xyz_rpy:
            posicion = punto_de_dato[:3]  # Extrae la posición del punto de dato
            rotacion = punto_de_dato[3:6]  # Extrae la rotación del punto de dato
            print(posicion)
            print(rotacion)
            posicion_extrusor = punto_de_dato[6]
            tiempo_absoluto = 0  # Puedes ajustar este valor según sea necesario
            resultado = inverse_kinematics_scorbot2(
                posicion, rotacion, 0.0, 1.0, int(1*1e9), True)
            print(resultado)
            puntos_de_trayectoria.append(resultado)

        for positions in puntos_de_trayectoria:
            trajectory_point = JointTrajectoryPoint()
            print(positions)
            trajectory_point.positions = positions[:7]
            trajectory_point.time_from_start = Duration(
                sec=int(positions[7:8][0]), nanosec=int(positions[8:9][0]))
            self.goal_msg.trajectory.points.append(trajectory_point)
    def calculateTrajectory9(self):
        x1 = xglobal
        print("X1 GLOBAL ACTUAL")
        print(x1)
        y1 = yglobal 
        z1 = zglobal - 0.005
        thetax1 = 0.0
        post_process_xyz_rpy = [[x1, y1, z1, thetax1, 0, 0, 0]]  # Lista de listas
        puntos_de_trayectoria = []

        for punto_de_dato in post_process_xyz_rpy:
            posicion = punto_de_dato[:3]  # Extrae la posición del punto de dato
            rotacion = punto_de_dato[3:6]  # Extrae la rotación del punto de dato
            print(posicion)
            print(rotacion)
            posicion_extrusor = punto_de_dato[6]
            tiempo_absoluto = 0  # Puedes ajustar este valor según sea necesario
            resultado = inverse_kinematics_scorbot2(
                posicion, rotacion, 0.0, 1.0, int(1*1e9), True)
            print(resultado)
            puntos_de_trayectoria.append(resultado)

        for positions in puntos_de_trayectoria:
            trajectory_point = JointTrajectoryPoint()
            trajectory_point.positions = positions[:7]
            trajectory_point.time_from_start = Duration(
                sec=int(positions[7:8][0]), nanosec=int(positions[8:9][0]))
            self.goal_msg.trajectory.points.append(trajectory_point) 
    def calculateTrajectory10(self):
        global theta1global, theta2global, theta3global, theta4global, theta5global, theta6global
        puntos_de_trayectoria = []
        theta1local = theta1global + 0.0174533
        print("ESTE ES EL VALOR NUEVO DE THETA1GLOBAL")
        print(theta1local)
        if (theta1local <= -2.03 or theta1local >= 0.8):
                             messagebox.showwarning("Error","Valor imposible en los límites físicos de las articulaciones")
                             puntos_de_trayectoria = [[0.0, float(theta1global), float(theta2global), float(theta3global), float(theta4global), float(theta5global), 0.0, 1.0, int(1*1e9)]]
        else:
            theta1global = theta1local
            global xglobal
            xglobal = -(a_2+link_2*cos(theta2global)+link_3*cos(theta2global+theta3global))*sin(theta1global)
            print("Valor global de X1 al mandar coordenadas manuales")
            print(xglobal)
            global yglobal
            yglobal = a_2*cos(theta1global)+d_1+(link_2*cos(theta1global)*cos(theta2global))+link_3*cos(theta1global)*cos(theta2global+theta3global)
            print("Valor global de Y1 al mandar coordenadas manuales")
            print(yglobal)
            global zglobal
            zglobal = link_1 + link_2*sin(theta2global) + link_3*sin(theta2global+theta3global)
            puntos_de_trayectoria = [[0.0, float(theta1global), float(theta2global), float(theta3global), float(theta4global), float(theta5global), 0.0, 1.0, int(1*1e9)]]
            print(puntos_de_trayectoria)
            print(type(puntos_de_trayectoria))
        

       

        for positions in puntos_de_trayectoria:
            trajectory_point = JointTrajectoryPoint()
            trajectory_point.positions = positions[:7]
            trajectory_point.time_from_start = Duration(
                sec=int(positions[7:8][0]), nanosec=int(positions[8:9][0]))
            self.goal_msg.trajectory.points.append(trajectory_point) 
    def calculateTrajectory11(self):
        global theta1global, theta2global, theta3global, theta4global, theta5global, theta6global
        puntos_de_trayectoria = []
        theta1local = theta1global - 0.0174533
        print("ESTE ES EL VALOR NUEVO DE THETA1GLOBAL")
        print(theta1local)
        if (theta1local <= -2.03 or theta1local >= 0.8):
                             messagebox.showwarning("Error","Valor imposible en los límites físicos de las articulaciones")
                             puntos_de_trayectoria = [[0.0, float(theta1global), float(theta2global), float(theta3global), float(theta4global), float(theta5global), 0.0, 1.0, int(1*1e9)]]
        else:
            theta1global = theta1local
            global xglobal
            xglobal = -(a_2+link_2*cos(theta2global)+link_3*cos(theta2global+theta3global))*sin(theta1global)
            print("Valor global de X1 al mandar coordenadas manuales")
            print(xglobal)
            global yglobal
            yglobal = a_2*cos(theta1global)+d_1+(link_2*cos(theta1global)*cos(theta2global))+link_3*cos(theta1global)*cos(theta2global+theta3global)
            print("Valor global de Y1 al mandar coordenadas manuales")
            print(yglobal)
            global zglobal
            zglobal = link_1 + link_2*sin(theta2global) + link_3*sin(theta2global+theta3global) 
            puntos_de_trayectoria = [[0.0, float(theta1global), float(theta2global), float(theta3global), float(theta4global), float(theta5global), 0.0, 1.0, int(1*1e9)]]
            print(puntos_de_trayectoria)
            print(type(puntos_de_trayectoria))
        

       

        for positions in puntos_de_trayectoria:
            trajectory_point = JointTrajectoryPoint()
            trajectory_point.positions = positions[:7]
            trajectory_point.time_from_start = Duration(
                sec=int(positions[7:8][0]), nanosec=int(positions[8:9][0]))
            self.goal_msg.trajectory.points.append(trajectory_point)
    def calculateTrajectory12(self):
        global theta1global, theta2global, theta3global, theta4global, theta5global, theta6global
        puntos_de_trayectoria = []
        theta2local = theta2global + 0.0174533
        print("ESTE ES EL VALOR NUEVO DE THETA2GLOBAL")
        print(theta2global)
        if (theta2local <= -0.323 or theta2local >= 2.06):
                             messagebox.showwarning("Error","Valor imposible en los límites físicos de las articulaciones")
                             puntos_de_trayectoria = [[0.0, float(theta1global), float(theta2global), float(theta3global), float(theta4global), float(theta5global), 0.0, 1.0, int(1*1e9)]]
        else:
            theta2global = theta2local
            global xglobal
            xglobal = -(a_2+link_2*cos(theta2global)+link_3*cos(theta2global+theta3global))*sin(theta1global)
            print("Valor global de X1 al mandar coordenadas manuales")
            print(xglobal)
            global yglobal
            yglobal = a_2*cos(theta1global)+d_1+(link_2*cos(theta1global)*cos(theta2global))+link_3*cos(theta1global)*cos(theta2global+theta3global)
            print("Valor global de Y1 al mandar coordenadas manuales")
            print(yglobal)
            global zglobal
            zglobal = link_1 + link_2*sin(theta2global) + link_3*sin(theta2global+theta3global) 
            puntos_de_trayectoria = [[0.0, float(theta1global), float(theta2global), float(theta3global), float(theta4global), float(theta5global), 0.0, 1.0, int(1*1e9)]]
            print(puntos_de_trayectoria)
            print(type(puntos_de_trayectoria))
        

       

        for positions in puntos_de_trayectoria:
            trajectory_point = JointTrajectoryPoint()
            trajectory_point.positions = positions[:7]
            trajectory_point.time_from_start = Duration(
                sec=int(positions[7:8][0]), nanosec=int(positions[8:9][0]))
            self.goal_msg.trajectory.points.append(trajectory_point)  
    def calculateTrajectory13(self):
        global theta1global, theta2global, theta3global, theta4global, theta5global, theta6global
        puntos_de_trayectoria = []
        theta2local = theta2global - 0.0174533
        print("ESTE ES EL VALOR NUEVO DE THETA2GLOBAL")
        print(theta2global)
        if (theta2local <= -0.323 or theta2local >= 2.06):
                             messagebox.showwarning("Error","Valor imposible en los límites físicos de las articulaciones")
                             puntos_de_trayectoria = [[0.0, float(theta1global), float(theta2global), float(theta3global), float(theta4global), float(theta5global), 0.0, 1.0, int(1*1e9)]]
        else:
            theta2global = theta2local
            global xglobal
            xglobal = -(a_2+link_2*cos(theta2global)+link_3*cos(theta2global+theta3global))*sin(theta1global)
            print("Valor global de X1 al mandar coordenadas manuales")
            print(xglobal)
            global yglobal
            yglobal = a_2*cos(theta1global)+d_1+(link_2*cos(theta1global)*cos(theta2global))+link_3*cos(theta1global)*cos(theta2global+theta3global)
            print("Valor global de Y1 al mandar coordenadas manuales")
            print(yglobal)
            global zglobal
            zglobal = link_1 + link_2*sin(theta2global) + link_3*sin(theta2global+theta3global) 
            puntos_de_trayectoria = [[0.0, float(theta1global), float(theta2global), float(theta3global), float(theta4global), float(theta5global), 0.0, 1.0, int(1*1e9)]]
            print(puntos_de_trayectoria)
            print(type(puntos_de_trayectoria))
        

       

        for positions in puntos_de_trayectoria:
            trajectory_point = JointTrajectoryPoint()
            trajectory_point.positions = positions[:7]
            trajectory_point.time_from_start = Duration(
                sec=int(positions[7:8][0]), nanosec=int(positions[8:9][0]))
            self.goal_msg.trajectory.points.append(trajectory_point)
    def calculateTrajectory14(self):
        global theta1global, theta2global, theta3global, theta4global, theta5global, theta6global
        puntos_de_trayectoria = []
        theta3local = theta3global + 0.0174533
        print("ESTE ES EL VALOR NUEVO DE THETA3GLOBAL")
        print(theta3global)
        if (theta3local <= -2.4 or theta3local >= 0.0):
                             messagebox.showwarning("Error","Valor imposible en los límites físicos de las articulaciones")
                             print(theta3local)
                             puntos_de_trayectoria = [[0.0, float(theta1global), float(theta2global), float(theta3global), float(theta4global), float(theta5global), 0.0, 1.0, int(1*1e9)]]
        else:
            theta3global = theta3local
            global xglobal
            xglobal = -(a_2+link_2*cos(theta2global)+link_3*cos(theta2global+theta3global))*sin(theta1global)
            print("Valor global de X1 al mandar coordenadas manuales")
            print(xglobal)
            global yglobal
            yglobal = a_2*cos(theta1global)+d_1+(link_2*cos(theta1global)*cos(theta2global))+link_3*cos(theta1global)*cos(theta2global+theta3global)
            print("Valor global de Y1 al mandar coordenadas manuales")
            print(yglobal)
            global zglobal
            zglobal = link_1 + link_2*sin(theta2global) + link_3*sin(theta2global+theta3global) 
            puntos_de_trayectoria = [[0.0, float(theta1global), float(theta2global), float(theta3global), float(theta4global), float(theta5global), 0.0, 1.0, int(1*1e9)]]
            print(puntos_de_trayectoria)
            print(type(puntos_de_trayectoria))
        

       

        for positions in puntos_de_trayectoria:
            trajectory_point = JointTrajectoryPoint()
            trajectory_point.positions = positions[:7]
            trajectory_point.time_from_start = Duration(
                sec=int(positions[7:8][0]), nanosec=int(positions[8:9][0]))
            self.goal_msg.trajectory.points.append(trajectory_point)   
    def calculateTrajectory15(self):
        global theta1global, theta2global, theta3global, theta4global, theta5global, theta6global
        puntos_de_trayectoria = []
        theta3local = theta3global - 0.0174533
        print("ESTE ES EL VALOR NUEVO DE THETA3GLOBAL")
        print(theta3local)
        if (theta3local <= -2.4 or theta3local >= 0.0):
                             messagebox.showwarning("Error","Valor imposible en los límites físicos de las articulaciones")
                             puntos_de_trayectoria = [[0.0, float(theta1global), float(theta2global), float(theta3global), float(theta4global), float(theta5global), 0.0, 1.0, int(1*1e9)]]
        else:
            theta3global = theta3local
            global xglobal
            xglobal = -(a_2+link_2*cos(theta2global)+link_3*cos(theta2global+theta3global))*sin(theta1global)
            print("Valor global de X1 al mandar coordenadas manuales")
            print(xglobal)
            global yglobal
            yglobal = a_2*cos(theta1global)+d_1+(link_2*cos(theta1global)*cos(theta2global))+link_3*cos(theta1global)*cos(theta2global+theta3global)
            print("Valor global de Y1 al mandar coordenadas manuales")
            print(yglobal)
            global zglobal
            zglobal = link_1 + link_2*sin(theta2global) + link_3*sin(theta2global+theta3global) 
            puntos_de_trayectoria = [[0.0, float(theta1global), float(theta2global), float(theta3global), float(theta4global), float(theta5global), 0.0, 1.0, int(1*1e9)]]
            print(puntos_de_trayectoria)
            print(type(puntos_de_trayectoria))
        

       

        for positions in puntos_de_trayectoria:
            trajectory_point = JointTrajectoryPoint()
            trajectory_point.positions = positions[:7]
            trajectory_point.time_from_start = Duration(
                sec=int(positions[7:8][0]), nanosec=int(positions[8:9][0]))
            self.goal_msg.trajectory.points.append(trajectory_point)
    def calculateTrajectory16(self):
        global theta1global, theta2global, theta3global, theta4global, theta5global, theta6global
        puntos_de_trayectoria = []
        theta4local = theta4global + 0.0174533 + 0.0174533
        print("ESTE ES EL VALOR NUEVO DE THETA4GLOBAL")
        print(theta4global)
        if (theta4local <= -1.0 or theta4local >= 1.68):
                             messagebox.showwarning("Error","Valor imposible en los límites físicos de las articulaciones")
                             puntos_de_trayectoria = [[0.0, float(theta1global), float(theta2global), float(theta3global), float(theta4global), float(theta5global), 0.0, 1.0, int(1*1e9)]]
        else:
            theta4global = theta4local
            global xglobal
            xglobal = -(a_2+link_2*cos(theta2global)+link_3*cos(theta2global+theta3global))*sin(theta1global)
            print("Valor global de X1 al mandar coordenadas manuales")
            print(xglobal)
            global yglobal
            yglobal = a_2*cos(theta1global)+d_1+(link_2*cos(theta1global)*cos(theta2global))+link_3*cos(theta1global)*cos(theta2global+theta3global)
            print("Valor global de Y1 al mandar coordenadas manuales")
            print(yglobal)
            global zglobal
            zglobal = link_1 + link_2*sin(theta2global) + link_3*sin(theta2global+theta3global)
            puntos_de_trayectoria = [[0.0, float(theta1global), float(theta2global), float(theta3global), float(theta4global), float(theta5global), 0.0, 1.0, int(1*1e9)]]
            print(puntos_de_trayectoria)
            print(type(puntos_de_trayectoria))
        

       

        for positions in puntos_de_trayectoria:
            trajectory_point = JointTrajectoryPoint()
            trajectory_point.positions = positions[:7]
            trajectory_point.time_from_start = Duration(
                sec=int(positions[7:8][0]), nanosec=int(positions[8:9][0]))
            self.goal_msg.trajectory.points.append(trajectory_point)
    def calculateTrajectory17(self):
        global theta1global, theta2global, theta3global, theta4global, theta5global, theta6global
        puntos_de_trayectoria = []
        theta4local = theta4global - 0.0174533 - 0.0174533
        print("ESTE ES EL VALOR NUEVO DE THETA4GLOBAL")
        print(theta4global)
        if (theta4local <= -1.0 or theta4local >= 1.68):
                             messagebox.showwarning("Error","Valor imposible en los límites físicos de las articulaciones")
                             puntos_de_trayectoria = [[0.0, float(theta1global), float(theta2global), float(theta3global), float(theta4global), float(theta5global), 0.0, 1.0, int(1*1e9)]]
        else:
            theta4global = theta4local
            global xglobal
            xglobal = -(a_2+link_2*cos(theta2global)+link_3*cos(theta2global+theta3global))*sin(theta1global)
            print("Valor global de X1 al mandar coordenadas manuales")
            print(xglobal)
            global yglobal
            yglobal = a_2*cos(theta1global)+d_1+(link_2*cos(theta1global)*cos(theta2global))+link_3*cos(theta1global)*cos(theta2global+theta3global)
            print("Valor global de Y1 al mandar coordenadas manuales")
            print(yglobal)
            global zglobal
            zglobal = link_1 + link_2*sin(theta2global) + link_3*sin(theta2global+theta3global)
            puntos_de_trayectoria = [[0.0, float(theta1global), float(theta2global), float(theta3global), float(theta4global), float(theta5global), 0.0, 1.0, int(1*1e9)]]
            print(puntos_de_trayectoria)
            print(type(puntos_de_trayectoria))
        

       

        for positions in puntos_de_trayectoria:
            trajectory_point = JointTrajectoryPoint()
            trajectory_point.positions = positions[:7]
            trajectory_point.time_from_start = Duration(
                sec=int(positions[7:8][0]), nanosec=int(positions[8:9][0]))
            self.goal_msg.trajectory.points.append(trajectory_point)
    def calculateTrajectory18(self):
        global theta1global, theta2global, theta3global, theta4global, theta5global, theta6global
        puntos_de_trayectoria = []
        theta5local = theta5global + 0.0174533 + 0.0174533
        print("ESTE ES EL VALOR NUEVO DE THETA5GLOBAL")
        print(theta5local)
        if (theta5local <= -0.52 or theta5local >= 0.52):
                             messagebox.showwarning("Error","Valor imposible en los límites físicos de las articulaciones")
                             puntos_de_trayectoria = [[0.0, float(theta1global), float(theta2global), float(theta3global), float(theta4global), float(theta5global), 0.0, 1.0, int(1*1e9)]]
        else:
            theta5global = theta5local
            global xglobal
            xglobal = -(a_2+link_2*cos(theta2global)+link_3*cos(theta2global+theta3global))*sin(theta1global)
            print("Valor global de X1 al mandar coordenadas manuales")
            print(xglobal)
            global yglobal
            yglobal = a_2*cos(theta1global)+d_1+(link_2*cos(theta1global)*cos(theta2global))+link_3*cos(theta1global)*cos(theta2global+theta3global)
            print("Valor global de Y1 al mandar coordenadas manuales")
            print(yglobal)
            global zglobal
            zglobal = link_1 + link_2*sin(theta2global) + link_3*sin(theta2global+theta3global) 
            puntos_de_trayectoria = [[0.0, float(theta1global), float(theta2global), float(theta3global), float(theta4global), float(theta5global), 0.0, 1.0, int(1*1e9)]]
            print(puntos_de_trayectoria)
            print(type(puntos_de_trayectoria))
        

        for positions in puntos_de_trayectoria:
            trajectory_point = JointTrajectoryPoint()
            trajectory_point.positions = positions[:7]
            trajectory_point.time_from_start = Duration(
                sec=int(positions[7:8][0]), nanosec=int(positions[8:9][0]))
            self.goal_msg.trajectory.points.append(trajectory_point)
    def calculateTrajectory19(self):
        global theta1global, theta2global, theta3global, theta4global, theta5global, theta6global
        puntos_de_trayectoria = []
        theta5local = theta5global - 0.0174533 - 0.0174533
        print("ESTE ES EL VALOR NUEVO DE THETA5GLOBAL")
        print(theta5global)
        if (theta5local <= -0.52 or theta5local >= 0.52):

                             messagebox.showwarning("Error","Valor imposible en los límites físicos de las articulaciones")
                             puntos_de_trayectoria = [[0.0, float(theta1global), float(theta2global), float(theta3global), float(theta4global), float(theta5global), 0.0, 1.0, int(1*1e9)]]
        else:
            theta5global = theta5local
            global xglobal
            xglobal = -(a_2+link_2*cos(theta2global)+link_3*cos(theta2global+theta3global))*sin(theta1global)
            print("Valor global de X1 al mandar coordenadas manuales")
            print(xglobal)
            global yglobal
            yglobal = a_2*cos(theta1global)+d_1+(link_2*cos(theta1global)*cos(theta2global))+link_3*cos(theta1global)*cos(theta2global+theta3global)
            print("Valor global de Y1 al mandar coordenadas manuales")
            print(yglobal)
            global zglobal
            zglobal = link_1 + link_2*sin(theta2global) + link_3*sin(theta2global+theta3global) 
            puntos_de_trayectoria = [[0.0, float(theta1global), float(theta2global), float(theta3global), float(theta4global), float(theta5global), 0.0, 1.0, int(1*1e9)]]
            print(puntos_de_trayectoria)
            print(type(puntos_de_trayectoria))
        

       

        for positions in puntos_de_trayectoria:
            trajectory_point = JointTrajectoryPoint()
            trajectory_point.positions = positions[:7]
            trajectory_point.time_from_start = Duration(
                sec=int(positions[7:8][0]), nanosec=int(positions[8:9][0]))
            self.goal_msg.trajectory.points.append(trajectory_point)
    def startTrajectory(self):
        self.calculateTrajectory()

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(
            self.goal_msg, feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)
    def startTrajectory2(self):

        self.calculateTrajectory2()

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(
            self.goal_msg, feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)
    def startTrajectory3(self):
        self.calculateTrajectory3()
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            self.goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
    def startTrajectory4(self):
        self.calculateTrajectory4()
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            self.goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
    def startTrajectory5(self):
        self.calculateTrajectory5()
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            self.goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
    def startTrajectory6(self):
        self.calculateTrajectory6()
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            self.goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
    def startTrajectory7(self):
        self.calculateTrajectory7()
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            self.goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
    def startTrajectory8(self):
        self.calculateTrajectory8()
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            self.goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
    def startTrajectory9(self):
        self.calculateTrajectory9()
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            self.goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
    def startTrajectory10(self):
        self.calculateTrajectory10()
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            self.goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
    def startTrajectory11(self):
        self.calculateTrajectory11()
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            self.goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
    def startTrajectory12(self):
        self.calculateTrajectory12()
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            self.goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
    def startTrajectory13(self):
        self.calculateTrajectory13()
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            self.goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
    def startTrajectory14(self):
        self.calculateTrajectory14()
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            self.goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
    def startTrajectory15(self):
        self.calculateTrajectory15()
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            self.goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
    def startTrajectory16(self):
        self.calculateTrajectory16()
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            self.goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
    def startTrajectory17(self):
        self.calculateTrajectory17()
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            self.goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
    def startTrajectory18(self):
        self.calculateTrajectory18()
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            self.goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
    def startTrajectory19(self):
        self.calculateTrajectory19()
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            self.goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        
        self._goal_handle = goal_handle

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        data = "p\n"
        ser2.write(data.encode())
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.error_code))
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(
            'Received feedback: {0}'.format(feedback.actual.positions))
        
    def cancel_goal(self):
        self.get_logger().info('Canceling goal')
        future = self._goal_handle.cancel_goal_async()
        future.add_done_callback(self.cancel_done)

    def cancel_done(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('Goal successfully canceled')
        else:
            self.get_logger().info('Goal failed to cancel')

        rclpy.shutdown()
def main(args=None):
    rclpy.init(args=args)

    action_client = ScorbotActionClient()
    action_client.startTrajectory()
    try:
        rclpy.spin(action_client)
    except KeyboardInterrupt:
        action_client.cancel_goal()
def main2(args=None):
    rclpy.init(args=args)

    action_client = ScorbotActionClient()
    action_client.startTrajectory2()
    try:
        rclpy.spin(action_client)
    except KeyboardInterrupt:
        action_client.cancel_goal()
def main3(args=None):
    rclpy.init(args=args)

    action_client = ScorbotActionClient()
    action_client.startTrajectory3()
    try:
        rclpy.spin(action_client)
    except KeyboardInterrupt:
        action_client.cancel_goal()
def main4(args=None):
    rclpy.init(args=args)

    action_client = ScorbotActionClient()
    action_client.startTrajectory4()
    try:
        rclpy.spin(action_client)
    except KeyboardInterrupt:
        action_client.cancel_goal()
def main5(args=None):
    rclpy.init(args=args)

    action_client = ScorbotActionClient()
    action_client.startTrajectory5()
    try:
        rclpy.spin(action_client)
    except KeyboardInterrupt:
        action_client.cancel_goal()
def main6(args=None):
    rclpy.init(args=args)

    action_client = ScorbotActionClient()
    action_client.startTrajectory6()
    try:
        rclpy.spin(action_client)
    except KeyboardInterrupt:
        action_client.cancel_goal()
def main7(args=None):
    rclpy.init(args=args)

    action_client = ScorbotActionClient()
    action_client.startTrajectory7()
    try:
        rclpy.spin(action_client)
    except KeyboardInterrupt:
        action_client.cancel_goal()
def main8(args=None):
    rclpy.init(args=args)

    action_client = ScorbotActionClient()
    action_client.startTrajectory8()
    try:
        rclpy.spin(action_client)
    except KeyboardInterrupt:
        action_client.cancel_goal()
def main9(args=None):
    rclpy.init(args=args)

    action_client = ScorbotActionClient()
    action_client.startTrajectory9()
    try:
        rclpy.spin(action_client)
    except KeyboardInterrupt:
        action_client.cancel_goal()
def main10(args=None):
    rclpy.init(args=args)

    action_client = ScorbotActionClient()
    action_client.startTrajectory10()
    try:
        rclpy.spin(action_client)
    except KeyboardInterrupt:
        action_client.cancel_goal()
def main11(args=None):
    rclpy.init(args=args)

    action_client = ScorbotActionClient()
    action_client.startTrajectory11()
    try:
        rclpy.spin(action_client)
    except KeyboardInterrupt:
        action_client.cancel_goal()
def main12(args=None):
    rclpy.init(args=args)

    action_client = ScorbotActionClient()
    action_client.startTrajectory12()
    try:
        rclpy.spin(action_client)
    except KeyboardInterrupt:
        action_client.cancel_goal()
def main13(args=None):
    rclpy.init(args=args)

    action_client = ScorbotActionClient()
    action_client.startTrajectory13()
    try:
        rclpy.spin(action_client)
    except KeyboardInterrupt:
        action_client.cancel_goal()
def main14(args=None):
    rclpy.init(args=args)

    action_client = ScorbotActionClient()
    action_client.startTrajectory14()
    try:
        rclpy.spin(action_client)
    except KeyboardInterrupt:
        action_client.cancel_goal()
def main15(args=None):
    rclpy.init(args=args)

    action_client = ScorbotActionClient()
    action_client.startTrajectory15()
    try:
        rclpy.spin(action_client)
    except KeyboardInterrupt:
        action_client.cancel_goal()
def main16(args=None):
    rclpy.init(args=args)

    action_client = ScorbotActionClient()
    action_client.startTrajectory16()
    try:
        rclpy.spin(action_client)
    except KeyboardInterrupt:
        action_client.cancel_goal()
def main17(args=None):
    rclpy.init(args=args)

    action_client = ScorbotActionClient()
    action_client.startTrajectory17()
    try:
        rclpy.spin(action_client)
    except KeyboardInterrupt:
        action_client.cancel_goal()
def main18(args=None):
    rclpy.init(args=args)

    action_client = ScorbotActionClient()
    action_client.startTrajectory18()
    try:
        rclpy.spin(action_client)
    except KeyboardInterrupt:
        action_client.cancel_goal()
def main19(args=None):
    rclpy.init(args=args)

    action_client = ScorbotActionClient()
    action_client.startTrajectory19()
    try:
        rclpy.spin(action_client)
    except KeyboardInterrupt:
        action_client.cancel_goal()
# Crear y configurar la ventana principal de la interfaz mini
mini_root = tk.Tk()
mini_root.title("Interfaz Mini")
mini_root.geometry("600x600")

# Título de la interfaz mini
label_title = tk.Label(mini_root, text="POSICIÓN PARA EJECUTAR LOS HOME", font=("Arial", 12, "bold"))
label_title.pack(pady=10)

# Cargar la imagen
image = Image.open("/home/ras-rpi/ScorbotUI_2/ScorbotUI/Scorbot_lab.JPG")  # Reemplaza "ruta_de_la_imagen.jpg" con la ruta correcta de tu imagen
image = image.rotate(-90, expand=True)  # Gira la imagen 90 grados a la derecha
image = image.resize((300, 300))   #mage.ANTIALIAS # Ajusta el tamaño de la imagen según sea necesario
photo = ImageTk.PhotoImage(image)
# Mostrar la imagen en la interfaz mini
label_image = tk.Label(mini_root, image=photo)
label_image.image = photo  # Guarda una referencia de la imagen para evitar que sea recolectada por el recolector de basura
label_image.pack(pady=10)

# Botones en la interfaz mini
button1 = tk.Button(mini_root, text="Home_Lower", command=lambda: button_pressed(button1),state=tk.NORMAL)
button1.pack(pady=10)

button2 = tk.Button(mini_root, text="Home_Upper", command=lambda: button_pressed(button2), state=tk.DISABLED)
button2.pack(pady=10)

button3 = tk.Button(mini_root, text="Controladores", command=lambda: button_pressed(button3), state=tk.DISABLED)
button3.pack(pady=10)
# Ejecutar el bucle principal de la aplicación para la interfaz mini

tempfile_path = os.path.join(tempfile.gettempdir(), "ARCHIVO_TEMPORTAL")
if os.path.exists(tempfile_path):
    mini_root.destroy()
    show_main_interface()  # Si el archivo existe, mostrar la interfaz principal
else:
    mini_root.mainloop()


