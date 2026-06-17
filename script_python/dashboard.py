import tkinter as tk
from tkinter import ttk
import serial
import serial.tools.list_ports
import json
import threading
import time

# ---------------------------------------------------------------
# CONFIGURACION
# ---------------------------------------------------------------
PUERTO = "COM5"
BAUDRATE = 115200
TIMEOUT = 1

# ---------------------------------------------------------------
# VENTANA PRINCIPAL
# ---------------------------------------------------------------

class Dashboard:
    def __init__(self, root):
        self.root = root
        self.root.title("SEGUIDOR DE LUZ — Telemetría")
        self.root.configure(bg="#1e1e1e")
        self.root.geometry("700x450")
        self.root.resizable(False, False)

        self.serial = None
        self.running = False
        self.ultima_trama = {}

        # --- CUADROS DE DATOS ---
        style = {"bg": "#2d2d2d", "fg": "#00ff88", "font": ("Consolas", 14)}
        label_style = {"bg": "#1e1e1e", "fg": "#888888", "font": ("Consolas", 10)}

        # MODO
        f_modo = tk.Frame(root, bg="#2d2d2d", bd=2, relief="ridge")
        f_modo.place(x=20, y=20, width=310, height=80)
        tk.Label(f_modo, text="MODO", **label_style).pack(anchor="w", padx=10, pady=(5,0))
        self.lbl_modo = tk.Label(f_modo, text="---", **style)
        self.lbl_modo.pack(anchor="w", padx=10)

        # ESTADO
        f_est = tk.Frame(root, bg="#2d2d2d", bd=2, relief="ridge")
        f_est.place(x=370, y=20, width=310, height=80)
        tk.Label(f_est, text="ESTADO", **label_style).pack(anchor="w", padx=10, pady=(5,0))
        self.lbl_est = tk.Label(f_est, text="---", **style)
        self.lbl_est.pack(anchor="w", padx=10)

        # ANGULO
        f_ang = tk.Frame(root, bg="#2d2d2d", bd=2, relief="ridge")
        f_ang.place(x=20, y=130, width=310, height=130)
        tk.Label(f_ang, text="ÁNGULO", **label_style).pack(anchor="w", padx=10, pady=(5,0))
        self.lbl_ang = tk.Label(f_ang, text="---°", **{**style, "font": ("Consolas", 36)})
        self.lbl_ang.pack(anchor="w", padx=10)

        # VOLTAJE
        f_v = tk.Frame(root, bg="#2d2d2d", bd=2, relief="ridge")
        f_v.place(x=370, y=130, width=310, height=130)
        tk.Label(f_v, text="VOLTAJE (mV)", **label_style).pack(anchor="w", padx=10, pady=(5,0))
        self.lbl_v = tk.Label(f_v, text="---mV", **{**style, "font": ("Consolas", 36), "fg": "#ffaa00"})
        self.lbl_v.pack(anchor="w", padx=10)

        # --- INDICADOR DE CONEXION ---
        self.lbl_conexion = tk.Label(root, text="⚫ Desconectado", bg="#1e1e1e", fg="#ff4444",
                                      font=("Consolas", 10))
        self.lbl_conexion.place(x=20, y=290)

        # --- ULTIMA TRAMA ---
        self.lbl_trama = tk.Label(root, text="", bg="#1e1e1e", fg="#555555",
                                   font=("Consolas", 9), anchor="w", justify="left")
        self.lbl_trama.place(x=20, y=330, width=660)

        # --- TIMER DE RECONEXION ---
        self.root.after(1000, self.conectar)

    def conectar(self):
        if self.serial and self.serial.is_open:
            return

        try:
            self.serial = serial.Serial(PUERTO, BAUDRATE, timeout=TIMEOUT)
            self.lbl_conexion.config(text="🟢 Conectado", fg="#00ff88")
            self.running = True
            hilo = threading.Thread(target=self.leer_serial, daemon=True)
            hilo.start()
        except Exception as e:
            self.lbl_conexion.config(text=f"⚫ Error: {e}", fg="#ff4444")
            # Reintentar en 3 segundos
            self.root.after(3000, self.conectar)

    def leer_serial(self):
        while self.running:
            try:
                if self.serial and self.serial.is_open and self.serial.in_waiting:
                    linea = self.serial.readline().decode("utf-8", errors="ignore").strip()
                    if linea:
                        try:
                            datos = json.loads(linea)
                            self.ultima_trama = datos
                            self.root.after(0, self.actualizar_labels, datos)
                        except json.JSONDecodeError:
                            pass  # linea no JSON, ignorar
                else:
                    time.sleep(0.01)
            except (serial.SerialException, OSError):
                self.running = False
                self.root.after(0, self.perder_conexion)
                break

    def actualizar_labels(self, datos):
        self.lbl_modo.config(text=datos.get("modo", "---"))
        self.lbl_est.config(text=datos.get("est", "---"))

        ang = datos.get("ang")
        if ang is not None:
            self.lbl_ang.config(text=f"{ang}°")

        v = datos.get("v")
        if v is not None:
            self.lbl_v.config(text=f"{v} mV")

        self.lbl_trama.config(text=f"Última trama: {json.dumps(datos)}")

    def perder_conexion(self):
        self.lbl_conexion.config(text="🔴 Desconectado", fg="#ff4444")
        if self.serial:
            try:
                self.serial.close()
            except:
                pass
        self.root.after(2000, self.conectar)

    def cerrar(self):
        self.running = False
        if self.serial and self.serial.is_open:
            try:
                self.serial.close()
            except:
                pass
        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    app = Dashboard(root)
    root.protocol("WM_DELETE_WINDOW", app.cerrar)
    root.mainloop()
