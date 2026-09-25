#!/usr/bin/env python3.9

import sys
import time
from collections import deque

import pyqtgraph as pg
import serial
import serial.tools.list_ports
from PyQt5.QtCore import Qt, QThread, QTimer, pyqtSignal, pyqtSlot
from PyQt5.QtGui import QFont
from PyQt5.QtWidgets import (
    QApplication,
    QComboBox,
    QFrame,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QPlainTextEdit,
    QPushButton,
    QSlider,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)


# -----------------------------------------------------------------------------
# WIDGET AUXILIAR: Indicador LED Circular com Estilo CSS
# -----------------------------------------------------------------------------
class LedIndicator(QLabel):
    def __init__(self, color_on="#00E676", color_off="#37474F", parent=None):
        super().__init__(parent)
        self.color_on = color_on
        self.color_off = color_off
        self.setFixedSize(18, 18)
        self.set_state(False)

    def set_state(self, active: bool):
        color = self.color_on if active else self.color_off
        border = "#B0BEC5" if active else "#263238"
        self.setStyleSheet(f"""
            background-color: {color};
            border-radius: 9px;
            border: 2px solid {border};
        """)

# -----------------------------------------------------------------------------
# THREAD DE LEITURA SERIAL (Com Fallback de Simulação)
# -----------------------------------------------------------------------------
class SerialWorker(QThread):
    # Sinais emitidos para a GUI principal:
    # (tempo_s, pulso_us, forca_g, estado_str)
    telemetry_received = pyqtSignal(float, float, float, str)
    raw_message_received = pyqtSignal(str)
    connection_status = pyqtSignal(bool, str)

    def __init__(self, port=None, baudrate=115200, simulation=False):
        super().__init__()
        self.port = port
        self.baudrate = baudrate
        self.simulation = simulation
        self.running = True
        self.ser = None

    def run(self):
        if self.simulation:
            self._run_simulation()
            return

        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            self.connection_status.emit(True, f"Conectado a {self.port}")
            
            while self.running:
                if self.ser.in_waiting > 0:
                    raw_line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if not raw_line:
                        continue
                    
                    # Se for linha de log/texto com prefixo '#' ou texto livre
                    if raw_line.startswith("#") or not ("," in raw_line):
                        self.raw_message_received.emit(raw_line)
                        continue

                    # Protocolo esperado do Teensy:
                    # tempo_ms,pulso_us,thrust_g,estado
                    # Exemplo: "14200,1600,432.5,Acquiring Data"
                    try:
                        parts = raw_line.split(',')
                        t_sec = float(parts[0]) / 1000.0
                        pulse_us = float(parts[1])
                        thrust_g = float(parts[2])
                        state = parts[3].strip() if len(parts) > 3 else "Unknown"
                        
                        self.telemetry_received.emit(t_sec, pulse_us, thrust_g, state)
                    except (ValueError, IndexError):
                        # Se não der parse numérico, joga na caixa de mensagens
                        self.raw_message_received.emit(raw_line)

        except serial.SerialException as e:
            self.connection_status.emit(False, str(e))
        finally:
            if self.ser and self.ser.is_open:
                self.ser.close()

    def _run_simulation(self):
        """Simulador caso queira testar a UI sem hardware conectado"""
        self.connection_status.emit(True, "Modo Simulação Ativo")
        t0 = time.time()
        pulse = 1000.0
        states = ["waiting Settlement", "Acquiring Data", "Test Completed"]
        state_idx = 0
        
        while self.running:
            t = time.time() - t0
            # Simula rampa/degrau de pulso
            pulse = 1000 + int((t % 20) / 2) * 100
            # Simula empuxo com ruído proporcional a pulso^2
            thrust = max(0.0, ((pulse - 1000) / 1000.0) ** 2 * 600.0 + (time.time() % 1) * 15)
            
            if t % 6 < 2:
                state_idx = 0
            elif t % 6 < 5:
                state_idx = 1
            else:
                state_idx = 2

            self.telemetry_received.emit(t, pulse, thrust, states[state_idx])
            
            if int(t * 10) % 20 == 0:
                self.raw_message_received.emit(f"[Teensy Log]: Step atualizado para {pulse:.0f} us")
                
            time.sleep(0.05)  # 20 Hz

    def send_command(self, cmd: str):
        if self.ser and self.ser.is_open:
            self.ser.write((cmd + "\n").encode('utf-8'))
        elif self.simulation:
            self.raw_message_received.emit(f"[Simulador Cmd Enviado]: {cmd}")

    def stop(self):
        self.running = False
        self.wait()

# -----------------------------------------------------------------------------
# JANELA PRINCIPAL DA INTERFACE GRÁFICA
# -----------------------------------------------------------------------------
class ThrustTestBenchGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Bancada de Teste de Empuxo - Teensy 4.1")
        self.resize(1100, 950)

        # Buffers de dados para os gráficos
        self.max_points = 600
        self.buf_time = deque(maxlen=self.max_points)
        self.buf_pulse = deque(maxlen=self.max_points)
        self.buf_thrust = deque(maxlen=self.max_points)

        self.serial_thread = None

        self._apply_dark_theme()
        self._init_ui()
        self._refresh_ports()

    def _apply_dark_theme(self):
        """Aparência moderna escura com correções para QComboBox no Linux"""
        self.setStyleSheet("""
            QMainWindow { background-color: #121212; }
            
            QGroupBox {
                border: 1px solid #37474F;
                border-radius: 6px;
                margin-top: 10px;
                font-weight: bold;
                color: #ECEFF1;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px;
            }
            
            QLabel { color: #CFD8DC; font-size: 13px; }
            
            /* --- QComboBox & QSpinBox (Aparência da caixa fechada) --- */
            QComboBox, QSpinBox {
                background-color: #1E1E1E;
                color: #FFFFFF;
                border: 1px solid #455A64;
                border-radius: 4px;
                padding: 4px 8px;
                min-height: 24px;
            }
            
            QComboBox:hover, QSpinBox:hover {
                border: 1px solid #00E5FF;
            }
            
            /* --- QComboBox (Aparência da setinha / Drop-down) --- */
            QComboBox::drop-down {
                subcontrol-origin: padding;
                subcontrol-position: top right;
                width: 20px;
                border-left-width: 1px;
                border-left-color: #455A64;
                border-top-right-radius: 2px;
                border-bottom-right-radius: 2px;
                background-color: #263238;
            }
            
            /* Usando um caractere unicode simples para a seta para baixo */
            QComboBox::down-arrow {
                image: none;
                /* Desenha uma setinha simples via CSS */
                border-left: 4px solid none;
                border-right: 4px solid none;
                border-top: 5px solid #FFFFFF;
                width: 0px;
                height: 0px;
            }

            /* --- QComboBox (Aparência da lista aberta) --- */
            QComboBox QAbstractItemView {
                background-color: #1E1E1E;
                color: #FFFFFF;
                selection-background-color: #37474F;
                selection-color: #00E5FF;
                border: 1px solid #455A64;
                outline: none; /* Remove contorno pontilhado estranho */
            }

            /* --- QSpinBox (Aparência das setinhas Up/Down) --- */
            QSpinBox::up-button, QSpinBox::down-button {
                background-color: #263238;
                border-left: 1px solid #455A64;
                width: 16px;
                subcontrol-origin: border;
            }
            QSpinBox::up-button {
                subcontrol-position: top right;
                border-top-right-radius: 3px;
            }
            QSpinBox::down-button {
                subcontrol-position: bottom right;
                border-bottom-right-radius: 3px;
                border-top: 1px solid #455A64;
            }
            QSpinBox::up-button:hover, QSpinBox::down-button:hover {
                background-color: #37474F;
            }
            QSpinBox::up-arrow {
                image: none;
                border-left: 4px solid none;
                border-right: 4px solid none;
                border-bottom: 5px solid #FFFFFF;
                width: 0px; height: 0px;
                margin-bottom: 2px;
            }
            QSpinBox::down-arrow {
                image: none;
                border-left: 4px solid none;
                border-right: 4px solid none;
                border-top: 5px solid #FFFFFF;
                width: 0px; height: 0px;
                margin-top: 2px;
            }
            /* Remove os botões feios nativos do tema do SO */
            QSpinBox {
                padding-right: 20px; 
            }
   
            /* --- Estilos dos QSlider --- */
            QSlider::groove:horizontal {
                border: 1px solid #37474F;
                height: 6px;
                background: #1E1E1E;
                margin: 2px 0;
                border-radius: 3px;
            }
            QSlider::sub-page:horizontal {
                background: #007C91;
                border: 1px solid #37474F;
                height: 6px;
                border-radius: 3px;
            }
            QSlider::handle:horizontal {
                background: #00E5FF;
                border: 1px solid #00B8D4;
                width: 14px;
                height: 14px;
                margin: -5px 0;
                border-radius: 7px;
            }
            QSlider::handle:horizontal:hover {
                background: #FFFFFF;
                border: 1px solid #00E5FF;
            }

            /* --- Botões Gerais --- */
            QPushButton {
                background-color: #263238;
                color: #FFFFFF;
                border: 1px solid #455A64;
                border-radius: 4px;
                padding: 6px 12px;
                font-weight: bold;
                min-height: 24px;
            }
            QPushButton:hover { background-color: #37474F; }
            QPushButton:pressed { background-color: #455A64; }
            
            /* Estilos específicos para os botões de ação */
            QPushButton#btnStart { background-color: #2E7D32; border: 1px solid #1B5E20; }
            QPushButton#btnStart:hover { background-color: #388E3C; }
            QPushButton#btnStart:pressed { background-color: #1B5E20; }
            
            QPushButton#btnTare { background-color: #F57C00; border: 1px solid #E65100; }
            QPushButton#btnTare:hover { background-color: #FB8C00; }
            QPushButton#btnTare:pressed { background-color: #E65100; }
            
            QPushButton#btnReset { background-color: #D32F2F; border: 1px solid #B71C1C; }
            QPushButton#btnReset:hover { background-color: #E53935; }
            QPushButton#btnReset:pressed { background-color: #B71C1C; }
            
            /* --- Área de Texto --- */
            QPlainTextEdit {
                background-color: #0A0A0A;
                color: #81C784;
                border: 1px solid #263238;
                font-family: 'Monospace';
                font-size: 11px;
            }
        """)

    def _init_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)
        main_layout.setSpacing(10)

        # -------------------------------------------------------------
        # 1. TOP BAR: Seleção da Porta Serial & Baud Rate
        # -------------------------------------------------------------
        top_bar = QHBoxLayout()
        
        lbl_port = QLabel("PORT:")
        self.combo_ports = QComboBox()
        self.combo_ports.setMinimumWidth(160)
        
        btn_refresh = QPushButton("Refresh")
        # btn_refresh.setMinimumWidth(32)
        btn_refresh.clicked.connect(self._refresh_ports)

        lbl_baud = QLabel("BAUD RATE:")
        self.combo_baud = QComboBox()
        self.combo_baud.addItems(["9600", "57600", "115200", "230400", "500000", "1000000"])
        self.combo_baud.setCurrentText("115200")

        self.btn_connect = QPushButton("Conectar")
        self.btn_connect.setStyleSheet("background-color: #1565C0; color: white;")
        self.btn_connect.clicked.connect(self._toggle_connection)

        self.btn_sim = QPushButton("Modo Simulação")
        self.btn_sim.clicked.connect(self._toggle_simulation)

        top_bar.addWidget(lbl_port)
        top_bar.addWidget(self.combo_ports)
        top_bar.addWidget(btn_refresh)
        top_bar.addSpacing(15)
        top_bar.addWidget(lbl_baud)
        top_bar.addWidget(self.combo_baud)
        top_bar.addSpacing(15)
        top_bar.addWidget(self.btn_connect)
        top_bar.addWidget(self.btn_sim)
        top_bar.addStretch()

        main_layout.addLayout(top_bar)

        # -------------------------------------------------------------
        # 2. SEÇÃO MÉDIA: CONFIGS (Esquerda) e STATUS/MENSAGENS (Direita)
        # -------------------------------------------------------------
        mid_layout = QHBoxLayout()

        # --- Bloco Esquerdo: Configurações ---
        box_configs = QGroupBox("CONFIGS:")
        grid_cfg = QGridLayout(box_configs)

        # 1. MIN PULSE (Slider + SpinBox)
        grid_cfg.addWidget(QLabel("MIN PULSE (µs):"), 0, 0)
        layout_min = QHBoxLayout()
        self.slider_min = QSlider(Qt.Horizontal)
        self.slider_min.setRange(800, 3000)
        self.slider_min.setValue(1000)
        self.spin_min = QSpinBox()
        self.spin_min.setRange(800, 3000)
        self.spin_min.setValue(1000)
        self.spin_min.setFixedWidth(90)
        self.slider_min.valueChanged.connect(self.spin_min.setValue)
        self.spin_min.valueChanged.connect(self.slider_min.setValue)
        layout_min.addWidget(self.slider_min)
        layout_min.addWidget(self.spin_min)
        grid_cfg.addLayout(layout_min, 0, 1)

        # 2. MAX PULSE (Slider + SpinBox)
        grid_cfg.addWidget(QLabel("MAX PULSE (µs):"), 1, 0)
        layout_max = QHBoxLayout()
        self.slider_max = QSlider(Qt.Horizontal)
        self.slider_max.setRange(800, 3000)
        self.slider_max.setValue(2000)
        self.spin_max = QSpinBox()
        self.spin_max.setRange(800, 3000)
        self.spin_max.setValue(2000)
        self.spin_max.setFixedWidth(90)
        self.slider_max.valueChanged.connect(self.spin_max.setValue)
        self.spin_max.valueChanged.connect(self.slider_max.setValue)
        layout_max.addWidget(self.slider_max)
        layout_max.addWidget(self.spin_max)
        grid_cfg.addLayout(layout_max, 1, 1)

        # 3. STEP (Slider + SpinBox)
        grid_cfg.addWidget(QLabel("STEP (µs):"), 2, 0)
        layout_step = QHBoxLayout()
        self.slider_step = QSlider(Qt.Horizontal)
        self.slider_step.setRange(5, 500)
        self.slider_step.setValue(50)
        self.spin_step = QSpinBox()
        self.spin_step.setRange(5, 500)
        self.spin_step.setValue(50)
        self.spin_step.setFixedWidth(90)
        self.slider_step.valueChanged.connect(self.spin_step.setValue)
        self.spin_step.valueChanged.connect(self.slider_step.setValue)
        layout_step.addWidget(self.slider_step)
        layout_step.addWidget(self.spin_step)
        grid_cfg.addLayout(layout_step, 2, 1)

        self.btn_start_test = QPushButton("Iniciar Teste / Enviar Config")
        self.btn_start_test.setObjectName("btnStart")
        self.btn_start_test.setStyleSheet("background-color: #2E7D32; color: white;")
        self.btn_start_test.clicked.connect(self._send_test_start)
        grid_cfg.addWidget(self.btn_start_test, 3, 0, 1, 2)
        
        # Novo Botão Tara (Laranja)
        self.btn_tare = QPushButton("Zerar Balança (TARE)")
        self.btn_tare.setObjectName("btnTare")
        self.btn_tare.clicked.connect(self._send_tare)
        grid_cfg.addWidget(self.btn_tare, 4, 0, 1, 2)

        # Novo Botão Reset (Vermelho)
        self.btn_reset = QPushButton("Parar Teste / Reset (IDLE)")
        self.btn_reset.setObjectName("btnReset")
        self.btn_reset.clicked.connect(self._send_reset)
        grid_cfg.addWidget(self.btn_reset, 5, 0, 1, 2)

        mid_layout.addWidget(box_configs, stretch=1)

        # --- Bloco Direito: Status e Mensagens ---
        box_status = QGroupBox("STATUS & MENSAGENS:")
        v_status = QVBoxLayout(box_status)

        # Luzes Indicadoras (LEDs)
        leds_layout = QHBoxLayout()
        self.led_running = LedIndicator(color_on="#00E676")  # Verde
        self.led_stopped = LedIndicator(color_on="#D50000")  # Vermelho
        self.led_stopped.set_state(True)  # Começa parado

        leds_layout.addWidget(self.led_running)
        leds_layout.addWidget(QLabel("EM ANDAMENTO"))
        leds_layout.addSpacing(20)
        leds_layout.addWidget(self.led_stopped)
        leds_layout.addWidget(QLabel("PARADO"))
        leds_layout.addStretch()
        v_status.addLayout(leds_layout)

        # State Machine Badge
        state_layout = QHBoxLayout()
        state_layout.addWidget(QLabel("State-Machine:"))
        self.lbl_state = QLabel("DESCONECTADO")
        self.lbl_state.setStyleSheet("""
            background-color: #263238; 
            color: #FFD54F; 
            font-weight: bold; 
            padding: 2px 6px; 
            border-radius: 3px;
        """)
        state_layout.addWidget(self.lbl_state)
        state_layout.addStretch()
        v_status.addLayout(state_layout)

        # Mensagens Recebidas
        v_status.addWidget(QLabel("MENSAGENS:"))
        self.txt_messages = QPlainTextEdit()
        self.txt_messages.setReadOnly(True)
        self.txt_messages.setMaximumHeight(85)
        v_status.addWidget(self.txt_messages)

        mid_layout.addWidget(box_status, stretch=2)
        main_layout.addLayout(mid_layout)

        # -------------------------------------------------------------
        # 3. GRÁFICO 1: PULSO ENVIADO (Pulsos x Tempo)
        # -------------------------------------------------------------
        box_pulse = QGroupBox()
        v_pulse = QVBoxLayout(box_pulse)
        
        h_pulse_header = QHBoxLayout()
        lbl_pulse_title = QLabel("PULSO ENVIADO:")
        lbl_pulse_title.setFont(QFont("Arial", 11, QFont.Bold))
        self.lbl_pulse_val = QLabel("--- µs")
        self.lbl_pulse_val.setFont(QFont("Monospace", 14, QFont.Bold))
        self.lbl_pulse_val.setStyleSheet("color: #00E5FF;")
        h_pulse_header.addWidget(lbl_pulse_title)
        h_pulse_header.addWidget(self.lbl_pulse_val)
        h_pulse_header.addStretch()
        v_pulse.addLayout(h_pulse_header)

        self.plot_pulse = pg.PlotWidget()
        self.plot_pulse.setLabel('left', 'Pulso', units='µs')
        self.plot_pulse.setLabel('bottom', 'Tempo', units='s')
        self.plot_pulse.showGrid(x=True, y=True, alpha=0.3)
        self.plot_pulse.setClipToView(True)
        self.curve_pulse = self.plot_pulse.plot(pen=pg.mkPen('#00E5FF', width=2))
        v_pulse.addWidget(self.plot_pulse)

        main_layout.addWidget(box_pulse, stretch=2)

        # -------------------------------------------------------------
        # 4. GRÁFICO 2: FORÇA / THRUST (Thrust x Tempo)
        # -------------------------------------------------------------
        box_thrust = QGroupBox()
        v_thrust = QVBoxLayout(box_thrust)

        h_thrust_header = QHBoxLayout()
        lbl_thrust_title = QLabel("FORÇA / Thrust medido:")
        lbl_thrust_title.setFont(QFont("Arial", 11, QFont.Bold))
        self.lbl_thrust_val = QLabel("--- g")
        self.lbl_thrust_val.setFont(QFont("Monospace", 14, QFont.Bold))
        self.lbl_thrust_val.setStyleSheet("color: #FFD54F;")
        h_thrust_header.addWidget(lbl_thrust_title)
        h_thrust_header.addWidget(self.lbl_thrust_val)
        h_thrust_header.addStretch()
        v_thrust.addLayout(h_thrust_header)

        self.plot_thrust = pg.PlotWidget()
        self.plot_thrust.setLabel('left', 'Thrust', units='g')
        self.plot_thrust.setLabel('bottom', 'Tempo', units='s')
        self.plot_thrust.showGrid(x=True, y=True, alpha=0.3)
        self.plot_thrust.setClipToView(True)
        self.curve_thrust = self.plot_thrust.plot(pen=pg.mkPen('#FFD54F', width=2))
        v_thrust.addWidget(self.plot_thrust)

        main_layout.addWidget(box_thrust, stretch=2)

    # -------------------------------------------------------------------------
    # AÇÕES E SLOTS
    # -------------------------------------------------------------------------
    def _refresh_ports(self):
        """Lista as portas COM /dev/tty do sistema"""
        self.combo_ports.clear()
        ports = serial.tools.list_ports.comports()
        for p in ports:
            self.combo_ports.addItem(p.device)
        
        # Se estiver no Linux e achar /dev/ttyACM0 (padrão Teensy), seleciona
        index = self.combo_ports.findText("/dev/ttyACM0")
        if index >= 0:
            self.combo_ports.setCurrentIndex(index)

    def _toggle_connection(self):
        if self.serial_thread and self.serial_thread.isRunning():
            self._disconnect()
        else:
            port = self.combo_ports.currentText()
            if not port:
                self.txt_messages.appendPlainText("Nenhuma porta serial selecionada!")
                return
            baud = int(self.combo_baud.currentText())
            self._start_worker(port=port, baud=baud, sim=False)

    def _toggle_simulation(self):
        if self.serial_thread and self.serial_thread.isRunning():
            self._disconnect()
        else:
            self._start_worker(sim=True)

    def _start_worker(self, port=None, baud=115200, sim=False):
        self.serial_thread = SerialWorker(port=port, baudrate=baud, simulation=sim)
        self.serial_thread.telemetry_received.connect(self._on_telemetry)
        self.serial_thread.raw_message_received.connect(self._on_raw_message)
        self.serial_thread.connection_status.connect(self._on_connection_status)
        self.serial_thread.start()

        self.btn_connect.setText("Desconectar")
        self.btn_connect.setStyleSheet("background-color: #C62828; color: white;")
        self.combo_ports.setEnabled(False)
        self.combo_baud.setEnabled(False)

    def _disconnect(self):
        if self.serial_thread:
            self.serial_thread.stop()
            self.serial_thread = None
        self.btn_connect.setText("Conectar")
        self.btn_connect.setStyleSheet("background-color: #1565C0; color: white;")
        self.combo_ports.setEnabled(True)
        self.combo_baud.setEnabled(True)
        self.led_running.set_state(False)
        self.led_stopped.set_state(True)
        self.lbl_state.setText("DESCONECTADO")

    def _send_test_start(self):
        """Envia os parâmetros configurados para o Teensy"""
        p_min = self.spin_min.value()
        p_max = self.spin_max.value()
        step = self.spin_step.value()
        
        # Envia comando no formato: START,min,max,step
        cmd = f"START,{p_min},{p_max},{step}"
        
        if self.serial_thread:
            self.serial_thread.send_command(cmd)
            self.txt_messages.appendPlainText(f">> {cmd}")
        else:
            self.txt_messages.appendPlainText("Aviso: Conecte a porta serial antes de iniciar!")


    def _send_tare(self):
        if self.serial_thread:
            self.serial_thread.send_command("TARE")
            self.txt_messages.appendPlainText(">> Comando TARE enviado. Aguarde...")
        else:
            self.txt_messages.appendPlainText("Aviso: Conecte a porta serial primeiro!")

    def _send_reset(self):
        if self.serial_thread:
            self.serial_thread.send_command("RESET")
            self.txt_messages.appendPlainText(">> Comando RESET enviado.")
        else:
            self.txt_messages.appendPlainText("Aviso: Conecte a porta serial primeiro!")


    @pyqtSlot(bool, str)
    def _on_connection_status(self, connected: bool, msg: str):
        self.txt_messages.appendPlainText(f"[SISTEMA]: {msg}")
        if not connected:
            self._disconnect()

    @pyqtSlot(str)
    def _on_raw_message(self, msg: str):
        self.txt_messages.appendPlainText(msg)
        # Auto-scroll para a última mensagem
        self.txt_messages.verticalScrollBar().setValue(
            self.txt_messages.verticalScrollBar().maximum()
        )

    @pyqtSlot(float, float, float, str)
    def _on_telemetry(self, t_sec: float, pulse_us: float, thrust_g: float, state: str):
        # Atualiza labels numéricos
        self.lbl_pulse_val.setText(f"{pulse_us:.0f} µs")
        self.lbl_thrust_val.setText(f"{thrust_g:.2f} g")

        # Atualiza Estado e Leds
        self.lbl_state.setText(state.upper())
        
        if "idle" in state.lower() or "completed" in state.lower() or "desconectado" in state.lower():
            self.led_running.set_state(False)
            self.led_stopped.set_state(True)
        else:
            self.led_running.set_state(True)
            self.led_stopped.set_state(False)

        # Atualiza Buffers e Curvas do PyQtGraph
        self.buf_time.append(t_sec)
        self.buf_pulse.append(pulse_us)
        self.buf_thrust.append(thrust_g)

        self.curve_pulse.setData(list(self.buf_time), list(self.buf_pulse))
        self.curve_thrust.setData(list(self.buf_time), list(self.buf_thrust))

    def closeEvent(self, event):
        if self.serial_thread:
            self.serial_thread.stop()
        event.accept()

# -----------------------------------------------------------------------------
# EXECUÇÃO DA APLICAÇÃO
# -----------------------------------------------------------------------------
if __name__ == "__main__":
    QApplication.setAttribute(Qt.AA_EnableHighDpiScaling, True)
    QApplication.setAttribute(Qt.AA_UseHighDpiPixmaps, True)
    
    app = QApplication(sys.argv)
    window = ThrustTestBenchGUI()
    window.show()
    sys.exit(app.exec_())
