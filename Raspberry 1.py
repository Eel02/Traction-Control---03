import evdev
import struct
import serial
import time
import threading
from evdev import ecodes

# 1. PARAMETRI CINEMATICI E DI SISTEMA
V_MAX = 0.6  # [m/s] Massima velocità lineare consentita
OMEGA_MAX = 2.0*V_MAX / 0.40 # [rad/s] Massima velocità angolare consentita
DEADBAND = 2000  # Soglia di reiezione del rumore per gli stick analogici

SEND_RATE_HZ = 50.0  # Frequenza di campionamento del loop di trasmissione
SEND_INTERVAL = 1.0 / SEND_RATE_HZ  # Delta T (0.01s)

# 2. VARIABILI CONDIVISE (STATE SPACE)
state_lock = threading.Lock()
v_target = 0.0
omega_target = 0.0
brake_active = False  # Flag di stato per il freno di emergenza


# 3. THREAD: ACQUISIZIONE INPUT
def gamepad_worker(device_path):
    global v_target, omega_target, brake_active

    try:
        gamepad = evdev.InputDevice(device_path)
        print(f"[INIT] Dispositivo acquisito correttamente: {gamepad.name}")
    except FileNotFoundError:
        print(f"[ERRORE FATALE] Dispositivo non trovato al percorso: {device_path}")
        return

    # Inizializzazione dello stato degli stick
    val_y = 0
    val_rx = 0

    try:
        # Loop asincrono di ascolto degli eventi hardware
        for event in gamepad.read_loop():

            # Gestione Pulsanti (Freno di Emergenza)
            if event.type == ecodes.EV_KEY:
                if event.code in [ecodes.BTN_B, ecodes.BTN_EAST] and event.value == 1:
                    with state_lock:
                        brake_active = not brake_active
                        if brake_active:
                            print("\n[INTERRUPT] FRENO DI EMERGENZA ATTIVATO - Motori bloccati.")
                        else:
                            print("\n[INTERRUPT] FRENO DI EMERGENZA RILASCIATO - Ripristino controllo cinematico.")

            # Gestione Assi Analogici
            elif event.type == ecodes.EV_ABS:
                # Stick Sinistro (Asse Y) -> Target Velocità
                if event.code == ecodes.ABS_Y:
                    val_y = event.value if abs(event.value) > DEADBAND else 0
                # Stick Destro (Asse X) -> Target Omega
                elif event.code == ecodes.ABS_RX:
                    val_rx = event.value if abs(event.value) > DEADBAND else 0

            # Equazioni cinematiche
            calc_v = (-val_y / 32768.0) * V_MAX
            calc_omega = (val_rx / 32768.0) * OMEGA_MAX

            # Aggiornamento protetto delle variabili di stato globali
            with state_lock:
                if brake_active:
                    # Overriding di sicurezza: la cinematica calcolata viene ignorata
                    v_target = 0.0
                    omega_target = 0.0
                else:
                    v_target = calc_v
                    omega_target = calc_omega

    except OSError:
        print("[AVVISO] Connessione hardware al gamepad interrotta.")

# 4. MAIN LOOP: TRASMISSIONE UART
def main():
    # Inizializzazione Hardware UART
    try:
        uart = serial.Serial('/dev/ttyACM0', baudrate=115200, timeout=0.1)
        print("[INIT] Interfaccia UART (ttyACM0) inizializzata a 115200 bps.")
    except serial.SerialException as e:
        print(f"[ERRORE FATALE] Impossibile aprire la porta seriale: {e}")
        return

    # Thread asincrono per la lettura del gamepad.
    gamepad_path = '/dev/input/by-id/usb-Logitech_Wireless_Gamepad_F710_51DB9EFC-event-joystick'
    t_gamepad = threading.Thread(target=gamepad_worker, args=(gamepad_path,), daemon=True)
    t_gamepad.start()

    print(f"[INFO] Loop di controllo real-time in esecuzione a {SEND_RATE_HZ} Hz.")
    print(f"[INFO] Premere il TASTO B per attivare/disattivare il freno di emergenza.")
    print("-" * 70)

    loop_counter = 0

    try:
        while True:
            # Acquisizione del timestamp per garantire temporizzazione corretta
            t_start = time.perf_counter()

            # Lettura protetta dei riferimenti spaziali
            with state_lock:
                current_v = v_target
                current_omega = omega_target
                current_brake = brake_active

            # ASSEMBLAGGIO (Protocollo verso STM)
            # Formato float standard IEEE 754 a 32 bit, Little-Endian ('<ff')
            payload = struct.pack('<ff', current_v, current_omega)

            # Calcolo del checksum (XOR logico sugli 8 byte del payload)
            checksum = 0
            for byte in payload:
                checksum ^= byte

            # Unione dei segmenti: Header (0xAA, 0x55) + Dati + Checksum
            packet = bytes([0xAA, 0x55]) + payload + bytes([checksum])

            # Invio fisico sul bus UART verso il microcontrollore STM
            uart.write(packet)
            uart.flush()

            # DEBUGGING A TERMINALE
            # Stampa a schermo mitigata per non saturare il buffer I/O (10 Hz)
            if loop_counter % 5 == 0:
                brake_status = " [BRAKE ON]" if current_brake else ""
                hex_str = ' '.join(f'{b:02X}' for b in packet)
                print(
                    f"[UART TX] V: {current_v:>+5.2f} m/s | Omega: {current_omega:>+5.2f} rad/s{brake_status} | RAW: [{hex_str}]")

            loop_counter += 1

            # REGOLAZIONE DELLA FREQUENZA
            t_elapsed = time.perf_counter() - t_start
            t_sleep = SEND_INTERVAL - t_elapsed

            # Compensazione dei ritardi di scheduling del sistema operativo
            if t_sleep > 0:
                time.sleep(t_sleep)

    except KeyboardInterrupt:
        print("\n[INFO] Ricevuto segnale di interruzione. Arresto del sistema.")
    finally:
        if uart and uart.is_open:
            # Comando finale (safe state) inviato per disarmare i driver prima della chiusura
            safe_payload = struct.pack('<ff', 0.0, 0.0)
            safe_checksum = 0
            for byte in safe_payload: safe_checksum ^= byte
            uart.write(bytes([0xAA, 0x55]) + safe_payload + bytes([safe_checksum]))
            uart.flush()
            uart.close()
            print("[INFO] Porta seriale chiusa e attuatori disarmati in sicurezza.")


if __name__ == '__main__':
    main()