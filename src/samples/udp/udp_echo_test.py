import socket
import sys

# --- KONFIGURACIJA ---
# IP adresa vašeg RAČUNARA na kojoj Python sluša.
# Mora odgovarati pc_ip u C kodu.
UDP_LISTEN_IP = "192.168.1.101" 
UDP_LISTEN_PORT = 1002

# Kreiraj UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

try:
    # Veži socket za lokalnu IP adresu i port
    sock.bind((UDP_LISTEN_IP, UDP_LISTEN_PORT))
except OSError as e:
    # Ako se ne može vezati, ispisuje se greška i izlazi se
    print(f"GREŠKA pri vezivanju za socket: {e}", file=sys.stderr)
    print("Proverite da li je IP adresa tačna za vaš PC i da port 1002 nije već zauzet.", file=sys.stderr)
    sys.exit(1)

# Nema ispisivanja početka slušanja, da bi bilo potpuno tiho
# Ako želite da ipak vidite da je server pokrenut, odkomentarišite sledeći red:
# print(f"Python ECHO server sluša tiho na {UDP_LISTEN_IP}:{UDP_LISTEN_PORT}...")

while True:
    try:
        # 1. Čekaj i primi podatke (data, addr su adrese POŠILJAOCA)
        # Nema .decode(), radimo sa sirovim (raw) bajtovima za najbolju brzinu.
        data, addr = sock.recvfrom(1024) 
        
        # 2. Pošalji isti paket nazad (ECHO)
        # Koristimo 'data' (sirovi bajtovi) i 'addr' (adresa pošiljaoca)
        sock.sendto(data, addr)
        
        # Svi print() pozivi su uklonjeni
        
    except KeyboardInterrupt:
        # Omogućava izlazak pritiskom na CTRL+C
        break
        
    except Exception as e:
        # I najmanja greška se ispisuje na standardni izlaz za greške (stderr)
        # Ali se ne prekida petlja, osim ako nije kritična
        # print(f"Došlo je do greške: {e}", file=sys.stderr)
        pass # Ignorišemo manje greške da bi server bio tih i brz