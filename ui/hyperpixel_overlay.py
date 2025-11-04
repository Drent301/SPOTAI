import pygame
import os
import time
import json
import sys

# Voeg de hoofdmap toe voor core-imports
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

# Instellen van de display-driver voor de HyperPixel
os.environ['SDL_FBDEV'] = '/dev/fb1'
os.environ['SDL_VIDEODRIVER'] = 'fbcon'

# Schermgrootte (HyperPixel 2.1 Round is 480x480)
SCREEN_WIDTH = 480
SCREEN_HEIGHT = 480

# Pad naar het UI-statusbestand
UI_STATE_FILE = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'data', 'ui_state.json')

class EyeDisplay:
    """
    Beheert de PyGame-overlay voor de Spot-AI ogen.
    Leest de gewenste status uit ui_state.json (gevuld door emotion_mapper).
    """
    def __init__(self):
        print("[EyeDisplay] Initialiseren...")
        pygame.init()
        pygame.mouse.set_visible(False)
        self.screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT), pygame.NOFRAME)
        self._running = True
        
        # Huidige UI-staat
        self.current_color = (0, 0, 0) # Start met zwart
        self.current_emotion = "booting"
        
        print("[EyeDisplay] PyGame geïnitialiseerd.")

    def stop(self):
        self._running = False

    def load_ui_state(self):
        """Leest de UI-staat uit het JSON-bestand."""
        try:
            with open(UI_STATE_FILE, 'r') as f:
                state = json.load(f)
                self.current_color = tuple(state.get('color', (0, 255, 0)))
                self.current_emotion = state.get('emotion', 'unknown')
        except (FileNotFoundError, json.JSONDecodeError):
            # Gebruik een standaardkleur als het bestand niet bestaat
            self.current_color = (50, 50, 50) # Donkergrijs (geeft fout aan)
            self.current_emotion = "error_loading"
            
    def run_display_loop(self):
        """De hoofd-loop die het scherm ververst."""
        print("[EyeDisplay] Display loop gestart.")
        
        while self._running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self._running = False
                if event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                    self._running = False

            # 1. Lees de gewenste status (van emotion_mapper)
            self.load_ui_state()
            
            # 2. Teken de UI
            self.draw_eyes()

            # 3. Update het scherm
            pygame.display.flip()
            
            time.sleep(0.05) # ~20 FPS

        print("[EyeDisplay] Display loop gestopt.")
        pygame.quit()

    def draw_eyes(self):
        """Tekent de UI op basis van de huidige geladen staat."""
        
        # Maak de achtergrond zwart
        self.screen.fill((0, 0, 0))
        
        # Teken een simpele cirkel met de statuskleur
        pygame.draw.circle(
            self.screen, 
            self.current_color,
            (SCREEN_WIDTH // 2, SCREEN_HEIGHT // 2), # Positie (midden)
            100  # Radius
        )
        
        # TODO: Teken hier de echte "ogen" en pupil-logica

if __name__ == "__main__":
    # Zorg dat de 'data' map bestaat voor de test
    data_dir = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'data')
    os.makedirs(data_dir, exist_ok=True)
    
    try:
        display = EyeDisplay()
        display.run_display_loop()
    except KeyboardInterrupt:
        print("\n[Main] Stop-signaal ontvangen.")