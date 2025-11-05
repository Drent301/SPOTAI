import pygame
import os
import time
import json
import sys
import math

# Voeg de hoofdmap toe voor core-imports
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from core.statebus import StateBus # <-- GEWIJZIGD

# Instellen van de display-driver voor de HyperPixel
os.environ['SDL_FBDEV'] = '/dev/fb1'
os.environ['SDL_VIDEODRIVER'] = 'fbcon'

# Schermgrootte (HyperPixel 2.1 Round is 480x480)
SCREEN_WIDTH = 480
SCREEN_HEIGHT = 480
CENTER_X = SCREEN_WIDTH // 2
CENTER_Y = SCREEN_HEIGHT // 2

# Oog-instellingen
EYE_RADIUS = 80
PUPIL_RADIUS = 30
EYE_OFFSET_X = 120 # Afstand van het midden
EYE_Y = CENTER_Y - 50

class EyeDisplay:
    """
    Beheert de PyGame-overlay voor de Spot-AI ogen.
    Leest de gewenste status nu direct uit de StateBus.
    """
    def __init__(self):
        print("[EyeDisplay] Initialiseren...")
        self.bus = StateBus() # <-- GEWIJZIGD
        pygame.init()
        pygame.mouse.set_visible(False)
        self.screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT), pygame.NOFRAME)
        self._running = True
        
        # Huidige UI-staat
        self.current_color = (0, 0, 0) # Start met zwart
        self.current_emotion = "booting"
        self.blink_timer = 0
        
        print("[EyeDisplay] PyGame geïnitialiseerd.")

    def stop(self):
        self._running = False

    def load_ui_state(self):
        """Leest de UI-staat nu uit de StateBus."""
        state = self.bus.get_value("ui_state")
        if state:
            self.current_color = tuple(state.get('color', (0, 255, 0)))
            self.current_emotion = state.get('emotion', 'unknown')
        else:
            # Fallback als de statebus leeg is
            self.current_color = (50, 50, 50)
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

            # 1. Lees de gewenste status (van emotion_mapper via StateBus)
            self.load_ui_state()
            
            # 2. Teken de UI
            self.draw_eyes()

            # 3. Update het scherm
            pygame.display.flip()
            
            # Update timers (voor animaties zoals knipperen)
            self.blink_timer = (self.blink_timer + 1) % 100 # Simpele knipper-cyclus
            
            time.sleep(0.05) # ~20 FPS

        print("[EyeDisplay] Display loop gestopt.")
        pygame.quit()

    def _draw_idle_eyes(self, color):
        """ Tekent standaard, open ogen. """
        # Linker oog
        pygame.draw.ellipse(self.screen, color, (CENTER_X - EYE_OFFSET_X - EYE_RADIUS, EYE_Y - EYE_RADIUS, EYE_RADIUS * 2, EYE_RADIUS * 2), 4)
        # Linker pupil
        pygame.draw.ellipse(self.screen, color, (CENTER_X - EYE_OFFSET_X - PUPIL_RADIUS, EYE_Y - PUPIL_RADIUS, PUPIL_RADIUS * 2, PUPIL_RADIUS * 2))

        # Rechter oog
        pygame.draw.ellipse(self.screen, color, (CENTER_X + EYE_OFFSET_X - EYE_RADIUS, EYE_Y - EYE_RADIUS, EYE_RADIUS * 2, EYE_RADIUS * 2), 4)
        # Rechter pupil
        pygame.draw.ellipse(self.screen, color, (CENTER_X + EYE_OFFSET_X - PUPIL_RADIUS, EYE_Y - PUPIL_RADIUS, PUPIL_RADIUS * 2, PUPIL_RADIUS * 2))

        # Knipper-animatie
        if self.blink_timer > 90:
            pygame.draw.rect(self.screen, (0,0,0), (0, EYE_Y - EYE_RADIUS, SCREEN_WIDTH, EYE_RADIUS * 2)) # Bedek de ogen
            pygame.draw.line(self.screen, color, (CENTER_X - EYE_OFFSET_X - EYE_RADIUS, EYE_Y), (CENTER_X - EYE_OFFSET_X + EYE_RADIUS, EYE_Y), 4)
            pygame.draw.line(self.screen, color, (CENTER_X + EYE_OFFSET_X - EYE_RADIUS, EYE_Y), (CENTER_X + EYE_OFFSET_X + EYE_RADIUS, EYE_Y), 4)

    def _draw_offline_eyes(self, color):
        """ Tekent 'slaperige' of offline ogen. """
        # Linker oog (half dicht)
        pygame.draw.line(self.screen, color, (CENTER_X - EYE_OFFSET_X - EYE_RADIUS, EYE_Y), (CENTER_X - EYE_OFFSET_X + EYE_RADIUS, EYE_Y), 4)
        # Rechter oog (half dicht)
        pygame.draw.line(self.screen, color, (CENTER_X + EYE_OFFSET_X - EYE_RADIUS, EYE_Y), (CENTER_X + EYE_OFFSET_X + EYE_RADIUS, EYE_Y), 4)

    def _draw_error_eyes(self, color):
        """ Tekent 'X' ogen voor een foutstatus. """
        # Linker X
        pygame.draw.line(self.screen, color, (CENTER_X - EYE_OFFSET_X - 40, EYE_Y - 40), (CENTER_X - EYE_OFFSET_X + 40, EYE_Y + 40), 6)
        pygame.draw.line(self.screen, color, (CENTER_X - EYE_OFFSET_X - 40, EYE_Y + 40), (CENTER_X - EYE_OFFSET_X + 40, EYE_Y - 40), 6)
        
        # Rechter X
        pygame.draw.line(self.screen, color, (CENTER_X + EYE_OFFSET_X - 40, EYE_Y - 40), (CENTER_X + EYE_OFFSET_X + 40, EYE_Y + 40), 6)
        pygame.draw.line(self.screen, color, (CENTER_X + EYE_OFFSET_X - 40, EYE_Y + 40), (CENTER_X + EYE_OFFSET_X + 40, EYE_Y - 40), 6)


    def draw_eyes(self):
        """
        Hoofd-tekenfunctie: Kiest de juiste oog-stijl op basis van de emotie.
        """
        
        # Maak de achtergrond zwart
        self.screen.fill((0, 0, 0))
        
        color = self.current_color
        
        if self.current_emotion == "idle":
            self._draw_idle_eyes(color)
        elif self.current_emotion == "offline":
            self._draw_offline_eyes(color)
        elif self.current_emotion == "error" or self.current_emotion == "error_loading":
            self._draw_error_eyes(color)
        else:
            # Fallback voor onbekende emoties
            self._draw_idle_eyes(color)


def main():
    # De __main__ block is nu simpeler, geen noodzaak om voor bestanden te zorgen
    display = EyeDisplay()
    try:
        display.run_display_loop()
    except KeyboardInterrupt:
        print("\n[Main] Stop-signaal ontvangen.")
        display.stop()

if __name__ == "__main__":
    main()
