import pygame
import os
import time

# Instellen van de display-driver voor de HyperPixel
# Dit moet gebeuren VOORDAT pygame.init() wordt aangeroepen
os.environ['SDL_FBDEV'] = '/dev/fb1'
os.environ['SDL_VIDEODRIVER'] = 'fbcon'

# Schermgrootte (HyperPixel 2.1 Round is 480x480)
SCREEN_WIDTH = 480
SCREEN_HEIGHT = 480

class EyeDisplay:
    """
    Beheert de PyGame-overlay voor de Spot-AI ogen.
    Dit script draait als een aparte service (de 'eyes-legacy-eyes.service').
    """
    def __init__(self):
        print("[EyeDisplay] Initialiseren...")
        pygame.init()
        
        # Zet de muiscursor uit
        pygame.mouse.set_visible(False)
        
        # Maak het hoofdscherm (fullscreen)
        self.screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT), pygame.NOFRAME)
        self._running = True
        print("[EyeDisplay] PyGame geïnitialiseerd.")

    def stop(self):
        """Stopt de main loop."""
        self._running = False

    def run_display_loop(self):
        """De hoofd-loop die het scherm ververst."""
        print("[EyeDisplay] Display loop gestart.")
        
        while self._running:
            # 1. Check voor events (bv. touch-input of stop-signaal)
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self._running = False
                if event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                    self._running = False

            # 2. Logica om status te lezen (van StateBus of /tmp/ bestanden)
            # ... (Nog te implementeren) ...
            
            # 3. Teken de UI
            self.draw_eyes()

            # 4. Update het scherm
            pygame.display.flip()
            
            # Wacht kort om de CPU niet te overbelasten
            time.sleep(0.05) # ~20 FPS

        print("[EyeDisplay] Display loop gestopt.")
        pygame.quit()

    def draw_eyes(self):
        """Tijdelijke placeholder om te tonen dat het werkt."""
        
        # Maak de achtergrond zwart
        self.screen.fill((0, 0, 0))
        
        # Teken een simpele witte cirkel als test
        pygame.draw.circle(
            self.screen, 
            (255, 255, 255),  # Kleur (wit)
            (SCREEN_WIDTH // 2, SCREEN_HEIGHT // 2), # Positie (midden)
            100  # Radius
        )

if __name__ == "__main__":
    try:
        display = EyeDisplay()
        display.run_display_loop()
    except KeyboardInterrupt:
        print("\n[Main] Stop-signaal ontvangen.")
        # Opmerking: de 'display' instance moet mogelijk een stop() methode hebben
        # die van buitenaf kan worden aangeroepen, maar voor nu is dit voldoende.