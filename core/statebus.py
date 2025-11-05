import redis
import json
import time

# De naam van de Redis Hash waarin de volledige state wordt opgeslagen.
STATE_HASH_NAME = "spotai:statebus"

class StateBus:
    def __init__(self, host='localhost', port=6379, db=0):
        """Initialiseert de verbinding met de Redis-server."""
        try:
            self.redis_client = redis.Redis(host=host, port=port, db=db, decode_responses=True)
            self.redis_client.ping()  # Controleer de verbinding
            print("StateBus: Succesvol verbonden met Redis.")
        except redis.exceptions.ConnectionError as e:
            print(f"StateBus: Fout bij het verbinden met Redis: {e}")
            print("Zorg ervoor dat de Redis-server draait op redis://{host}:{port}")
            # In een echte implementatie zou hier een fallback of exit kunnen zijn.
            self.redis_client = None

    def _is_connected(self):
        """Controleert of de verbinding met Redis actief is."""
        if self.redis_client is None:
            print("StateBus: Geen actieve Redis-verbinding.")
            return False
        return True

    def set_value(self, key, value):
        """
        Werkt een waarde bij in de Redis Hash en voegt een timestamp toe.
        Waarden worden als JSON-string opgeslagen voor consistentie.
        """
        if not self._is_connected():
            return

        try:
            # Serialiseer de waarde naar een JSON-string
            serialized_value = json.dumps(value)

            # Gebruik een pipeline voor atomische operaties
            pipe = self.redis_client.pipeline()
            pipe.hset(STATE_HASH_NAME, key, serialized_value)
            pipe.hset(STATE_HASH_NAME, 'last_update', time.time())
            pipe.execute()

            print(f"StateBus: {key} ingesteld.")
        except Exception as e:
            print(f"StateBus: Fout bij instellen van '{key}': {e}")

    def get_value(self, key, default=None):
        """Haalt een waarde op uit de Redis Hash en deserialiseert deze."""
        if not self._is_connected():
            return default

        try:
            serialized_value = self.redis_client.hget(STATE_HASH_NAME, key)
            if serialized_value is None:
                return default

            # Deserialiseer de waarde uit de JSON-string
            return json.loads(serialized_value)
        except Exception as e:
            print(f"StateBus: Fout bij ophalen van '{key}': {e}")
            return default

    def get_all_values(self):
        """Haalt de volledige state hash op uit Redis."""
        if not self._is_connected():
            return None

        try:
            full_hash = self.redis_client.hgetall(STATE_HASH_NAME)
            # Deserialiseer elke waarde in de dictionary
            deserialized_hash = {
                key: json.loads(value)
                for key, value in full_hash.items()
            }
            return deserialized_hash
        except Exception as e:
            print(f"StateBus: Fout bij ophalen van de volledige state: {e}")
            return None


if __name__ == "__main__":
    print("StateBus (Redis) module test...")
    bus = StateBus()

    if bus._is_connected():
        # Testgegevens
        test_dict = {"x": 10, "y": 25.5}
        test_list = ["start", "processing", "end"]

        bus.set_value("robot_mode", "idle")
        bus.set_value("battery_level", 98.5)
        bus.set_value("motor_coordinates", test_dict)
        bus.set_value("task_queue", test_list)

        print(f"Huidige robot_mode: {bus.get_value('robot_mode')}")
        print(f"Batterijniveau: {bus.get_value('battery_level')}")
        print(f"Motorcoördinaten: {bus.get_value('motor_coordinates')}")
        print(f"Wachtrij: {bus.get_value('task_queue')}")

        print("\nVolledige state bus:")
        print(bus.get_all_values())
