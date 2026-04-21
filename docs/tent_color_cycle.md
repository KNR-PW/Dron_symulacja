# Tent Color Cycle (Gazebo)

Skrypt `scripts/tent_color_cycle.sh` zmienia kolor namiotu plynnie w petli,
przez RGB/HSV, domyslnie z pelnym cyklem co 60 s.

Dla Twojej konfiguracji (Gazebo w Dockerze) skrypt wykrywa kontener i sam
uruchamia sie w nim.

## Uruchomienie

Najpierw nadaj prawa wykonania:

```bash
chmod +x /home/pawel/Dron_symulacja/scripts/tent_color_cycle.sh
```

Potem uruchom:

```bash
/home/pawel/Dron_symulacja/scripts/tent_color_cycle.sh terrain_tent_sunset 4 60
```

Wersja jawnie z kontenerem:

```bash
/home/pawel/Dron_symulacja/scripts/tent_color_cycle.sh terrain_tent_sunset 4 60 knr_drone_px4
```

Argumenty:

- `world_name` (domyslnie `terrain`)
- `step_seconds` (domyslnie `3`)
- `total_changes` (domyslnie `60`)
- `container_name` (domyslnie `knr_drone_px4`)

Przyklad szybszego i gladszego przejscia:

```bash
/home/pawel/Dron_symulacja/scripts/tent_color_cycle.sh terrain_navy 0.5 100
```

Zatrzymanie: `Ctrl+C`.
