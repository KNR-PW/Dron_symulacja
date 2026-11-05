**Cel główny:** Działasz jako Senior Robotics Software Engineer specjalizujący się w systemach ROS/PX4 oraz autonomii UAV. Twoim zadaniem jest przeprowadzenie rygorystycznego code review obecnego systemu śledzenia (tracking), a następnie zaprojektowanie logiki nowej misji zgodnie z twardymi zasadami zawodów lotniczych.

**Zasady współpracy (Token Optimization & Output Rules):**
1. Bądź maksymalnie zwięzły (No yapping). Pomiń uprzejmości, wstępy i podsumowania.
2. Przed wygenerowaniem dużych bloków kodu, zawsze najpierw przedstaw plan działania (pseudokod lub listę kroków). Poczekaj na moje zatwierdzenie.
3. Modyfikując kod, używaj formatu pokazującego tylko zmienione bloki (git diff), chyba że wyraźnie poproszę o pełny plik.

---

### Faza 1: Code Review i Optymalizacja (Tracking & Gimbal)
Jako plik wejściowy potraktuj `suas_tracking.md`. Znajdują się tam obecne węzły (nodes) oraz pliki startowe (launch) odpowiadające za sterowanie trackingiem i gimbalem podczas podlotu.

**Twoje zadania dla Fazy 1:**
2. Zaproponuj uproszczenia kodu: usuń redundancje, zoptymalizuj pętle sterowania i przepływ danych (np. subskrypcje wizji).
3.  Wskaż ewentualne wąskie gardła i błędy logiki.

Oczekuję krótkiego raportu technicznego (bullet points) z błędami i proponowaną architekturą poprawki. Po zakończeniu Fazy 1 zatrzymaj się i zapytaj mnie, czy zatwierdzam zmiany i czy możemy przejść do Fazy 2.

---

### Faza 2: Implementacja misji "SUAS 3.6 Search, Detect, and Deliver"
Po zatwierdzeniu Fazy 1, zaprojektujesz i zaimplementujesz nową maszynę stanów (State Machine) dla misji zrzutu ładunków, kompatybilną z PX4.

obecnie mam w symulacji tylko namiot trzeba by dodac czlowieka i sprawdzi cjak działa qwykrywanie i pozmieniac plikl launchowy do detekcji 

nastepnie chialbym abys przygotował pełna misje na suas ktora wykona polecenia automatycznie wedlug opisu na stronei i bedzi etrzymala sie regulaminu, jak cos narazie nie da si ezrobi  c mow uproscimuy , jhestemy narazi ena etapie testow , wiecchce przygotowasc wersje testowa i omowic stenariusz testow wq  rzeczywistosci , w symulatorze na tewm moemnnbt moge miec wiecej i pelna misje juz 

przesyłam strone do opisu z suasa 2026 https://robonation.gitbook.io/suas-resources/2026-team-handbook/section-3-mission-demonstration/3.6-search-detect-and-deliver tresc:3.6 Search, Detect, and Deliver

UAS should be able to deliver a payload to a target of interest. As with all other mission elements, the UAS must remain above the 150’ AGL minimum altitude fence while conducting deliveries.

Teams will be given 2 delivery objects at the start of Mission Time:

    GP908 Strobing Beacon w/ 3 AAA batteries installed. This object weighs approximately 155g and will be labeled with an identifier for the team. An STL and pictures of the beacon can be found for download here. 

    8oz plastic water bottle. This object weighs approximately 255g and will be labeled with an identifier for the team. No specific brand of water bottle will be used, teams should design a system capable of adapting to the minor weight/size changes between bottles from different vendors. The general diameter of an 8oz water bottle ranges between 2 - 2.5", with height ranging between 5-6".

UAS must carry all delivery objects they plan to drop at the same time - landing to re-load payloads is NOT permitted. The UAS must fly at least one full waypoint lap prior to performing any deliveries.

The area of interest is defined in Search Boundary. Judges may be in the Search Boundary to score the drops, and the ground may be marked to identify the 50ft drop target radius. The targets may be temporarily occluded while judges evaluate drops from another team and clear any debris.
3.6.1 Delivery Requirements

An attempted delivery is classified by any payload that is released by the UAS in flight, with no dependency on the success of the attempted delivery.

Each independent delivery payload must be no heavier than 2lbs and must not contain any ability to sustain flight (propulsion, propellers, etc.). The delivery payload must land undamaged and must be safe for humans to be present in the drop area. The payload must be safe to retrieve and safe to handle. Payloads that are delivered in freefall, with no form of retardant mechanism, will not be deemed successful. If the UAS were to drop multiple payloads at once, only the best scoring dropped payload will used for scoring

Judges must be able to safely and easily retrieve and separate the delivery object from the delivery payload to verify that it’s undamaged. Separation must not require tools or any instructions. If the judge is unable to separate the delivery object, then the drop will not count.
3.6.2 Target Detection

One mannequin and one tent will be scattered amongst other debris around the Search Boundary. Teams must detect the mannequin to deliver the water bottle, and detect the tent to deliver the strobing beacon. The tent will be in its open state and of the "pop-up" variant. The mannequin may be positioned in any orientation (laying down, sitting up, face-down, etc.) and will be surrounded/covered by surrounding features such as bushes/trees/vehicles/etc.
3.6.3 Search, Detect, and Deliver Scoring

Each of the 2 deliveries will given points as follows - with a maximum of 100 points per delivery:

    Object Survives (Within Vicinity of Search Boundary) = 20 Points

    Object Lands within 50' of an Target = 50 Points

    Object Delivered to the Correct Target (Water Bottle to Mannequin + Beacon to Tent) = 30 Points

Any delivery objects that are delivered without the UAS conducting at least one waypoint lap prior to deliveries will not be counted and thus be given 0 points.

3.7 Penalties

The team will be penalized as follows throughout the mission demonstration. Penalties are defined as a percentage of achievable component points. Unlike points, penalties do not have a bound. This means going over the allowed time can cost the team full points for mission demonstration. If penalties are greater than points, the team will receive a zero for demonstration. Teams cannot score points while generating a penalty.
3.8.1 Excess Time (0.5% Per Second)

The team will receive a penalty equal to 0.5% of mission demonstration points for every second of mission time over limits.
3.8.2 Things Fall Off Aircraft (10% Per Item)

If parts fall off the UAS during flight, teams receive a penalty equal to 10% of demonstration points.
3.8.3 Crash/Collison (50% Per Crash/Collision)

If the UAS crashes during flight or collides with another team's UAS, the team will receive a penalty equal to 50% of demonstration points. If a team is in their runway’s dedicated airspace, then they will not receive a penalty for the collision, and only the offending team will receive a penalty.
3.8.4 Unsafe Operations (50% Per Infraction)

If the UAS or team conducts unsafe flight operations and the team does not respond correctly the commands by the judges (manual takeover, kill switch, etc.), the team will received a penalty equal to 50% of demonstration points. 
3.8.5 Unsafe Out of Bounds (Mission Termination)

Teams are given a flight boundary in the Mission Flight Boundary. Teams will be evaluated by human observers and by judges at the GCS. If the GCS Judge deems the Mission Flight Boundary breach as safety critical, they will command a mission termination and return to launch.
3.8.6 Manual Takeover (Restart Lap)

With exception to takeoff or landing, the aircraft must fly the rest of the mission autonomously. Any transition to manual flight will require the UAS to return to the start of the waypoint lap (at a minimum). If the UAS lands and takes off again, the UAS must fly at least one waypoint lap before attempting other tasks.  

uwagi odemnie , organizatorzy wskazali ze operator bedzi emogl pomoc lekko , wiec mozna zastosowac jakis mechanizm gdzi eja bede cos tez robvił np zatwierdzal  , lub cos klikne jak pojdzi enie tak zeby zaczał odf nowa , ty wymyusl jak prosto i nie komplikowac zrobvi ale zeby mnie wykorzystac 