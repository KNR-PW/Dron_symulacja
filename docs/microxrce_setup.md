# Dokument opisuje kroki do testowania agenta microXRCE z hardware'm

## Połączenie PX4 - Raspi

1. Łączymy interfejs UART z portu TELEM2 z RX1 TX1 GND na raspberry. 

2. Łączymy się z Orange Cube poprzez QGC i tam `ustawiamy parametr UXRCE_DDS_CFG` na `TELEM2`.

3. Wykonujemy komendę do połączenia:
```
sudo MicroXRCEAgent serial --dev /dev/ttyAMA0 -b 921600
```
należy zwrócić uwagę, żeby `baudrate na porcie TELEM2 był zgodny` (sprawdzić parametr w QGC) oraz czy `raspberry pojawiła się na porcie ttyAMA0` (ewentualnie odpowiednio zamienić ten fragment).

Jeżeli w konsoli pojawią się na poczatku takie wpisy:
```
_key: 0x00000001, session_id: 0x81
[1766003594.190530] info     | SessionManager.hpp | establish_session        | session established    | client_key: 0x00000001, address: 1
[1766003594.253115] info     | ProxyClient.cpp    | create_participant       | participant created    | client
```
oznacza to, że poprawnie skonfigurowano połaczenie i można przystąpić do uruchamiania modułów ROSa.

Przy budowaniu kodu może zdarzyć się, że agent microxrce nie pozwala na ukończenie builda:
```
Finished <<< drone_gui [5.19s]                                                                       
Starting >>> microxrcedds_agent
--- stderr: microxrcedds_agent                                                    
gmake[2]: *** No rule to make target '/opt/ros/jazzy/lib/libfastrtps.so.2.14.4', needed by 'libmicroxrcedds_agent.so.2.4.3'.  Stop.
gmake[1]: *** [CMakeFiles/Makefile2:85: CMakeFiles/microxrcedds_agent.dir/all] Error 2
gmake: *** [Makefile:136: all] Error 2
---
Failed   <<< microxrcedds_agent [0.27s, exited with code 2]
```

Trzeba wtedy skipnąć ten moduł przy budowaniu:
```
colcon build --packages-skip microxrcedds_agent
```
 i powinno być babag - klasycznie 
 ```
 source install/setup.bash
 ```