#!/usr/bin/env python3
"""Wspolna geometria kamery: piksel -> punkt na ziemi.

Uzywaja tego DWA wezly i to celowo ten sam kod:
  * suas_geolocator — zamienia detekcje na wspolrzedne GPS,
  * suas_flight_controller — liczy, gdzie wzgledem drona lezy cel, zeby
    sterowac na METRY, a nie na ulamki kadru.

Dlaczego wspolny modul, a nie kopia w kazdym wezle: te wzory sa zweryfikowane
liczbowo (dla roll=pitch=0 redukuja sie dokladnie do prostego "piksel * GSD",
a przy przechylach daja zero bledu wzgledem prawdy z Gazebo). Druga kopia
predzej czy pozniej rozjechalaby sie z pierwsza.
"""

import math


def _rot_body_to_ned(roll, pitch, yaw):
    """Macierz obrotu FRD (przod-prawo-dol) -> NED, konwencja Z-Y-X.

    R = Rz(yaw) * Ry(pitch) * Rx(roll). Dla roll=pitch=0 redukuje sie do
    samego obrotu o kurs, czyli do tego, co robil poprzedni wzor.
    """
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp,     cp * sr,                cp * cr),
    )


def _project_pixel(u, v, alt, roll, pitch, yaw,
                   mount_pitch_deg, cam_yaw_offset, focal_px, cx, cy,
                   stabilized=False):
    """Piksel (u, v) -> punkt na ziemi wzgledem drona.

    Zwraca (d_north, d_east, slant) w metrach albo None, gdy promien nie trafia
    w ziemie (patrzy w horyzont albo w gore).

    Osie kamery w ukladzie FRD drona (x=przod, y=prawo, z=dol), przy kacie
    montazu `mount_pitch_deg` (-90 = prosto w dol):

        phi = -mount_pitch_deg          0 = poziomo w przod, 90 = prosto w dol
        boresight     b = ( cos phi, 0,  sin phi)
        gora kadru   up = ( sin phi, 0, -cos phi)
        prawo kadru  rt = ( 0,       1,   0     )

    Sprawdzenie: dla phi=90 wychodzi b=(0,0,1) czyli w dol, a gora kadru
    (1,0,0) czyli nos drona - zgodnie z konwencja "gora kadru = przod drona".

    DLACZEGO TO JEST WAZNE: gimbal nie jest stabilizowany, wiec na prostym
    galsie z predkoscia 8 m/s kopter trzyma staly pitch ok. 10 stopni i kamera
    patrzy 10 stopni obok nadiru. Na 80 m to 14 m bledu - wiecej niz cala
    reszta budzetu dokladnosci razem wzieta.
    """
    phi = math.radians(-mount_pitch_deg)
    cph, sph = math.cos(phi), math.sin(phi)
    b = (cph, 0.0, sph)
    up = (sph, 0.0, -cph)
    rt = (0.0, 1.0, 0.0)

    # Obrot kadru wzgledem nosa drona (jak kamera jest wkrecona w uchwyt).
    if cam_yaw_offset:
        ca, sa = math.cos(cam_yaw_offset), math.sin(cam_yaw_offset)
        def _rz(w):
            return (ca * w[0] - sa * w[1], sa * w[0] + ca * w[1], w[2])
        b, up, rt = _rz(b), _rz(up), _rz(rt)

    du = u - cx
    dv = cy - v                      # v rosnie w DOL obrazu
    d_body = tuple(focal_px * b[i] + du * rt[i] + dv * up[i] for i in range(3))

    # Stabilizowany mount sam zdejmuje roll/pitch - wtedy zostaje sam kurs,
    # bo gimbal jest jednoosiowy i kadr i tak krazy razem z rama.
    R = (_rot_body_to_ned(0.0, 0.0, yaw) if stabilized
         else _rot_body_to_ned(roll, pitch, yaw))
    dn, de, dd = (sum(R[i][j] * d_body[j] for j in range(3)) for i in range(3))

    if dd <= 1e-6:
        return None                  # promien nie idzie w dol

    scale = alt / dd
    slant = scale * math.sqrt(dn * dn + de * de + dd * dd)
    return scale * dn, scale * de, slant
