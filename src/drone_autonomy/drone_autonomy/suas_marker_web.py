#!/usr/bin/env python3
"""
suas_marker_web — lekkie GUI do recznego wskazywania celow.

Operator oglada obraz z kamery w przegladarce i klika w obiekt, ktorego automat
nie znajdzie. Na 80 m czlowiek ma w kadrze ok. 11 px, czyli ponizej progu YOLO —
tam czlowiek przy ekranie widzi wiecej niz model.

    LPM gdziekolwiek   ->  obraz ZAMARZA (aktualna klatka, pelna jakosc)
    LPM na obiekcie    ->  male menu w miejscu kliku:  NAMIOT | CZLOWIEK
    LPM w klase        ->  zapisane, podglad WRACA na zywo sam
    PPM albo Esc       ->  anuluj, wroc na zywo

Trzy klikniecia, zero pisania. Pomylke poprawiasz klikajac jeszcze raz — nowy
znacznik tej samej klasy nadpisuje poprzedni (geolokator daje znacznikom
operatora pierwszenstwo nad automatem).

DLACZEGO NAJPIERW ZAMROZENIE, POTEM KLIK:
Klikajac w obraz na zywo trafialbys w kadr sprzed sekundy, a serwer dostalby
najnowsza klatke — piksel wskazywalby co innego. Zamrozona klatka niesie wlasny
header.stamp, wiec geolokator rzutuje ten piksel telemetria z DOKLADNIE tej
chwili. Opoznienie podgladu przestaje byc zrodlem bledu.

KOSZT W SPOCZYNKU — to jest glowne kryterium projektowe tego wezla:
  * bez otwartej strony wezel NIE SUBSKRYBUJE NICZEGO i nic nie liczy,
  * podglad na zywo to przekazywanie gotowych bajtow JPEG z
    /tent_detections/image/compressed — bez dekodowania i bez cv_bridge.
    Subskrypcja powstaje przy pierwszym kliencie i znika przy ostatnim,
    a detektor publikuje podglad tylko przy aktywnej subskrypcji, wiec przy
    zamknietej przegladarce caly lancuch jest wylaczony,
  * ZAMROZ KLATKE oddaje ostatni gotowy JPEG z rolling-cache kamery (bajty tak,
    jak przyszly) — bez dekodowania, bez re-encode, bez zakladania subskrypcji
    na klik. Cache zyje z tego samego pulsu widzow co podglad. Nic sie nie kreci
    w tle, a zamrozenie jest natychmiastowe.

Zamiast Flaska idzie http.server ze standardowej biblioteki — cztery endpointy
nie potrzebuja frameworka, a zero zaleznosci znaczy, ze to samo dziala
w kontenerze symulacji i na Jetsonie bez instalowania czegokolwiek.

Uruchomienie:
    ros2 run drone_autonomy suas_marker_web
    -> http://<ip>:5000
"""

import json
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool

from drone_interfaces.msg import OperatorMark

CLASS_NAMES = {0: 'namiot', 1: 'czlowiek'}

PAGE = """<!doctype html>
<meta charset="utf-8">
<title>SUAS — oznaczanie celow</title>
<style>
  html,body{margin:0;height:100%;background:#111;color:#ddd;
            font:14px system-ui,sans-serif;overflow:hidden}
  #wrap{position:relative;height:calc(100% - 34px);display:flex;
        align-items:center;justify-content:center}
  img{max-width:100%;max-height:100%;display:block}
  #frozen{display:none}
  #bar{height:34px;display:flex;align-items:center;gap:18px;padding:0 12px;
       background:#000;border-top:1px solid #333;white-space:nowrap}
  #mode{font-weight:700}
  .live{color:#4ade80} .froz{color:#fbbf24}
  #menu{position:absolute;display:none;background:#1b1b1b;border:1px solid #555;
        border-radius:6px;overflow:hidden;box-shadow:0 4px 16px #000a}
  #menu button{display:block;width:100%;padding:10px 20px;border:0;
        background:none;color:#eee;font:inherit;text-align:left;cursor:pointer}
  #menu button:hover{background:#333}
  .dot{position:absolute;width:12px;height:12px;margin:-6px 0 0 -6px;
       border-radius:50%;border:2px solid #fff;pointer-events:none}
</style>

<div id="wrap">
  <img id="live" src="/stream" alt="podglad na zywo">
  <img id="frozen">
  <div id="menu">
    <button data-c="0">NAMIOT</button>
    <button data-c="1">CZLOWIEK</button>
  </div>
</div>
<div id="bar">
  <span id="mode" class="live">NA ZYWO</span>
  <span id="hint">LPM = zamroz klatke</span>
  <span id="status">—</span>
</div>

<script>
const live=document.getElementById('live'), frozen=document.getElementById('frozen'),
      menu=document.getElementById('menu'), wrap=document.getElementById('wrap'),
      mode=document.getElementById('mode'), hint=document.getElementById('hint'),
      statusEl=document.getElementById('status');
// Stan trzymamy w zmiennej, a nie zgadujemy po style.display: element ma regule
// display:none w arkuszu, wiec inline '' by go NIE pokazal, tylko przywrocil
// regule. Stad tez wszedzie jawne 'block'.
let frozenMode=false, stamp=null, pick=null, dot=null, url=null;

function toLive(){
  frozen.style.display='none'; live.style.display='block';
  menu.style.display='none'; if(dot){dot.remove(); dot=null;}
  if(url){URL.revokeObjectURL(url); url=null;}
  frozenMode=false; stamp=null; pick=null;
  mode.textContent='NA ZYWO'; mode.className='live';
  hint.textContent='LPM = zamroz klatke';
}

async function freeze(){
  hint.textContent='lapie klatke...';
  let r;
  try{ r=await fetch('/grab'); }
  catch(e){ hint.textContent='brak polaczenia'; return; }
  if(!r.ok){ hint.textContent='blad: '+await r.text(); return; }
  stamp={sec:+r.headers.get('X-Stamp-Sec'), nanosec:+r.headers.get('X-Stamp-Nsec')};
  if(url) URL.revokeObjectURL(url);
  url=URL.createObjectURL(await r.blob());
  frozen.src=url;
  live.style.display='none'; frozen.style.display='block';
  frozenMode=true;
  mode.textContent='ZAMROZONA'; mode.className='froz';
  hint.textContent='LPM na obiekcie = wybierz klase   |   PPM albo Esc = anuluj';
}

// Piksel w ORYGINALNEJ klatce: obraz jest skalowany do okna, wiec klik
// trzeba przeliczyc przez naturalWidth/szerokosc wyswietlana.
function pixel(ev){
  const b=frozen.getBoundingClientRect();
  if(!b.width || !frozen.naturalWidth) return null;   // klatka jeszcze nie gotowa
  return {u:(ev.clientX-b.left)*frozen.naturalWidth/b.width,
          v:(ev.clientY-b.top)*frozen.naturalHeight/b.height,
          x:ev.clientX, y:ev.clientY};
}

// LPM prowadzi caly przeplyw: zamroz -> wskaz obiekt -> wybierz klase.
// PPM zostaje jako anuluj (i tak trzeba mu zablokowac menu przegladarki).
wrap.addEventListener('contextmenu', ev=>{ ev.preventDefault(); toLive(); });

wrap.addEventListener('click', ev=>{
  if(menu.style.display==='block'){        // menu otwarte -> klik obok je zamyka
    menu.style.display='none';
    if(dot){dot.remove(); dot=null;}
    return;
  }
  if(!frozenMode){ freeze(); return; }
  const q=pixel(ev);
  if(!q){ hint.textContent='klatka jeszcze sie laduje, sprobuj ponownie'; return; }
  pick=q;
  const w=wrap.getBoundingClientRect();
  menu.style.left=(ev.clientX-w.left)+'px';
  menu.style.top=(ev.clientY-w.top)+'px';
  menu.style.display='block';
  if(dot) dot.remove();
  dot=document.createElement('div'); dot.className='dot';
  dot.style.left=(ev.clientX-w.left)+'px'; dot.style.top=(ev.clientY-w.top)+'px';
  wrap.appendChild(dot);
});

menu.addEventListener('click', async ev=>{
  ev.stopPropagation();                    // zeby handler wrap nie zamknal menu
  const c=ev.target.dataset.c; if(c===undefined||!pick) return;
  const r=await fetch('/mark',{method:'POST',body:JSON.stringify(
      {u:pick.u, v:pick.v, class_id:+c, stamp:stamp})});
  statusEl.textContent=await r.text();
  toLive();
});

document.addEventListener('keydown', ev=>{ if(ev.key==='Escape') toLive(); });

fetch('/status').then(r=>r.text()).then(t=>statusEl.textContent=t);

// Puls obecnosci operatora. Misja pyta o potwierdzenie TYLKO wtedy, gdy ktos
// patrzy — a jedyny pewny dowod na to musi przyjsc OD PRZEGLADARKI.
// Liczenie widzow po stronie serwera nie wystarcza: gdy padnie Tailscale,
// przegladarka nie zamyka polaczenia, a TCP bez keepalive zauwaza martwy drugi
// koniec dopiero po minutach. Puls znika w 2 s.
setInterval(()=>{ fetch('/ping').catch(()=>{}); }, 2000);
fetch('/ping').catch(()=>{});
</script>
"""


class MarkerNode(Node):
    def __init__(self):
        super().__init__('suas_marker_web')

        # Klatka do ZAMRAZANIA i klikania. Preview OAK 1024x1024 (skompresowane) —
        # DOKLADNIE ta rozdzielczosc, w ktorej geolokator liczy rzut piksela
        # (suas_geolocator img_w=1024, cx=512), wiec u,v z klikniecia ida 1:1 bez
        # skalowania. NIE bierzemy tu pelnej klatki /oak/rgb/image_raw (2028x1520):
        # geolokator zrzutowalby jej piksele wg modelu 1024 i znacznik operatora
        # trafilby w zle miejsce. Preview detektora (preview_topic) tez nie: jest
        # przeskalowany do 960 px i ma narysowane boxy.
        self.declare_parameter('camera_topic', '/oak/rgb/preview/image_raw/compressed')
        self.declare_parameter('preview_topic', '/tent_detections/image/compressed')
        self.declare_parameter('port', 5000)
        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('targets_json',
                               os.path.expanduser('~/suas_targets/targets.json'))

        p = self.get_parameter
        self.camera_topic = p('camera_topic').value
        self.preview_topic = p('preview_topic').value
        self.targets_json = p('targets_json').value

        self.pub = self.create_publisher(OperatorMark, '/operator_mark', 10)

        # ── Obecnosc operatora ──────────────────────────────────────────
        # Misja pyta o potwierdzenie tylko wtedy, gdy ktos faktycznie patrzy.
        # Dowodem jest PULS OD PRZEGLADARKI (GET /ping co 2 s), a nie liczba
        # widzow po naszej stronie: gdy padnie Tailscale, przegladarka nie
        # zamyka polaczenia, a TCP bez keepalive zauwaza to dopiero po minutach.
        # Przy zerwanym laczu pingi znikaja i /operator_online robi sie false
        # w ciagu ok. 3 s.
        self.declare_parameter('operator_ping_ttl', 5.0)
        self.ping_ttl = self.get_parameter('operator_ping_ttl').value
        self._last_ping = 0.0
        self.online_pub = self.create_publisher(Bool, '/operator_online', 10)
        self.create_timer(1.0, self._publish_online)

        # ── Podglad + rolling-cache klatki: subskrypcje tylko gdy ktos patrzy ──
        self._viewers = 0
        self._viewers_lock = threading.Lock()
        self._preview_sub = None
        self._last_jpeg = None
        self._jpeg_event = threading.Event()
        # Ostatnia klatka z camera_topic (gotowy JPEG + stamp). /grab oddaje ja
        # od reki, wiec zamrozenie jest natychmiastowe — bez zakladania nowej
        # subskrypcji i czekania na klatke przy kazdym klikniejciu.
        self._cam_sub = None
        self._last_cam = None
        # Tworzenie i kasowanie subskrypcji robimy w timerze, czyli w watku
        # executora — a nie w watku HTTP. Inaczej grzebalibysmy w wezle
        # rownolegle do spinu, co potrafi sie zemscic w najgorszym momencie.
        self.create_timer(0.5, self._sync_subs)

        port = p('port').value
        host = p('host').value
        self._srv = ThreadingHTTPServer((host, port), self._handler_factory())
        threading.Thread(target=self._srv.serve_forever, daemon=True).start()
        self.get_logger().info(
            f"GUI oznaczania: http://{host}:{port}  |  klatka z {self.camera_topic}, "
            f"podglad z {self.preview_topic}")
        self.get_logger().info(
            "Bez otwartej strony ten wezel nie subskrybuje niczego.")

    # ────────────────────── Obecnosc operatora ──────────────────────

    def _publish_online(self):
        """Czy operator patrzy: ostatni ping z przegladarki mlodszy niz ttl.

        Publikujemy co sekunde niezaleznie od wyniku, zeby misja odrozniala
        "operatora nie ma" od "wezel GUI nie chodzi" (wtedy nie ma zadnych
        wiadomosci i misja tez uzna, ze operatora nie ma — to samo zachowanie,
        wiec bezpiecznie).
        """
        online = (time.time() - self._last_ping) < self.ping_ttl
        if online != getattr(self, '_online_last', None):
            self._online_last = online
            self.get_logger().info(
                f"operator {'JEST' if online else 'NIE MA'} — misja "
                f"{'bedzie pytac' if online else 'decyduje sama'}")
        self.online_pub.publish(Bool(data=online))

    # ────────────────────── Podglad ──────────────────────

    def _sync_subs(self):
        with self._viewers_lock:
            want = self._viewers > 0
        if want and self._preview_sub is None:
            qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                             reliability=ReliabilityPolicy.BEST_EFFORT)
            self._preview_sub = self.create_subscription(
                CompressedImage, self.preview_topic, self._preview_cb, qos)
            self._cam_sub = self.create_subscription(
                CompressedImage, self.camera_topic, self._cam_cb, qos)
            self.get_logger().info("podglad WLACZONY (ktos oglada)")
        elif not want and self._preview_sub is not None:
            self.destroy_subscription(self._preview_sub)
            self.destroy_subscription(self._cam_sub)
            self._preview_sub = None
            self._cam_sub = None
            self._last_jpeg = None
            self._last_cam = None
            self.get_logger().info("podglad WYLACZONY (nikt nie oglada)")

    def _preview_cb(self, msg: CompressedImage):
        # Zero dekodowania — przekazujemy bajty tak, jak przyszly.
        self._last_jpeg = bytes(msg.data)
        self._jpeg_event.set()

    def _cam_cb(self, msg: CompressedImage):
        # Gotowy JPEG prosto z kamery -> rolling-cache do zamrazania (bytes, stamp).
        self._last_cam = (bytes(msg.data), msg.header.stamp)

    # ────────────────────── Lapanie klatki ──────────────────────

    def grab(self):
        """Ostatnia klatka z kamery (gotowy JPEG) + jej stamp — od reki.

        Bierzemy ja z rolling-cache napelnianego subskrypcja camera_topic (patrz
        _sync_subs), ktora zyje tylko gdy ktos patrzy. ZERO dekodowania i ZERO
        czekania na nowa subskrypcje — dlatego zamrozenie jest natychmiastowe.
        Bajty ida do przegladarki tak, jak przyszly z kamery (JPEG 1024x1024).
        """
        return self._last_cam or (None, None)

    # ────────────────────── Znacznik ──────────────────────

    def mark(self, class_id, u, v, stamp):
        # Strona potrafi przyslac null, gdy klik padl zanim klatka sie zaladowala
        # (JSON.stringify zamienia NaN na null). Lepiej powiedziec to wprost niz
        # wywalic sie na float(None).
        if u is None or v is None:
            raise ValueError("klik bez wspolrzednych — zamroz klatke jeszcze raz")
        m = OperatorMark()
        if stamp:
            m.header.stamp.sec = int(stamp.get('sec', 0))
            m.header.stamp.nanosec = int(stamp.get('nanosec', 0))
        m.class_id = int(class_id)
        m.u = float(u)
        m.v = float(v)
        self.pub.publish(m)
        name = CLASS_NAMES.get(int(class_id), class_id)
        self.get_logger().info(
            f"znacznik [{name}] piksel ({u:.0f}, {v:.0f}) -> /operator_mark")
        return f"zapisano: {name} @ ({u:.0f}, {v:.0f})"

    def status(self):
        try:
            with open(self.targets_json) as fh:
                data = json.load(fh)
        except Exception:
            return "brak targets.json — geolokator jeszcze nic nie zapisal"
        out = []
        for key, label in (('tent', 'namiot'), ('people', 'czlowiek')):
            best = (data.get(key) or {}).get('best')
            if best:
                out.append(f"{label}: {best['lat']:.6f} {best['lon']:.6f} "
                           f"({best['source']}, obs={best['n_obs']})")
            else:
                out.append(f"{label}: —")
        return "   |   ".join(out)

    # ────────────────────── HTTP ──────────────────────

    def _handler_factory(self):
        node = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *a):
                pass                                  # cisza w konsoli

            def _send(self, code, ctype, body, extra=None):
                self.send_response(code)
                self.send_header('Content-Type', ctype)
                self.send_header('Content-Length', str(len(body)))
                for k, v in (extra or {}).items():
                    self.send_header(k, v)
                self.end_headers()
                self.wfile.write(body)

            def do_GET(self):
                if self.path == '/':
                    self._send(200, 'text/html; charset=utf-8', PAGE.encode())
                elif self.path == '/ping':
                    # Puls obecnosci operatora. Musi przyjsc OD przegladarki —
                    # tylko to dowodzi, ze lacze zyje i ktos patrzy.
                    node._last_ping = time.time()
                    self._send(200, 'text/plain', b'ok')
                elif self.path == '/stream':
                    self._stream()
                elif self.path == '/grab':
                    jpg, stamp = node.grab()
                    if jpg is None:
                        self._send(504, 'text/plain; charset=utf-8',
                                   f"brak klatki z {node.camera_topic}".encode())
                        return
                    self._send(200, 'image/jpeg', jpg, {
                        'X-Stamp-Sec': str(stamp.sec),
                        'X-Stamp-Nsec': str(stamp.nanosec),
                        'Cache-Control': 'no-store'})
                elif self.path == '/status':
                    self._send(200, 'text/plain; charset=utf-8',
                               node.status().encode())
                else:
                    self._send(404, 'text/plain', b'404')

            def do_POST(self):
                if self.path != '/mark':
                    self._send(404, 'text/plain', b'404')
                    return
                n = int(self.headers.get('Content-Length', 0))
                try:
                    d = json.loads(self.rfile.read(n))
                    txt = node.mark(d['class_id'], d['u'], d['v'], d.get('stamp'))
                except Exception as e:
                    self._send(400, 'text/plain; charset=utf-8', str(e).encode())
                    return
                self._send(200, 'text/plain; charset=utf-8', txt.encode())

            def _stream(self):
                """MJPEG. Subskrypcja zyje tylko na czas polaczenia."""
                with node._viewers_lock:
                    node._viewers += 1
                try:
                    self.send_response(200)
                    self.send_header(
                        'Content-Type',
                        'multipart/x-mixed-replace; boundary=frame')
                    self.end_headers()
                    while True:
                        node._jpeg_event.wait(timeout=2.0)
                        node._jpeg_event.clear()
                        jpg = node._last_jpeg
                        if jpg is None:
                            continue
                        self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\n'
                                         b'Content-Length: '
                                         + str(len(jpg)).encode()
                                         + b'\r\n\r\n' + jpg + b'\r\n')
                except (BrokenPipeError, ConnectionResetError):
                    pass
                finally:
                    with node._viewers_lock:
                        node._viewers -= 1

        return Handler

    def destroy_node(self):
        try:
            self._srv.shutdown()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MarkerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
