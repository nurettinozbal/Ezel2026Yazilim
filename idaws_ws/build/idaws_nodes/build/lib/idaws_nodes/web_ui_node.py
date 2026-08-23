"""
web_ui_node
───────────
Kamera kalibrasyon paneli. SADECE dört şey var:

  1. Canlı kamera görüntüsü (YOLO çizimli)
  2. Görüntü filtresi ayarları — güneşte beyazlayan görüntüyü toparlamak için
  3. Kamera (lens/V4L2) ayarları — pozlama, odak, beyaz dengesi...
  4. ROS 2 log akışı

Telemetri, harita, LiDAR radarı, manuel kumanda, parkur seçimi ve kayıt
listesi KALDIRILDI. Görev kontrolü artık tamamen otopilot parametreleri
(SCR_USER1/2/3) üzerinden yapılıyor, panelin görevi yalnızca kamerayı
kalibre edebilmek.

Panel hiçbir donanıma dokunmaz: kamerayı yolo_vision_node açar, bu node
sadece `vision/image_annotated` akışını gösterir ve ayarları ROS parametresi
olarak yolo_vision_node'a YAZAR (/yolo_vision_node/set_parameters). Yani
paneldeki her kaydırıcı, gerçek node parametresini değiştirir — panel
kapansa bile ayar node'da kalır.
"""

import json
import os
import re
import subprocess
import threading
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String, Int32
from sensor_msgs.msg import CompressedImage
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rcl_interfaces.srv import SetParameters, GetParameters

from flask import Flask, render_template_string, jsonify, request, Response

VISION_NODE = 'yolo_vision_node'

# Panelden yazılabilen parametreler ve tipleri. Tip yanlış gönderilirse
# otopilot değil ROS reddeder — ParameterValue'da doğru alanı doldurmak şart.
BOOL_PARAMS = {'filter_enabled', 'filter_before_detect', 'clahe_enabled', 'record_enabled'}
INT_PARAMS = {'clahe_grid', 'infer_width', 'record_width', 'stream_max_width'}

# "Ayarları Kaydet" panelin FILTER/CAM/DETECT/RECORD listelerindeki TÜM
# alanları jetson_params.yaml'a geri yazar ki idaws stop/start ya da reboot
# sonrası panelden ayarlanan değerler KAYBOLMASIN (normalde bu değerler
# sadece canlı ROS parametresi — dosyaya hiç dokunulmuyordu).
SAVE_PARAM_NAMES = [
    'filter_enabled', 'filter_before_detect', 'gamma', 'brightness_gain',
    'brightness_offset', 'saturation_scale', 'clahe_enabled', 'clahe_clip', 'clahe_grid',
    'auto_exposure', 'exposure', 'brightness', 'contrast', 'saturation', 'gain',
    'sharpness', 'auto_wb', 'wb_temperature', 'autofocus', 'focus', 'zoom',
    'confidence_threshold', 'infer_width',
    'record_enabled', 'record_segment_sec', 'record_fps', 'record_width',
]

# idaws_jetson paketi config/'i colcon build'te KOPYALAR (symlink değil) —
# bu yüzden sadece kaynağa yazmak yeterli değil, install/ altındaki asıl
# çalışma zamanında okunan kopya da güncellenmeli, yoksa "kaydet" bir
# sonraki 'idaws start'ta hiç etkili olmaz (rebuild gerektirir).
JETSON_PARAMS_PATHS = [
    '/home/ezelproject/idaws-jetson/ros2/idaws_jetson/config/jetson_params.yaml',
    '/home/ezelproject/idaws_ws/install/idaws_jetson/share/idaws_jetson/config/jetson_params.yaml',
]

# WiFi/hotspot yönetimi: sahada Jetson'a doğrudan erişim yok, panel araca
# bağlı bir tarayıcıdan (mevcut ağ ya da hotspot üzerinden) tek kontrol yolu.
# nmcli çağrıları ile NetworkManager sürülür — ezelproject kullanıcısının
# `nmcli general permissions` çıktısı network-control/settings.modify.system/
# wifi.share.* için hepsi "yes", yani sudo GEREKMİYOR.
#
# ÖNEMLİ: idaws-jetson deposunda ZATEN açılışta otomatik çalışan bir
# systemd servisi (idaws-hotspot.service) ve onun kurduğu bir NM profili
# (con-name "idaws-hotspot", KÜÇÜK harf) var — bkz. network/hotspot.sh.
# Panel BUNUNLA AYNI profili/servisi sürer; ayrı bir profil (ör. farklı
# harf büyüklüğü) açarsak ikisi birbirini yönetemeden çakışır ve panelden
# "kapat" denince gerçek profil kapanmaz (yaşandı — servis "active (exited)"
# olarak kalıp NM'in kendi autoconnect'i ya da systemd Restart= mantığı
# hotspot'u geri getirebiliyordu). "Kapat" bu yüzden HEM servisi durdurur
# HEM connection.autoconnect'i kapatır ki hiçbir mekanizma geri getirmesin.
HOTSPOT_SCRIPT = '/home/ezelproject/idaws-jetson/network/hotspot.sh'
HOTSPOT_SERVICE = 'idaws-hotspot.service'
WIFI_HOTSPOT_CONNECTION = 'idaws-hotspot'
_ANSI_RE = re.compile(r'\x1b\[[0-9;]*m')

# Hazır ayar takımları — sahada tek tıkla başlangıç noktası.
PRESETS = {
    'varsayilan': {
        'filter_enabled': True, 'gamma': 1.0, 'brightness_gain': 1.0,
        'brightness_offset': 0.0, 'saturation_scale': 1.0,
        'clahe_enabled': False, 'auto_exposure': 3.0,
    },
    'gunes': {
        # Yanmayı önlemek için pozlamayı MANUEL'e alıp kısar, gamma ile
        # gölgeleri geri getirir, CLAHE ile yerel kontrastı toparlar.
        'filter_enabled': True, 'gamma': 0.75, 'brightness_gain': 1.0,
        'brightness_offset': -10.0, 'saturation_scale': 1.3,
        'clahe_enabled': True, 'clahe_clip': 2.5,
        'auto_exposure': 1.0, 'exposure': 80.0,
    },
    'bulutlu': {
        'filter_enabled': True, 'gamma': 1.15, 'brightness_gain': 1.1,
        'brightness_offset': 5.0, 'saturation_scale': 1.15,
        'clahe_enabled': False, 'auto_exposure': 3.0,
    },
}


class WebUiNode(Node):

    def __init__(self):
        super().__init__('web_ui_node')

        self.declare_parameter('web_port', 5000)
        self.declare_parameter('annotated_topic', 'vision/image_annotated')
        self.declare_parameter('stream_fps', 25.0)
        self.declare_parameter('log_lines', 300)

        self._port = int(self.get_parameter('web_port').value)
        self._annotated_topic = self.get_parameter('annotated_topic').value
        self._stream_fps = max(1.0, float(self.get_parameter('stream_fps').value))
        log_lines = int(self.get_parameter('log_lines').value)

        self._frame_lock = threading.Lock()
        self._jpeg = None
        self._last_frame_t = 0.0
        self._stats = {}
        self._log = deque(maxlen=log_lines)

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        # JPEG akışı: yolo_vision_node zaten sıkıştırıyor, panel baytları
        # tarayıcıya OLDUĞU GİBİ verir — decode/encode YOK. Ham Image'ı alıp
        # yeniden JPEG'lemek hem Jetson CPU'sunu hem DDS'i boşuna yiyordu.
        self.create_subscription(CompressedImage, self._annotated_topic + '/compressed',
                                 self._cb_compressed, sensor_qos)
        self.create_subscription(String, 'vision/stats', self._cb_stats, 10)
        self._subscribe_rosout()

        self._param_client = self.create_client(
            SetParameters, f'/{VISION_NODE}/set_parameters')
        self._get_param_client = self.create_client(
            GetParameters, f'/{VISION_NODE}/get_parameters')
        # Doğrudan model seçimi — SCR_USER4 ile aynı model_dir indeksini
        # kullanır (bkz. yolo_vision_node._select_model_by_index).
        self.pub_select_model = self.create_publisher(Int32, 'cmd/select_model', 10)

        self._app = Flask(__name__)
        self._setup_routes()
        threading.Thread(target=self._run_flask, daemon=True).start()

        self.get_logger().info(
            f'web_ui_node başlatıldı — panel http://0.0.0.0:{self._port} '
            f'(kamera kalibrasyon paneli)'
        )

    # ───────────────────── Abonelikler ─────────────────────

    def _subscribe_rosout(self):
        try:
            from rcl_interfaces.msg import Log
        except ImportError:
            self.get_logger().warn('rcl_interfaces/Log yok — log paneli devre dışı.')
            return
        # /rosout derin bir kuyrukla yayınlanır; VOLATILE abone uyumludur.
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
        )
        self.create_subscription(Log, '/rosout', self._cb_log, qos)

    _LEVELS = {10: 'DEBUG', 20: 'INFO', 30: 'WARN', 40: 'ERROR', 50: 'FATAL'}

    def _cb_log(self, msg):
        self._log.append({
            't': time.strftime('%H:%M:%S'),
            'lvl': self._LEVELS.get(msg.level, str(msg.level)),
            'node': msg.name,
            'msg': msg.msg,
        })

    def _cb_stats(self, msg: String):
        try:
            self._stats = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    def _cb_compressed(self, msg: CompressedImage):
        """Gelen JPEG doğrudan saklanır — decode edilmez."""
        self._last_frame_t = time.monotonic()
        with self._frame_lock:
            self._jpeg = bytes(msg.data)

    # ───────────────────── Parametre yazma ─────────────────────

    def _set_vision_params(self, values: dict):
        """yolo_vision_node'a parametre yazar. Panel Flask thread'inden çağrılır;
        executor ana thread'de döndüğü için call_async + future yoklaması
        kullanılır (spin_until_future_complete başka thread'den çağrılırsa
        executor'ı kilitler)."""
        if not self._param_client.wait_for_service(timeout_sec=2.0):
            return False, f'{VISION_NODE} parametre servisi yok (node çalışıyor mu?)'

        params = []
        for name, value in values.items():
            pv = ParameterValue()
            if name in BOOL_PARAMS:
                pv.type = ParameterType.PARAMETER_BOOL
                pv.bool_value = bool(value)
            elif name in INT_PARAMS:
                pv.type = ParameterType.PARAMETER_INTEGER
                pv.integer_value = int(value)
            else:
                pv.type = ParameterType.PARAMETER_DOUBLE
                pv.double_value = float(value)
            params.append(Parameter(name=name, value=pv))

        req = SetParameters.Request()
        req.parameters = params
        future = self._param_client.call_async(req)

        deadline = time.monotonic() + 3.0
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        if not future.done():
            return False, 'parametre servisi zaman aşımı'

        result = future.result()
        bad = [p.name for p, r in zip(params, result.results) if not r.successful]
        if bad:
            return False, f'reddedildi: {", ".join(bad)}'
        return True, 'ok'

    def _get_vision_params(self, names):
        """yolo_vision_node'un ŞU ANKİ (canlı) parametre değerlerini okur —
        panel yeniden yüklendiğinde sıfırlanan tarayıcı state'ine değil,
        node'daki gerçek değere güvenmek için (bkz. 'Ayarları Kaydet')."""
        if not self._get_param_client.wait_for_service(timeout_sec=2.0):
            return None, f'{VISION_NODE} parametre servisi yok (node çalışıyor mu?)'
        req = GetParameters.Request()
        req.names = list(names)
        future = self._get_param_client.call_async(req)

        deadline = time.monotonic() + 3.0
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        if not future.done():
            return None, 'parametre servisi zaman aşımı'

        result = future.result()
        out = {}
        for name, pv in zip(names, result.values):
            if pv.type == ParameterType.PARAMETER_BOOL:
                out[name] = pv.bool_value
            elif pv.type == ParameterType.PARAMETER_INTEGER:
                out[name] = pv.integer_value
            elif pv.type == ParameterType.PARAMETER_DOUBLE:
                out[name] = pv.double_value
            elif pv.type == ParameterType.PARAMETER_STRING:
                out[name] = pv.string_value
            # PARAMETER_NOT_SET: node'da hiç böyle bir parametre yok — atla.
        return out, None

    @staticmethod
    def _format_yaml_value(name, value):
        if isinstance(value, bool):
            return 'true' if value else 'false'
        if name in INT_PARAMS or isinstance(value, int):
            return str(int(value))
        v = round(float(value), 4)
        return f'{v:.1f}' if v == int(v) else repr(v)

    def _patch_yaml_section(self, path, section, values):
        """path içindeki 'section:' bloğunda, values'taki her anahtarın
        DEĞERİNİ yerinde değiştirir — satırın geri kalanı (yorum dahil)
        AYNEN korunur. Tüm dosyayı yaml.dump ile yeniden yazmak yüzlerce
        satırlık Türkçe açıklama yorumunu silerdi, bu yüzden metin bazlı
        nokta atışı düzenleme tercih edildi."""
        try:
            with open(path, 'r', encoding='utf-8') as f:
                lines = f.readlines()
        except OSError as e:
            return False, f'{path} okunamadı: {e}'

        start = next((i for i, l in enumerate(lines) if l.rstrip('\n') == f'{section}:'), None)
        if start is None:
            return False, f'{path} içinde {section}: bulunamadı'
        end = len(lines)
        for i in range(start + 1, len(lines)):
            if lines[i].strip() and not lines[i][0].isspace():
                end = i
                break

        updated = set()
        for i in range(start, end):
            m = re.match(r'^(\s+)([A-Za-z0-9_]+):(\s*)([^#\n]*?)(\s*#.*)?$', lines[i])
            if not m:
                continue
            indent, key, _, _old, comment = m.groups()
            if key not in values:
                continue
            new_val = self._format_yaml_value(key, values[key])
            lines[i] = f'{indent}{key}: {new_val}{comment or ""}\n'
            updated.add(key)

        try:
            with open(path, 'w', encoding='utf-8') as f:
                f.writelines(lines)
        except OSError as e:
            return False, f'{path} yazılamadı: {e}'
        return True, updated

    def _save_vision_params(self):
        values, err = self._get_vision_params(SAVE_PARAM_NAMES)
        if values is None:
            return False, err

        total_updated = set()
        errors = []
        for path in JETSON_PARAMS_PATHS:
            if not os.path.exists(path):
                continue
            ok, result = self._patch_yaml_section(path, 'yolo_vision_node', values)
            if not ok:
                errors.append(result)
            else:
                total_updated |= result

        if errors and not total_updated:
            return False, '; '.join(errors)
        missing = set(SAVE_PARAM_NAMES) - total_updated
        msg = f'{len(total_updated)} ayar kalıcı olarak kaydedildi'
        if missing:
            msg += f' — dosyada bulunamayan {len(missing)} alan: {", ".join(sorted(missing))}'
        return True, msg

    # ───────────────────── WiFi / Hotspot (nmcli) ─────────────────────

    @staticmethod
    def _nmcli(*args, timeout=15):
        try:
            r = subprocess.run(['nmcli'] + list(args), capture_output=True,
                               text=True, timeout=timeout)
            return r.returncode == 0, r.stdout.strip(), r.stderr.strip()
        except (subprocess.TimeoutExpired, FileNotFoundError, OSError) as e:
            return False, '', str(e)

    @staticmethod
    def _split_terse(line):
        # nmcli -t çıktısında ':' kaçışlanır ('\:') — kaçışlanmamış ':' üzerinden böl.
        parts = re.split(r'(?<!\\):', line)
        return [p.replace('\\:', ':') for p in parts]

    def _wifi_device(self):
        ok, out, _ = self._nmcli('-t', '-f', 'DEVICE,TYPE', 'device', 'status')
        if not ok:
            return None
        for line in out.splitlines():
            parts = self._split_terse(line)
            if len(parts) >= 2 and parts[1] == 'wifi':
                return parts[0]
        return None

    def _wifi_status(self):
        dev = self._wifi_device()
        if not dev:
            return {'device': None, 'connected': False, 'ssid': None, 'ip': None, 'hotspot': False}
        ok, out, _ = self._nmcli('-t', '-f', 'GENERAL.CONNECTION,IP4.ADDRESS',
                                 'device', 'show', dev)
        conn = ip = None
        if ok:
            for line in out.splitlines():
                parts = self._split_terse(line)
                if len(parts) < 2:
                    continue
                key, val = parts[0], ':'.join(parts[1:])
                if key == 'GENERAL.CONNECTION':
                    conn = val if val and val != '--' else None
                elif key.startswith('IP4.ADDRESS'):
                    ip = val.split('/')[0] if val else None
        hotspot = conn == WIFI_HOTSPOT_CONNECTION
        return {
            'device': dev,
            'connected': bool(conn) and not hotspot,
            'ssid': conn if (conn and not hotspot) else None,
            'ip': ip,
            'hotspot': hotspot,
        }

    def _wifi_known_ssids(self):
        ok, out, _ = self._nmcli('-t', '-f', 'NAME,TYPE', 'connection', 'show')
        if not ok:
            return []
        names = []
        for line in out.splitlines():
            parts = self._split_terse(line)
            if len(parts) >= 2 and parts[1] == '802-11-wireless' and parts[0] != WIFI_HOTSPOT_CONNECTION:
                names.append(parts[0])
        return names

    def _wifi_scan(self):
        dev = self._wifi_device()
        if not dev:
            return []
        self._nmcli('device', 'wifi', 'rescan', 'ifname', dev, timeout=10)
        time.sleep(2.0)
        ok, out, _ = self._nmcli('-t', '-f', 'SSID,SIGNAL,SECURITY,IN-USE',
                                 'device', 'wifi', 'list', 'ifname', dev)
        if not ok:
            return []
        known = set(self._wifi_known_ssids())
        seen = set()
        result = []
        for line in out.splitlines():
            parts = self._split_terse(line)
            if len(parts) < 4:
                continue
            ssid, signal, security, in_use = parts[0], parts[1], parts[2], parts[3]
            if not ssid or ssid in seen:
                continue
            seen.add(ssid)
            result.append({
                'ssid': ssid,
                'signal': int(signal) if signal.isdigit() else 0,
                'secure': bool(security) and security != '--',
                'in_use': in_use.strip() == '*',
                'saved': ssid in known,
            })
        result.sort(key=lambda n: -n['signal'])
        return result

    def _hotspot_stop_if_active(self):
        if self._wifi_status()['hotspot']:
            self._hotspot_stop()

    @staticmethod
    def _strip_ansi(s):
        return _ANSI_RE.sub('', s or '')

    @staticmethod
    def _systemctl(*args, timeout=20):
        try:
            r = subprocess.run(['systemctl'] + list(args), capture_output=True,
                               text=True, timeout=timeout)
            return r.returncode == 0, r.stdout.strip(), r.stderr.strip()
        except (subprocess.TimeoutExpired, FileNotFoundError, OSError) as e:
            return False, '', str(e)

    def _wifi_connect(self, ssid, password):
        dev = self._wifi_device()
        if not dev:
            return False, 'wifi cihazı bulunamadı'
        self._hotspot_stop_if_active()
        args = ['device', 'wifi', 'connect', ssid, 'ifname', dev]
        if password:
            args += ['password', password]
        ok, out, err = self._nmcli(*args, timeout=30)
        if ok:
            return True, f'{ssid} ağına bağlandı'
        return False, (err or out or f'{ssid} ağına bağlanılamadı')

    def _hotspot_start(self, ssid, password):
        if len(password) < 8:
            return False, 'hotspot şifresi en az 8 karakter olmalı (WPA2)'
        # hotspot.sh SSID/şifreyi env'den okur (IDAWS_SSID/IDAWS_WIFI_PASS);
        # script zaten mevcut "idaws-hotspot" profilini günceller/oluşturur
        # ve connection.autoconnect'i yeniden "yes" yapar (aşağıdaki "kapat"
        # ile kapatılmış olsa bile) — systemd servisiyle AYNI kod yolu.
        env = dict(os.environ)
        env['IDAWS_SSID'] = ssid
        env['IDAWS_WIFI_PASS'] = password
        try:
            r = subprocess.run(['bash', HOTSPOT_SCRIPT, 'up'],
                               capture_output=True, text=True, timeout=25, env=env)
        except (subprocess.TimeoutExpired, FileNotFoundError, OSError) as e:
            return False, str(e)
        out = self._strip_ansi((r.stdout or '') + (r.stderr or '')).strip()
        if r.returncode == 0:
            return True, f'hotspot açıldı: {ssid}'
        return False, (out or 'hotspot açılamadı')

    def _hotspot_stop(self):
        # ÖNCE systemd servisini durdur — bu, açılıştaki OTOMATİK hotspot
        # mekanizmasıyla AYNI yol (ExecStop=hotspot.sh down) olduğu için
        # servisin "active (exited)" takılı kalıp arkadan geri getirmesini
        # önler. Servis yönetilemiyorsa (ör. polkit/oturum yoksa) doğrudan
        # script'e düş.
        ok, _, err = self._systemctl('stop', HOTSPOT_SERVICE)
        out = err
        if not ok:
            try:
                r = subprocess.run(['bash', HOTSPOT_SCRIPT, 'down'],
                                   capture_output=True, text=True, timeout=20)
                ok = r.returncode == 0
                out = self._strip_ansi((r.stdout or '') + (r.stderr or '')).strip()
            except (subprocess.TimeoutExpired, FileNotFoundError, OSError) as e:
                return False, str(e)
        # Servis/nmcli "down" yapsa bile profil autoconnect:yes olarak
        # KALIYORDU (hotspot.sh'nin kendi tasarımı — "up" ile hemen geri
        # gelsin diye) — NetworkManager'ın bunu kendiliğinden yeniden
        # etkinleştirmesini de burada kapatıyoruz. "Kapatınca TAMAMEN
        # kapansın" tam olarak bunu istiyor.
        self._nmcli('connection', 'modify', WIFI_HOTSPOT_CONNECTION,
                   'connection.autoconnect', 'no', timeout=10)
        if not ok:
            return False, (out or 'hotspot kapatılamadı')
        return True, 'hotspot kapatıldı (otomatik yeniden bağlanma da devre dışı)'

    # ───────────────────── Flask ─────────────────────

    def _run_flask(self):
        self._app.run(host='0.0.0.0', port=self._port,
                      threaded=True, debug=False, use_reloader=False)

    def _mjpeg(self):
        boundary = b'--frame\r\n'
        period = 1.0 / self._stream_fps
        while True:
            with self._frame_lock:
                jpeg = self._jpeg
            if jpeg is not None:
                yield boundary + b'Content-Type: image/jpeg\r\n\r\n' + jpeg + b'\r\n'
            time.sleep(period)

    def _setup_routes(self):
        app = self._app

        @app.route('/')
        def index():
            return render_template_string(PAGE)

        @app.route('/stream')
        def stream():
            return Response(self._mjpeg(),
                            mimetype='multipart/x-mixed-replace; boundary=frame')

        @app.route('/api/status')
        def status():
            alive = (time.monotonic() - self._last_frame_t) < 2.0
            return jsonify({'stats': self._stats, 'camera_alive': alive})

        @app.route('/api/log')
        def log():
            return jsonify({'lines': list(self._log)})

        @app.route('/api/params', methods=['POST'])
        def set_params():
            data = request.get_json(silent=True) or {}
            if not data:
                return jsonify({'ok': False, 'msg': 'boş istek'}), 400
            ok, msg = self._set_vision_params(data)
            return jsonify({'ok': ok, 'msg': msg}), (200 if ok else 500)

        @app.route('/api/preset/<name>', methods=['POST'])
        def preset(name):
            if name not in PRESETS:
                return jsonify({'ok': False, 'msg': 'bilinmeyen preset'}), 404
            ok, msg = self._set_vision_params(PRESETS[name])
            return jsonify({'ok': ok, 'msg': msg, 'values': PRESETS[name]}), (200 if ok else 500)

        @app.route('/api/params/save', methods=['POST'])
        def save_params():
            ok, msg = self._save_vision_params()
            return jsonify({'ok': ok, 'msg': msg}), (200 if ok else 500)

        @app.route('/api/model/select', methods=['POST'])
        def select_model():
            data = request.get_json(silent=True) or {}
            try:
                idx = int(data['index'])
            except (KeyError, TypeError, ValueError):
                return jsonify({'ok': False, 'msg': 'gecersiz index'}), 400
            msg = Int32()
            msg.data = idx
            self.pub_select_model.publish(msg)
            return jsonify({'ok': True, 'msg': f'model_index={idx} gonderildi'})

        @app.route('/api/wifi/status')
        def wifi_status():
            return jsonify(self._wifi_status())

        @app.route('/api/wifi/scan')
        def wifi_scan():
            return jsonify({'networks': self._wifi_scan()})

        @app.route('/api/wifi/connect', methods=['POST'])
        def wifi_connect():
            data = request.get_json(silent=True) or {}
            ssid = str(data.get('ssid', '')).strip()
            password = str(data.get('password', ''))
            if not ssid:
                return jsonify({'ok': False, 'msg': 'ssid boş'}), 400
            ok, msg = self._wifi_connect(ssid, password)
            return jsonify({'ok': ok, 'msg': msg}), (200 if ok else 500)

        @app.route('/api/wifi/hotspot/on', methods=['POST'])
        def wifi_hotspot_on():
            data = request.get_json(silent=True) or {}
            ssid = str(data.get('ssid', '')).strip() or 'IDAWS-USV'
            password = str(data.get('password', ''))
            ok, msg = self._hotspot_start(ssid, password)
            return jsonify({'ok': ok, 'msg': msg}), (200 if ok else 500)

        @app.route('/api/wifi/hotspot/off', methods=['POST'])
        def wifi_hotspot_off():
            ok, msg = self._hotspot_stop()
            return jsonify({'ok': ok, 'msg': msg}), (200 if ok else 500)


PAGE = r"""
<!doctype html>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>IDAWS Kamera</title>
<style>
  :root { --bg:#111; --panel:#1b1b1b; --line:#2e2e2e; --fg:#e8e8e8; --dim:#9a9a9a; --ok:#39d353; --bad:#ff5555; }
  * { box-sizing: border-box; }
  body { margin:0; background:var(--bg); color:var(--fg);
         font:14px/1.4 system-ui, sans-serif; }
  .wrap { display:flex; height:100vh; }
  .left { flex:1; display:flex; flex-direction:column; min-width:0; }
  .right { width:340px; background:var(--panel); border-left:1px solid var(--line);
           overflow-y:auto; padding:12px; }
  .view { flex:1; display:flex; align-items:center; justify-content:center;
          background:#000; min-height:0; }
  .view img { max-width:100%; max-height:100%; object-fit:contain; }
  .bar { display:flex; gap:14px; flex-wrap:wrap; padding:8px 12px;
         background:var(--panel); border-top:1px solid var(--line); font-size:13px; }
  .bar b { color:#fff; }
  .logbox { height:190px; overflow-y:auto; background:#0c0c0c;
            border-top:1px solid var(--line); font:11px/1.35 ui-monospace, monospace;
            padding:6px 10px; }
  .logbox div { white-space:pre-wrap; word-break:break-word; }
  .WARN { color:#e3b341; } .ERROR,.FATAL { color:var(--bad); } .INFO { color:#8b8b8b; }
  h3 { margin:16px 0 8px; font-size:12px; text-transform:uppercase;
       letter-spacing:.08em; color:var(--dim); border-bottom:1px solid var(--line);
       padding-bottom:5px; }
  h3:first-child { margin-top:0; }
  .row { display:flex; align-items:center; gap:8px; margin:7px 0; }
  .row label { flex:0 0 118px; font-size:12px; color:#ccc; }
  .row input[type=range] { flex:1; min-width:0; accent-color:#4a9eff; }
  .row .val { flex:0 0 46px; text-align:right; font:11px ui-monospace,monospace;
              color:#4a9eff; }
  .row input[type=checkbox] { accent-color:#4a9eff; width:16px; height:16px; }
  .presets { display:flex; gap:6px; }
  .presets button { flex:1; padding:8px 4px; background:#262626; color:var(--fg);
                    border:1px solid var(--line); border-radius:4px; cursor:pointer;
                    font-size:12px; }
  .presets button:hover { background:#333; }
  .hint { font-size:11px; color:var(--dim); margin:4px 0 0; }
  #toast { position:fixed; bottom:14px; left:50%; transform:translateX(-50%);
           background:#000; border:1px solid var(--line); padding:8px 14px;
           border-radius:4px; opacity:0; transition:opacity .25s; font-size:13px; }
  #toast.show { opacity:1; }
  @media (max-width: 820px) { .wrap { flex-direction:column; } .right { width:auto; } }
</style>

<div class="wrap">
  <div class="left">
    <div class="view"><img src="/stream" alt="kamera"></div>
    <div class="bar">
      <span>kamera <b id="s-cam">—</b></span>
      <span>yakalama <b id="s-cap">—</b></span>
      <span>tespit <b id="s-det">—</b></span>
      <span>cikis <b id="s-out">—</b></span>
      <span>gecikme <b id="s-ms">—</b></span>
      <span>kutu <b id="s-box">—</b></span>
      <span>model <b id="s-model">—</b></span>
      <span>kayit <b id="s-rec">—</b></span>
    </div>
    <div class="bar" id="model-list" style="font-size:0.85em; opacity:0.85;">model_dir taranıyor…</div>
    <div class="logbox" id="log"></div>
  </div>

  <div class="right">
    <h3>Ag (WiFi)</h3>
    <div id="wifi-status" class="hint">yukleniyor...</div>
    <div class="row" style="margin-top:8px;">
      <button id="hotspot-toggle" onclick="toggleHotspot()" style="flex:1;padding:7px;
              background:#262626;color:#eee;border:1px solid #2e2e2e;border-radius:4px;cursor:pointer;">
        Hotspot
      </button>
      <button onclick="scanWifi()" style="flex:1;padding:7px;
              background:#262626;color:#eee;border:1px solid #2e2e2e;border-radius:4px;cursor:pointer;">
        Ag Tara
      </button>
    </div>
    <div id="hotspot-form" style="display:none; margin-top:6px;">
      <input id="hs-ssid" placeholder="Hotspot adi (SSID)" value="IDAWS-USV"
             style="width:100%;margin-bottom:5px;padding:6px;background:#111;color:#eee;
                    border:1px solid #333;border-radius:4px;box-sizing:border-box;">
      <input id="hs-pass" placeholder="Sifre (en az 8 karakter)" type="text"
             style="width:100%;margin-bottom:5px;padding:6px;background:#111;color:#eee;
                    border:1px solid #333;border-radius:4px;box-sizing:border-box;">
      <button onclick="startHotspot()" style="width:100%;padding:7px;background:#1f3a5c;
              color:#eee;border:1px solid #2e2e2e;border-radius:4px;cursor:pointer;">Hotspot Ac</button>
    </div>
    <div id="wifi-list" style="margin-top:6px; max-height:150px; overflow-y:auto;"></div>
    <div id="wifi-connect-form" style="display:none; margin-top:6px;">
      <div id="wifi-connect-ssid" style="font-size:12px; color:#ccc; margin-bottom:4px;"></div>
      <input id="wc-pass" placeholder="Sifre (kayitliysa bos birak)" type="text"
             style="width:100%;margin-bottom:5px;padding:6px;background:#111;color:#eee;
                    border:1px solid #333;border-radius:4px;box-sizing:border-box;">
      <button onclick="connectWifi()" style="width:100%;padding:7px;background:#1f3a5c;
              color:#eee;border:1px solid #2e2e2e;border-radius:4px;cursor:pointer;">Bagla</button>
    </div>
    <p class="hint">Hotspot acarken/ag degistirirken panel gecici olarak baglantiyi kaybedebilir.</p>

    <h3>Hazir ayar</h3>
    <div class="presets">
      <button onclick="preset('varsayilan')">Varsayilan</button>
      <button onclick="preset('gunes')">Gunes</button>
      <button onclick="preset('bulutlu')">Bulutlu</button>
    </div>
    <p class="hint">Gunes: pozlama manuel + kisik, gamma dusuk, CLAHE acik.</p>
    <button onclick="saveParams()" style="width:100%;padding:8px;margin-top:6px;
            background:#1f3a5c;color:#eee;border:1px solid #2e2e2e;border-radius:4px;cursor:pointer;">
      Ayarlari Kaydet (kalici)
    </button>
    <p class="hint">Kaydirici degisiklikleri normalde sadece bu oturumda gecerli —
       idaws stop/start ya da reboot sonrasi dosyadaki eski degerlere doner.
       Bu buton su anki degerleri jetson_params.yaml'a yazar.</p>

    <h3>Goruntu filtresi</h3>
    <div id="filter"></div>

    <h3>Kamera (lens)</h3>
    <p class="hint">-1 = dokunma (kameranin kendi ayari).
       Pozlama icin once auto_exposure = 1 (manuel) yap.</p>
    <div id="cam"></div>

    <h3>Tespit</h3>
    <div id="detect"></div>

    <h3>Kayit</h3>
    <div id="record"></div>
  </div>
</div>
<div id="toast"></div>

<script>
// [ad, etiket, min, max, adim, varsayilan, tip]
const FILTER = [
  ['filter_enabled','Filtre acik',0,1,1,1,'bool'],
  ['filter_before_detect','YOLO-ya da uygula',0,1,1,1,'bool'],
  ['gamma','Gamma',0.3,2.5,0.05,1.0,'num'],
  ['brightness_gain','Kontrast',0.5,2.5,0.05,1.0,'num'],
  ['brightness_offset','Parlaklik',-80,80,1,0,'num'],
  ['saturation_scale','Doygunluk',0.0,2.5,0.05,1.0,'num'],
  ['clahe_enabled','CLAHE acik',0,1,1,0,'bool'],
  ['clahe_clip','CLAHE siddet',0.5,8.0,0.1,2.0,'num'],
  ['clahe_grid','CLAHE izgara',2,16,1,8,'int'],
];
const CAM = [
  ['auto_exposure','Oto pozlama',-1,3,1,-1,'num'],
  ['exposure','Pozlama',-1,2000,1,-1,'num'],
  ['brightness','Parlaklik',-1,255,1,-1,'num'],
  ['contrast','Kontrast',-1,255,1,-1,'num'],
  ['saturation','Doygunluk',-1,255,1,-1,'num'],
  ['gain','Kazanc',-1,255,1,-1,'num'],
  ['sharpness','Keskinlik',-1,255,1,-1,'num'],
  ['auto_wb','Oto beyaz deng.',-1,1,1,-1,'num'],
  ['wb_temperature','Beyaz deng. K',-1,10000,50,-1,'num'],
  ['autofocus','Oto odak',-1,1,1,-1,'num'],
  ['focus','Odak',-1,255,1,-1,'num'],
  ['zoom','Zoom',-1,500,1,-1,'num'],
];
const DETECT = [
  ['confidence_threshold','Guven esigi',0.05,0.95,0.05,0.45,'num'],
  ['infer_width','Cikarim genisligi',320,1920,32,960,'int'],
];
const RECORD = [
  ['record_enabled','Kayit acik',0,1,1,1,'bool'],
  ['record_segment_sec','Segment sn',10,300,5,30,'num'],
  ['record_fps','Kayit FPS',5,50,1,25,'num'],
  ['record_width','Kayit genisligi',640,1920,160,1280,'int'],
];

function build(hostId, spec) {
  const host = document.getElementById(hostId);
  spec.forEach(([name,label,min,max,step,def,type]) => {
    const row = document.createElement('div');
    row.className = 'row';
    if (type === 'bool') {
      row.innerHTML = `<label>${label}</label>
        <input type="checkbox" id="p-${name}" ${def ? 'checked':''}>
        <span class="val"></span>`;
      row.querySelector('input').onchange = e =>
        send({[name]: e.target.checked});
    } else {
      row.innerHTML = `<label>${label}</label>
        <input type="range" id="p-${name}" min="${min}" max="${max}"
               step="${step}" value="${def}">
        <span class="val" id="v-${name}">${def}</span>`;
      const inp = row.querySelector('input');
      inp.oninput = () => document.getElementById('v-'+name).textContent = inp.value;
      // Surukleme bitince gonder: her pikselde servis cagirmak node-u bogar.
      inp.onchange = () =>
        send({[name]: type === 'int' ? parseInt(inp.value) : parseFloat(inp.value)});
    }
    host.appendChild(row);
  });
}

let toastT;
function toast(msg, bad) {
  const t = document.getElementById('toast');
  t.textContent = msg;
  t.style.borderColor = bad ? '#ff5555' : '#2e2e2e';
  t.classList.add('show');
  clearTimeout(toastT);
  toastT = setTimeout(() => t.classList.remove('show'), 1800);
}

async function send(obj) {
  try {
    const r = await fetch('/api/params', {
      method:'POST', headers:{'Content-Type':'application/json'},
      body: JSON.stringify(obj)});
    const d = await r.json();
    if (!d.ok) toast(d.msg, true);
  } catch (e) { toast('baglanti hatasi', true); }
}

async function saveParams() {
  toast('kaydediliyor...');
  try {
    const r = await fetch('/api/params/save', {method:'POST'});
    const d = await r.json();
    toast(d.msg, !d.ok);
  } catch (e) { toast('baglanti hatasi', true); }
}

async function preset(name) {
  const r = await fetch('/api/preset/'+name, {method:'POST'});
  const d = await r.json();
  if (!d.ok) { toast(d.msg, true); return; }
  for (const [k,v] of Object.entries(d.values)) {
    const el = document.getElementById('p-'+k);
    if (!el) continue;
    if (el.type === 'checkbox') el.checked = !!v;
    else { el.value = v; const s = document.getElementById('v-'+k); if (s) s.textContent = v; }
  }
  toast('preset uygulandi: '+name);
}

build('filter', FILTER); build('cam', CAM);
build('detect', DETECT); build('record', RECORD);

async function poll() {
  try {
    const d = await (await fetch('/api/status')).json();
    const s = d.stats || {};
    document.getElementById('s-cam').textContent = d.camera_alive ? 'acik' : (s.camera_error || 'YOK');
    document.getElementById('s-cam').style.color = d.camera_alive ? '#39d353' : '#ff5555';
    document.getElementById('s-cap').textContent = (s.capture_fps ?? '—') + ' fps';
    document.getElementById('s-det').textContent = (s.detect_fps ?? '—') + ' fps';
    document.getElementById('s-out').textContent = (s.output_fps ?? '—') + ' fps';
    document.getElementById('s-ms').textContent = (s.detect_ms ?? '—') + ' ms';
    document.getElementById('s-box').textContent = s.boxes ?? '—';
    document.getElementById('s-model').textContent = s.model ?? '—';
    document.getElementById('s-rec').textContent = s.recording || 'kapali';

    const avail = s.model_available || [];
    const curIdx = s.model_index ?? 0;
    document.getElementById('model-list').textContent = avail.length
      ? 'SCR_USER4: ' + avail.map(m =>
          (m.index === curIdx ? '[' + m.index + '=' + m.name + ']' : m.index + '=' + m.name)
        ).join('  ')
      : 'model_dir bos veya okunamiyor';
  } catch (e) {}
}

async function pollLog() {
  try {
    const d = await (await fetch('/api/log')).json();
    const box = document.getElementById('log');
    const stick = box.scrollTop + box.clientHeight >= box.scrollHeight - 30;
    box.innerHTML = d.lines.map(l =>
      `<div class="${l.lvl}">${l.t} [${l.lvl}] ${l.node}: ${l.msg}</div>`).join('');
    if (stick) box.scrollTop = box.scrollHeight;
  } catch (e) {}
}

let selectedSsid = null;

async function wifiStatus() {
  try {
    const s = await (await fetch('/api/wifi/status')).json();
    const box = document.getElementById('wifi-status');
    if (s.hotspot) {
      box.innerHTML = '<b style="color:#4a9eff">HOTSPOT ACIK</b> — ip: ' + (s.ip || '—') +
        '<br><span style="color:#e3b341">bu kartta AP modundayken ag taramasi calismiyor — ' +
        'once Hotspot Kapat</span>';
    } else if (s.connected) {
      box.innerHTML = 'baglandi: <b>' + s.ssid + '</b> — ip: ' + (s.ip || '—');
    } else {
      box.innerHTML = '<span style="color:#ff5555">baglanti yok</span>' +
        (s.device ? ' — cihaz: ' + s.device : ' — wifi cihazi yok');
    }
    document.getElementById('hotspot-toggle').textContent = s.hotspot ? 'Hotspot Kapat' : 'Hotspot';
  } catch (e) {}
}

function toggleHotspot() {
  const btn = document.getElementById('hotspot-toggle');
  if (btn.textContent === 'Hotspot Kapat') {
    stopHotspot();
  } else {
    const form = document.getElementById('hotspot-form');
    form.style.display = form.style.display === 'none' ? 'block' : 'none';
  }
}

async function startHotspot() {
  const ssid = document.getElementById('hs-ssid').value.trim();
  const password = document.getElementById('hs-pass').value;
  if (!ssid || password.length < 8) { toast('SSID gir, sifre >= 8 karakter olsun', true); return; }
  toast('hotspot aciliyor...');
  try {
    const r = await fetch('/api/wifi/hotspot/on', {method:'POST', headers:{'Content-Type':'application/json'},
      body: JSON.stringify({ssid, password})});
    const d = await r.json();
    toast(d.msg, !d.ok);
    document.getElementById('hotspot-form').style.display = 'none';
    wifiStatus();
  } catch (e) { toast('baglanti hatasi (hotspot acilirken sayfa kopmus olabilir)', true); }
}

async function stopHotspot() {
  toast('hotspot kapatiliyor...');
  try {
    const r = await fetch('/api/wifi/hotspot/off', {method:'POST'});
    const d = await r.json();
    toast(d.msg, !d.ok);
    wifiStatus();
  } catch (e) { toast('baglanti hatasi', true); }
}

async function scanWifi() {
  const list = document.getElementById('wifi-list');
  list.innerHTML = '<div class="hint">taraniyor...</div>';
  try {
    const d = await (await fetch('/api/wifi/scan')).json();
    const nets = d.networks || [];
    if (!nets.length) { list.innerHTML = '<div class="hint">ag bulunamadi</div>'; return; }
    list.innerHTML = '';
    nets.forEach(n => {
      const row = document.createElement('div');
      row.style.cssText = 'display:flex;justify-content:space-between;align-items:center;' +
        'padding:5px 6px;border-bottom:1px solid #2e2e2e;cursor:pointer;font-size:12px;';
      row.innerHTML = '<span>' + (n.in_use ? '* ' : '') + n.ssid + (n.saved ? ' (kayitli)' : '') +
        '</span><span style="color:#9a9a9a">' + n.signal + '%' + (n.secure ? ' (sifreli)' : '') + '</span>';
      row.onclick = () => selectWifi(n.ssid, n.saved);
      list.appendChild(row);
    });
  } catch (e) { list.innerHTML = '<div class="hint">tarama hatasi</div>'; }
}

function selectWifi(ssid, saved) {
  selectedSsid = ssid;
  document.getElementById('wifi-connect-form').style.display = 'block';
  document.getElementById('wifi-connect-ssid').textContent = 'Sec: ' + ssid +
    (saved ? ' (kayitli sifre kullanilacak, bos birakabilirsin)' : '');
  document.getElementById('wc-pass').value = '';
}

async function connectWifi() {
  if (!selectedSsid) return;
  const password = document.getElementById('wc-pass').value;
  toast('baglaniliyor: ' + selectedSsid + ' ...');
  try {
    const r = await fetch('/api/wifi/connect', {method:'POST', headers:{'Content-Type':'application/json'},
      body: JSON.stringify({ssid: selectedSsid, password})});
    const d = await r.json();
    toast(d.msg, !d.ok);
    wifiStatus();
  } catch (e) { toast('baglanti hatasi (ag degisirken sayfa kopmus olabilir)', true); }
}

setInterval(poll, 1000);
setInterval(pollLog, 2000);
setInterval(wifiStatus, 4000);
poll(); pollLog(); wifiStatus();
</script>
"""


def main(args=None):
    rclpy.init(args=args)
    node = WebUiNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # SIGINT'te rclpy kendi handler'ıyla context'i zaten kapatmış olabilir;
        # ikinci kez çağırmak RCLError fırlatır.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
