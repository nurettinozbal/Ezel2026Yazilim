import { useState, useEffect, useRef, useCallback, useEffectEvent } from 'react';
import { VehicleContext } from '@/context/useVehicle';
import { telemetryService } from '@/services/telemetryService';
import { isInsidePolygon } from '@/features/DroneControl/utils/geofence';
import { buildMissionWaypoints } from '@/shared/lib/missionContract';

const GEOFENCE_STORAGE_KEY = 'ezel_iha_geofence';

const loadStoredGeofence = () => {
  try {
    const raw = window.localStorage.getItem(GEOFENCE_STORAGE_KEY);
    if (!raw) return [];
    const parsed = JSON.parse(raw);
    if (!Array.isArray(parsed)) return [];
    return parsed.filter((p) => Number.isFinite(p?.lat) && Number.isFinite(p?.lon));
  } catch {
    // Bozuk/erişilemez localStorage yarışma günü arayüzü çökertmemeli.
    return [];
  }
};

const INITIAL_TELEMETRY = {
  ida: {
    sys_id: 1, speed: 0, battery_percent: 0, voltage: 0.0, current: 0.0,
    gps_sats: 0, hdop: 9.9, rssi: -99,
    mode: 'DISARMED', lat: 0, lon: 0, heading: 0, current_wp: 0,
    target_heading: null, target_speed: null, dist_to_wp: 0,
    motor_left_pwm: null, motor_right_pwm: null,
    motor_left_pct: null, motor_right_pct: null,
    rpm_left: null, rpm_right: null,
    roll: null, pitch: null, yaw: null,
    home_lat: null, home_lon: null, home_locked: false, home_locked_at: 0,
    return_home_pending: false, return_home_status: 'idle', return_home_message: '',
    armed: false, connected: false, last_heartbeat: 0,
    autonomy_state: null, autonomy_action: null, parkur: null,
    perception_detection_count: 0, perception_obstacle_count: 0,
    logging_active: false, logging_count: 0,
    autonomy_last_update_monotonic: 0, autonomy_fresh: false,
  },
  iha: {
    sys_id: 2, battery: 0, voltage: 0.0, alt: 0, detected_color: 'BEKLENİYOR',
    lat: 0, lon: 0, mode: 'DISARMED',
    gps_sats: 0, hdop: 9.9, rssi: -99,
    home_lat: null, home_lon: null, home_locked: false, home_locked_at: 0,
    return_home_pending: false, return_home_status: 'idle', return_home_message: '',
    armed: false, connected: false, last_heartbeat: 0, last_update: 0,
  },
  system: {
    rssi: -99, gps_sats: 0, hdop: 9.9, failsafe_status: 'UNKNOWN',
    ida_link: 'DISCONNECTED', iha_link: 'DISCONNECTED', emergency_active: false,
    target_delivery_status: 'not_sent', target_delivery_message: '',
    target_locked: false, mission_started: false,
    target_source: 'MANUAL', target_confidence: 0, target_locked_at: 0,
    valid_target_colors: ['KIRMIZI', 'YEŞİL', 'SİYAH'],
    mission_uploaded: false, mission_waypoint_count: 0,
    command_auth_required: true, command_auth_configured: false,
    auto_return_on_link_loss: true, return_home_mode: 'RTL',
    logs: [],
  },
};

const STATUS_LABELS = {
  MOCK: 'AKTİF (Mock)',
  CONNECTING: 'BAĞLANIYOR',
  CONNECTED: 'AKTİF (Backend)',
  RECONNECTING: 'YENİDEN BAĞLANIYOR',
  ERROR: 'BAĞLANTI HATASI',
  DISCONNECTED: 'BAĞLANTI YOK',
};

const WARNING_COMMAND_STATUSES = new Set(['sent_unconfirmed', 'acked', 'acked_unverified', 'partial_unconfirmed', 'busy']);
const ERROR_COMMAND_STATUSES = new Set(['rejected', 'failed', 'timeout', 'aborted', 'blocked_unverified', 'partial_failed', 'unauthorized']);

const getCommandLogType = (result) => {
  if (!result.ok || ERROR_COMMAND_STATUSES.has(result.status)) return 'ERROR';
  if (WARNING_COMMAND_STATUSES.has(result.status)) return 'WARNING';
  return 'SUCCESS';
};

export const VehicleProvider = ({ children }) => {
  const [systemLogs, setSystemLogs] = useState([
    { id: 1, time: new Date().toLocaleTimeString(), type: 'INFO', msg: 'Sistem başlatıldı...' },
    { id: 2, time: new Date().toLocaleTimeString(), type: 'SUCCESS', msg: 'Arayüz ve Modüller yüklendi.' },
  ]);
  const [isEmergencyActive, setIsEmergencyActive] = useState(false);
  const [telemetry, setTelemetry] = useState(INITIAL_TELEMETRY);
  const [connectionStatus, setConnectionStatus] = useState('BAĞLANIYOR');
  const [missionPoints, setMissionPoints] = useState([]);
  const [missionDefaultParkur, setMissionDefaultParkurState] = useState(1);
  const [isLogging, setIsLogging] = useState(false);
  const [geofencePoints, setGeofencePoints] = useState(loadStoredGeofence);
  const [debugSnapshot, setDebugSnapshot] = useState(null);
  const [vehicleTests, setVehicleTests] = useState([]);
  const [vehicleTestResult, setVehicleTestResult] = useState(null);
  const [vehicleTestEvents, setVehicleTestEvents] = useState([]);
  const pendingManualTargetSend = useRef(false);
  const previousGeofenceStateRef = useRef(null);
  const missionPointSequenceRef = useRef(0);

  const addLog = useCallback((message, type = 'INFO') => {
    const newLog = {
      id: Date.now() + Math.random(),
      time: new Date().toLocaleTimeString(),
      type,
      msg: message,
    };
    setSystemLogs(prev => [...prev, newLog].slice(-100));
  }, []);

  const handleTelemetry = useEffectEvent((newData) => {
    const merged = {
      ida: { ...INITIAL_TELEMETRY.ida, ...newData.ida },
      iha: { ...INITIAL_TELEMETRY.iha, ...newData.iha },
      system: { ...INITIAL_TELEMETRY.system, ...newData.system },
    };
    setTelemetry(merged);
    setIsEmergencyActive(Boolean(merged.system.emergency_active));

    // Masaüstü testleri için KRİTİK BATARYA voltaj denetimi tamamen kaldırıldı.

    // İHA uçuş bölgesi ihlali (şartname §5.5.3.1). Telemetri gelişi harici bir
    // olaydır; kontrolü burada yapmak effect içinde setState çağırmaktan kaçınır.
    // Yalnız durum DEĞİŞİMİNDE loglanır — her tick loglamak terminali boğardı.
    const insideNow = isInsidePolygon(merged.iha.lat, merged.iha.lon, geofencePoints);
    if (insideNow !== null && previousGeofenceStateRef.current !== insideNow) {
      const isFirstObservation = previousGeofenceStateRef.current === null;
      previousGeofenceStateRef.current = insideNow;

      if (!insideNow) {
        addLog('İHA UÇUŞ BÖLGESİ DIŞINDA! Şartname §5.5.3.1: Parkur-3 başarısız sayılabilir.', 'ERROR');
      } else if (!isFirstObservation) {
        addLog('İHA uçuş bölgesine geri döndü.', 'WARNING');
      }
    }
  });

  const handleBackendLog = useEffectEvent((log) => {
    addLog(log.message || log.msg || 'Backend log mesajı', log.level || log.type || 'INFO');
  });

  const handleStatus = useEffectEvent((status) => {
    setConnectionStatus(STATUS_LABELS[status] || status);
    if (!['CONNECTED', 'MOCK'].includes(status)) {
      setTelemetry((previous) => ({
        ...previous,
        ida: { ...previous.ida, autonomy_fresh: false },
      }));
    }
  });

  const handleDebugSnapshot = useEffectEvent((snapshot) => {
    setDebugSnapshot({
      ...snapshot,
      client_received_monotonic: window.performance.now() / 1000,
    });
  });

  const handleVehicleTest = useEffectEvent((event) => {
    if (event.event === 'manifest' && Array.isArray(event.tests)) {
      setVehicleTests(event.tests);
      return;
    }
    setVehicleTestEvents((previous) => [...previous, event].slice(-50));
    setVehicleTestResult(event);
  });

  const handleCommandResult = useEffectEvent((result) => {
    if (result.command === 'LOCK_TARGET' && pendingManualTargetSend.current) {
      pendingManualTargetSend.current = false;
      if (!result.ok) {
        addLog(`HEDEF KİLİTLENEMEDİ; İDA'YA GÖNDERİLMEDİ: ${result.message}`, 'ERROR');
        return;
      }

      addLog("HEDEF KİLİTLENDİ; İDA'YA GÖNDERİM BAŞLATILIYOR.", 'SUCCESS');
      if (!telemetryService.sendCommand('SEND_TARGET_TO_IDA')) {
        addLog("HEDEF İDA'YA GÖNDERİLEMEDİ: WebSocket bağlantısı yok.", 'ERROR');
      }
      return;
    }

    if (result.command === 'SEND_TARGET_TO_IDA') {
      addLog(
        result.status === 'acked'
          ? `HEDEF PIXHAWK'A YAZILDI; JETSON DOĞRULAMASI BEKLENİYOR: ${result.message}`
          : result.status === 'sent_unconfirmed'
          ? `HEDEF İDA'YA ONAYSIZ GÖNDERİLDİ: ${result.message}`
          : result.ok
            ? `HEDEF İDA'YA GÖNDERİLDİ: ${result.message}`
            : `HEDEF İDA'YA GÖNDERİLEMEDİ: ${result.message}`,
        getCommandLogType(result),
      );
      return;
    }

    if (result.command === 'RESET_EMERGENCY') {
      if (result.ok) {
        setIsEmergencyActive(false);
      }
      addLog(
        result.ok
          ? `EMERGENCY RESET BAŞARILI: ${result.message}`
          : `EMERGENCY RESET REDDEDİLDİ: ${result.message}`,
        result.ok ? 'SUCCESS' : 'ERROR',
      );
      return;
    }

    addLog(
      `${result.command || 'Komut'} [${result.status || 'unknown'}]: ${result.message || 'sonuç alındı'}`,
      getCommandLogType(result),
    );
  });

  useEffect(() => {
    telemetryService.connect({
      onTelemetry: handleTelemetry,
      onLog: handleBackendLog,
      onStatus: handleStatus,
      onCommandResult: handleCommandResult,
      onDebugSnapshot: handleDebugSnapshot,
      onVehicleTest: handleVehicleTest,
    });
    return () => telemetryService.disconnect();
  }, []);

  const sendCommand = useCallback((command, payload = {}) => {
    const sent = telemetryService.sendCommand(command, payload);
    if (!sent) {
      addLog(`KOMUT GÖNDERİLEMEDİ: ${command} - WebSocket bağlantısı yok`, 'ERROR');
    }
    return sent;
  }, [addLog]);

  const runPassiveTest = useCallback((test, expectations = null) => {
    if (!telemetryService.sendVehicleTest('start', {
      test_id: test.id, timeout_s: test.timeout_s, expectations,
    })) {
      setVehicleTestResult({ schema_version: 1, event: 'rejected', test_id: test.id, status: 'disconnected' });
      return false;
    }
    return true;
  }, []);

  const cancelPassiveTest = useCallback((runId) => (
    telemetryService.sendVehicleTest('cancel', { run_id: runId })
  ), []);

  const triggerEmergency = () => {
    if (isEmergencyActive) {
      if (sendCommand('EMERGENCY_STOP_ALL')) {
        addLog('Emergency komutu tekrar gönderildi; güvenlik kilidi aktif kalıyor.', 'ERROR');
      }
      return;
    }
    if (sendCommand('EMERGENCY_STOP_ALL')) {
      setIsEmergencyActive(true);
      addLog('ACİL DURDURMA KOMUTU GÖNDERİLDİ - backend güvenlik kilidi aktif.', 'ERROR');
    }
  };

  const resetEmergency = () => {
    if (sendCommand('RESET_EMERGENCY')) {
      addLog('Kontrollü emergency reset isteği gönderildi.', 'WARNING');
    }
  };

  const updateDroneColorManuel = (inputColor) => {
    const formattedColor = inputColor?.toUpperCase().trim();
    if (!formattedColor || formattedColor === 'AUTO') {
      addLog('Hedef kilidi uzaktan otomatik moda alınamaz.', 'WARNING');
      return;
    }
    pendingManualTargetSend.current = true;
    if (sendCommand('LOCK_TARGET', {
      color: formattedColor,
      source: 'MANUAL',
      confidence: 1.0,
    })) {
      addLog(`HEDEF KİLİTLEME İSTEĞİ: ${formattedColor}`, 'WARNING');
    } else {
      pendingManualTargetSend.current = false;
    }
  };

  const toggleArm = () => {
    if (isEmergencyActive) {
      addLog('HATA: Emergency kilidi aktifken ARM edilemez.', 'ERROR');
      return;
    }
    const isArmed = telemetry.ida.armed ?? telemetry.ida.mode !== 'DISARMED';
    sendCommand(isArmed ? 'DISARM_IDA' : 'ARM_IDA');
  };

  const startMission = () => {
    if (isEmergencyActive) {
      addLog('HATA: Emergency kilidi aktifken görev başlatılamaz.', 'ERROR');
      return;
    }
    if (!telemetry.system.mission_uploaded) {
      addLog('HATA: Backend üzerinde doğrulanmış görev upload yok, görev başlatılamadı.', 'ERROR');
      return;
    }
    sendCommand('START_IDA_MISSION');
  };

  const stopMission = () => {
    sendCommand('STOP_IDA_MISSION');
  };

  const uploadMission = () => {
    if (missionPoints.length === 0) {
      addLog('HATA: Yüklenecek görev listesi boş.', 'ERROR');
      return;
    }
    try {
      sendCommand('UPLOAD_IDA_MISSION', {
        waypoints: buildMissionWaypoints(missionPoints),
      });
    } catch (error) {
      addLog(`HATA: ${error.message}`, 'ERROR');
    }
  };

  const setMissionDefaultParkur = (parkur) => {
    const value = Number(parkur);
    if (![1, 2, 3].includes(value)) return;
    setMissionDefaultParkurState(value);
    addLog(`Yeni görev noktaları için parkur P${value} seçildi.`, 'INFO');
  };

  const addMissionPoint = (lat, lon, parkur = null) => {
    const selectedParkur = parkur === null ? missionDefaultParkur : Number(parkur);
    if (![1, 2, 3].includes(selectedParkur)) return;
    missionPointSequenceRef.current += 1;
    const newPoint = {
      id: missionPointSequenceRef.current, lat, lon, parkur: selectedParkur,
    };
    setMissionPoints(prev => [...prev, newPoint]);
    addLog(`Rota Planlayıcı: WP#${newPoint.id} P${selectedParkur} eklendi`, 'INFO');
  };

  const setMissionPointParkur = (id, parkur) => {
    const value = Number(parkur);
    if (![1, 2, 3].includes(value)) return;
    setMissionPoints((points) => points.map((point) => (
      point.id === id ? { ...point, parkur: value } : point
    )));
  };

  const clearMission = () => {
    setMissionPoints([]);
    missionPointSequenceRef.current = 0;
    addLog('Görev listesi temizlendi.', 'WARNING');
  };

  const toggleLogging = () => {
    const newState = !isLogging;
    setIsLogging(newState);
    addLog(newState ? 'Kara Kutu Kaydı BAŞLATILDI.' : 'Kara Kutu Kaydı DURDURULDU.', newState ? 'SUCCESS' : 'WARNING');
  };

  const downloadLogs = () => {
    addLog('Backend log dosyaları backend/logs klasörüne yazılıyor.', 'INFO');
  };

  // --- İHA KOMUTLARI ---
  // Hepsi sendCommand üzerinden gider; WebSocket auth + CommandGate zinciri korunur.
  // İHA komutları bilinçli olarak İDA görev kilidine tabi değildir: şartname §5.5.3.1
  // İHA'nın manuel kontrol edilebilmesine izin verir.

  const toggleArmIha = () => {
    if (isEmergencyActive) {
      addLog('HATA: Emergency kilidi aktifken İHA ARM edilemez.', 'ERROR');
      return;
    }
    const isArmed = Boolean(telemetry.iha.armed);
    sendCommand(isArmed ? 'DISARM_IHA' : 'ARM_IHA');
  };

  const setIhaMode = (mode) => {
    const normalized = String(mode || '').toUpperCase().trim();
    if (!normalized) {
      addLog('HATA: Geçersiz İHA modu.', 'ERROR');
      return;
    }
    sendCommand('SET_IHA_MODE', { mode: normalized });
  };

  const triggerIhaReturnHome = () => {
    if (sendCommand('SET_IHA_MODE', { mode: 'RTL' })) {
      addLog('İHA için RTL (eve dönüş) komutu gönderildi.', 'WARNING');
    }
  };

  // --- GEOFENCE (İHA UÇUŞ BÖLGESİ) ---

  const persistGeofence = useCallback((points) => {
    try {
      window.localStorage.setItem(GEOFENCE_STORAGE_KEY, JSON.stringify(points));
    } catch {
      addLog('UYARI: Uçuş bölgesi tarayıcı hafızasına kaydedilemedi.', 'WARNING');
    }
  }, [addLog]);

  const addGeofencePoint = (lat, lon) => {
    if (!Number.isFinite(lat) || !Number.isFinite(lon)) return;
    setGeofencePoints((prev) => {
      const next = [...prev, { lat, lon }];
      persistGeofence(next);
      addLog(`Uçuş bölgesi: ${next.length}. sınır noktası eklendi.`, 'INFO');
      return next;
    });
  };

  const undoGeofencePoint = () => {
    setGeofencePoints((prev) => {
      if (prev.length === 0) return prev;
      const next = prev.slice(0, -1);
      persistGeofence(next);
      addLog(`Uçuş bölgesi: son nokta silindi (${next.length} nokta kaldı).`, 'WARNING');
      return next;
    });
  };

  const clearGeofence = () => {
    setGeofencePoints([]);
    persistGeofence([]);
    previousGeofenceStateRef.current = null;
    addLog('Uçuş bölgesi tamamen temizlendi.', 'WARNING');
  };

  // Render sırasında saf hesap: UI üç durumu ayırt eder (null = sınır/konum yok).
  const isIhaInsideGeofence = isInsidePolygon(telemetry.iha.lat, telemetry.iha.lon, geofencePoints);

  return (
    <VehicleContext.Provider value={{
      telemetry, connectionStatus,
      triggerEmergency, isEmergencyActive, resetEmergency,
      toggleArm, startMission, stopMission, uploadMission, sendCommand,
      missionPoints, addMissionPoint, setMissionPointParkur, clearMission,
      missionDefaultParkur, setMissionDefaultParkur,
      isLogging, setIsLogging: toggleLogging, downloadLogs,
      systemLogs, addLog,
      updateDroneColorManuel,
      toggleArmIha, setIhaMode, triggerIhaReturnHome,
      geofencePoints, addGeofencePoint, undoGeofencePoint, clearGeofence,
      isIhaInsideGeofence,
      debugSnapshot, vehicleTests, vehicleTestResult, vehicleTestEvents,
      runPassiveTest, cancelPassiveTest,
    }}>
      {children}
    </VehicleContext.Provider>
  );
};
