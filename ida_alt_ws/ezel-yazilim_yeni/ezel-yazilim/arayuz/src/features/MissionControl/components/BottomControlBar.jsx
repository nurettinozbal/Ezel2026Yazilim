import { useVehicle } from '@/context/useVehicle';
import { Play, Square, Activity, Signal, Satellite, ShieldCheck, Power, Anchor } from 'lucide-react';
import styles from '../styles/BottomControlBar.module.css'; 

const BottomControlBar = () => {
  const { 
    telemetry, 
    isLogging, setIsLogging, downloadLogs, 
    toggleArm, startMission, stopMission
  } = useVehicle();
  
  const { system, ida } = telemetry;
  const isArmed = ida.armed ?? ida.mode !== 'DISARMED';
  const missionReady = Boolean(system.mission_uploaded);
  const missionStarted = Boolean(system.mission_started);

  const getRssiColor = (dbm) => {
    if (dbm > -60) return '#2ecc71'; 
    if (dbm > -75) return '#f1c40f'; 
    return '#e74c3c'; 
  };

  const getHdopColor = (hdop) => {
    if (hdop < 1.2) return '#2ecc71'; 
    if (hdop < 2.0) return '#f1c40f';
    return '#e74c3c';
  };

  // --- 6S BATARYA YÜZDE HESAPLAYICI ---
  const getBatteryPercent = (voltage) => {
    if (!voltage || voltage <= 0) return 0;
    const maxV = 25.2; // 6S Tam Dolu (4.2V x 6)
    const minV = 21.0; // 6S Güvenli Boş Sınır (3.5V x 6)
    let pct = ((voltage - minV) / (maxV - minV)) * 100;
    return Math.max(0, Math.min(100, Math.round(pct)));
  };

  const batteryPct = (ida.battery_percent && ida.battery_percent > 0) 
                     ? ida.battery_percent 
                     : getBatteryPercent(ida.voltage);

  // --- KONTROL MODU İÇİN DİNAMİK RENK YÖNETİMİ ---
  let modeTextColor = '#fff';
  let modeTextShadow = 'none';
  if (ida.mode === 'AUTO') { 
    modeTextColor = '#3498db'; 
    modeTextShadow = '0 0 8px rgba(52, 152, 219, 0.6)'; 
  } else if (isArmed) { 
    modeTextColor = '#e74c3c'; 
    modeTextShadow = '0 0 8px rgba(231, 76, 60, 0.6)'; 
  } else { 
    modeTextColor = '#f1c40f'; 
  }

  return (
    <div className={styles.container}>
      
      {/* SOL: SİSTEM SAĞLIĞI VE BATARYA */}
      <div className={styles.healthGroup} style={{ display: 'flex', alignItems: 'center', gap: '10px' }}>
        
        {/* YENİ: 6S İÇİN ŞIK BATARYA GÖSTERGESİ */}
        <div style={{ 
          display: 'flex', 
          alignItems: 'center', 
          gap: '6px',
          border: '1px solid #e74c3c', 
          backgroundColor: '#16161a',
          padding: '4px 8px', 
          borderRadius: '4px' 
        }}>
          <span style={{ color: '#fff', fontWeight: 'bold', fontSize: '13px', letterSpacing: '0.5px' }}>
            Batarya: {batteryPct}% - {ida.voltage ? ida.voltage.toFixed(1) : '0.0'}V (6S)
          </span>
        </div>
        
        <div className={styles.healthItem} style={{ color: getRssiColor(system.rssi) }}>
          <Signal size={16} />
          <span>RFD: {system.rssi} dBm</span>
        </div>
        <div className={styles.healthItem} style={{ color: getHdopColor(system.hdop) }}>
          <Satellite size={16} />
          <span>GPS: {system.gps_sats} Sats (HDOP: {system.hdop})</span>
        </div>
        <div className={styles.failsafeItem}>
          <ShieldCheck size={16} />
          <span>FS: {system.failsafe_status}</span>
        </div>
      </div>

      {/* ORTA: KOMUTLAR VE MOD DURUMU */}
      <div className={styles.commandGroup} style={{ display: 'flex', alignItems: 'center', gap: '15px' }}>
        
        {/* KONTROL MODU GÖSTERGESİ (Sol Köşeye Yaslı) */}
        <div style={{ 
          display: 'flex', 
          alignItems: 'center', 
          backgroundColor: '#16161a', 
          border: '1px solid #333', 
          borderRadius: '6px', 
          padding: '4px 12px', 
          gap: '10px',
          boxShadow: 'inset 0 0 10px rgba(0,0,0,0.5)',
        }}>
          <div style={{ backgroundColor: '#2d2d2d', padding: '6px', borderRadius: '4px', display: 'flex', alignItems: 'center', justifyContent: 'center' }}>
            <Anchor size={16} color="#f1c40f" />
          </div>
          <div style={{ display: 'flex', flexDirection: 'column', justifyContent: 'center', textAlign: 'left' }}>
            <span style={{ fontSize: '9px', color: '#888', fontWeight: 'bold', letterSpacing: '1px', marginBottom: '-2px' }}>
              KONTROL MODU
            </span>
            <span style={{ 
              fontSize: '14px', 
              color: modeTextColor,
              fontWeight: '900', 
              letterSpacing: '1.5px', 
              textShadow: modeTextShadow 
            }}>
              {ida.mode || 'DISARMED'}
            </span>
          </div>
        </div>
        
        {/* ARM / DISARM BUTTON */}
        <button 
          onClick={toggleArm}
          className={`${styles.commandButton} ${isArmed ? styles.armButtonActive : styles.armButtonInactive}`}
        >
          <Power size={14} />
          {isArmed ? 'DISARM (KAPAT)' : 'ARM (AÇ)'}
        </button>
        
        {/* GÖREV BAŞLAT BUTTON */}
        <button 
          onClick={startMission}
          disabled={missionStarted || !isArmed || !missionReady}
          title={missionReady ? 'Yüklenmiş görev hazır' : 'Önce görevi İDA’ya yükleyin'}
          className={`${styles.commandButton} ${(!missionStarted && isArmed && missionReady) ? styles.missionButtonActive : styles.missionButtonInactive}`}
        >
          <Play size={14} /> GÖREV BAŞLAT
        </button>

        {/* STOP, timeout/restart nedeniyle mission_started bayrağı kaçsa bile
            daima erişilebilir güvenli HOLD + Jetson ACK çıkışıdır. */}
        <button
          onClick={stopMission}
          title="Pixhawk HOLD ve Jetson SCR_USER6 STOP_ACK doğrulaması"
          className={`${styles.commandButton} ${styles.missionButtonActive}`}
        >
          <Square size={14} /> GÖREV DURDUR
        </button>
      </div>

      {/* SAĞ: LOG VE DURUM */}
      <div className={styles.logGroup}>
        <Activity size={16} color="#2ecc71" className={styles.pulseIcon} />
        
        <button 
          onClick={setIsLogging} 
          className={styles.recButton}
          style={{ background: isLogging ? '#ef4444' : '#27272a' }} 
        >
          {isLogging ? 'REC ●' : 'KAYIT YOK'}
        </button>
        
        <button onClick={downloadLogs} className={styles.csvButton}>
          CSV
        </button>
      </div>
    </div>
  );
};

export default BottomControlBar;
