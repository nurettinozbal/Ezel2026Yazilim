import { useState, useRef, useEffect } from 'react';
import { useVehicle } from '@/context/useVehicle';
import { Upload, Trash2 } from 'lucide-react';
import styles from '../styles/MissionPlanner.module.css'; 

// --- DİNAMİK YÖN GÖSTERGELİ DEVİR SAATİ BİLEŞENİ ---
const RpmGauge = ({ title, rpm, pct, maxRpm = 5000 }) => {
  const hasRpm = Number.isFinite(rpm);
  const motorPct = Number.isFinite(pct) ? pct : 0;
  const isReverse = motorPct < 0; 
  const percentage = hasRpm
    ? Math.max(0, Math.min(Math.abs(rpm) / maxRpm, 1))
    : Math.max(0, Math.min(Math.abs(motorPct) / 100, 1));
  const rotation = percentage * 180 - 90; 
  
  const gaugeColor = isReverse ? '#e67e22' : (percentage > 0.8 ? '#e74c3c' : '#3498db');
  const directionText = isReverse ? 'GERİ' : (percentage > 0.01 ? 'İLERİ' : 'NÖTR');
  const directionColor = isReverse ? '#e67e22' : (percentage > 0.01 ? '#2ecc71' : '#666');

  return (
    <div style={{ textAlign: 'center', width: '90px', margin: '0' }}>
      <svg viewBox="0 0 100 55" style={{ overflow: 'visible' }}>
        <path d="M 10 50 A 40 40 0 0 1 90 50" fill="none" stroke="#222" strokeWidth="8" strokeLinecap="round" />
        <path d="M 10 50 A 40 40 0 0 1 90 50" fill="none" 
              stroke={gaugeColor} 
              strokeWidth="8" strokeLinecap="round" 
              strokeDasharray="125" 
              strokeDashoffset={125 - (125 * percentage)} 
              style={{ transition: 'stroke-dashoffset 0.2s ease-out' }} />
        <line x1="50" y1="50" x2="50" y2="15" stroke="#fff" strokeWidth="2" 
              transform={`rotate(${rotation} 50 50)`} 
              style={{ transition: 'transform 0.2s cubic-bezier(0.4, 2, 0.4, 0.5)' }} />
        <circle cx="50" cy="50" r="4" fill={gaugeColor} />
      </svg>
      <div style={{ color: '#fff', fontSize: '13px', fontWeight: 'bold', marginTop: '5px' }}>
        {hasRpm ? (isReverse ? `-${Math.abs(rpm)}` : rpm) : `${Math.round(motorPct)}%`}
      </div>
      <div style={{ color: directionColor, fontSize: '9px', fontWeight: 'bold', letterSpacing: '0.5px', marginBottom: '2px', transition: 'color 0.2s' }}>
        {directionText}
      </div>
      <div style={{ color: '#888', fontSize: '9px', letterSpacing: '1px' }}>{title}</div>
    </div>
  );
};
// -----------------------------------------------------------------

const MissionPlanner = () => {
  const {
    missionPoints, addMissionPoint, setMissionPointParkur, clearMission,
    missionDefaultParkur, setMissionDefaultParkur,
    uploadMission, telemetry, sendCommand,
  } = useVehicle();
  const { system, ida } = telemetry || { system: {}, ida: {} };

  const [lat, setLat] = useState('');
  const [lon, setLon] = useState('');
  const [isUploading, setIsUploading] = useState(false);

  // --- LOG TERMİNALİ İÇİN OTOMATİK KAYDIRMA REFERANSI ---
  const terminalRef = useRef(null);

  useEffect(() => {
    if (terminalRef.current) {
      terminalRef.current.scrollTop = terminalRef.current.scrollHeight;
    }
  }, [system?.logs]);
  // ------------------------------------------------------------

  const sats = ida?.satellites || ida?.sats || ida?.satellites_visible || 0;
  const fixType = ida?.fix_type || 0;
  const pixhawkWaypointCount = system?.mission_waypoint_count || 0;

  let gpsColor = '#e74c3c';
  let gpsText = 'BAĞLANTI YOK';

  if (fixType >= 3) {
    gpsColor = '#2ecc71'; 
    gpsText = '3D FIX';
  } else if (fixType === 2) {
    gpsColor = '#3498db'; 
    gpsText = '2D FIX';
  } else if (fixType === 1 || sats > 0) {
    gpsColor = '#f1c40f'; 
    gpsText = 'ARANIYOR...';
  }

  const handleAdd = () => {
    if (lat && lon) {
      addMissionPoint(parseFloat(lat), parseFloat(lon));
      setLat(''); setLon('');
    }
  };

  const handleUploadClick = () => {
    if (missionPoints.length === 0) return;
    setIsUploading(true);
    uploadMission();
    setTimeout(() => {
      setIsUploading(false);
    }, 1500);
  };

  const handleForceClear = () => {
    clearMission();
    sendCommand("CLEAR_IDA_MISSION");
  };

  return (
    <div 
      className={styles.container} 
      style={{ 
        width: '100%',
        /* ZORUNLU KISITLAMA: Ekranın yüksekliğinden alt ve üst barların payını (~140px) çıkarıyoruz */
        maxHeight: 'calc(100vh - 140px)', 
        height: '100%', 
        overflowY: 'auto', /* İçerik sınırları aşınca kaydırma çubuğunu açar */
        overflowX: 'hidden',
        display: 'flex', 
        flexDirection: 'column', 
        paddingRight: '8px', 
        paddingBottom: '20px' 
      }}
    >
      <h3 className={styles.title} style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', flexShrink: 0 }}>
        <span>GÖREV PLANLAMA</span>
        <span style={{
          fontSize: '10px',
          backgroundColor: pixhawkWaypointCount > 0 ? '#3498db' : '#333',
          color: pixhawkWaypointCount > 0 ? '#fff' : '#888',
          padding: '3px 8px',
          borderRadius: '12px',
          fontWeight: 'bold',
          letterSpacing: '0.5px',
          border: `1px solid ${pixhawkWaypointCount > 0 ? '#2980b9' : '#444'}`
        }}>
          PİXHAWK: {pixhawkWaypointCount > 0 ? `${pixhawkWaypointCount} NOKTA YÜKLÜ` : 'HAFIZA BOŞ'}
        </span>
      </h3>
      
      <div className={styles.inputGroup} style={{ flexShrink: 0 }}>
        <input 
          type="number" placeholder="Hedef Enlem" value={lat} onChange={e => setLat(e.target.value)}
          className={styles.input}
        />
        <input 
          type="number" placeholder="Hedef Boylam" value={lon} onChange={e => setLon(e.target.value)}
          className={styles.input}
        />
      </div>

      <div style={{
        display: 'flex', alignItems: 'center', justifyContent: 'space-between',
        gap: '10px', marginTop: '8px', padding: '8px',
        background: '#0a0a0a', border: '1px solid #333', borderRadius: '4px',
        flexShrink: 0,
      }}>
        <div>
          <div style={{ color: '#fff', fontSize: '11px', fontWeight: 'bold' }}>YENİ NOKTA PARKURU</div>
          <div style={{ color: '#888', fontSize: '9px' }}>
            Harita tıklaması ve koordinat ekleme bu parkuru kullanır. İlk HOME daima P1'dir.
          </div>
        </div>
        <select
          aria-label="Yeni nokta parkuru"
          value={missionDefaultParkur}
          onChange={(event) => setMissionDefaultParkur(Number(event.target.value))}
          style={{ background: '#111', color: '#fff', padding: '6px 10px', border: '1px solid #555' }}
        >
          <option value={1}>P1 — Koridor</option>
          <option value={2}>P2 — Engel</option>
          <option value={3}>P3 — Hedef</option>
        </select>
      </div>

      <button onClick={handleAdd} className={styles.addButton} style={{ flexShrink: 0 }}>
        Koordinat verisi ekle
      </button>

      <div className={styles.pointList} style={{ marginTop: '15px', flexShrink: 0, overflowY: 'auto', maxHeight: '150px' }}>
        {missionPoints.map((p, index) => (
          <div key={`${p.id || index}-${index}`} className={styles.pointItem}>
            <span>#{index + 1}: {p.lat}, {p.lon}</span>
            <select
              aria-label={`WP ${index + 1} parkur`}
              value={p.parkur ?? 1}
              onChange={(event) => setMissionPointParkur(p.id, Number(event.target.value))}
              style={{ marginLeft: '8px', background: '#111', color: '#fff' }}
            >
              <option value={1}>P1</option>
              <option value={2}>P2</option>
              <option value={3}>P3</option>
            </select>
          </div>
        ))}
        {missionPoints.length === 0 && <span style={{color: '#444', fontSize: '10px'}}>Liste Boş</span>}
      </div>

      <div className={styles.actionButtons} style={{ flexShrink: 0 }}>
        <button 
          onClick={handleUploadClick} 
          className={styles.uploadButton}
          disabled={isUploading || missionPoints.length === 0}
          style={{ 
            opacity: (isUploading || missionPoints.length === 0) ? 0.5 : 1,
            cursor: (isUploading || missionPoints.length === 0) ? 'not-allowed' : 'pointer'
          }}
        >
          <Upload size={14} /> {isUploading ? 'GÖREVİ YÜKLENİYOR...' : 'GÖREV YÜKLE'}
        </button>
        
        <button 
          onClick={handleForceClear} 
          className={styles.clearButton} 
          disabled={isUploading}
        >
          <Trash2 size={14} />
        </button>
      </div>

      {/* --- SİSTEM LOGLARI TERMİNALİ --- */}
      <div 
        ref={terminalRef} 
        style={{
          backgroundColor: '#0a0a0a', 
          border: '1px solid #333', 
          borderRadius: '4px', 
          padding: '10px', 
          marginTop: '15px', 
          height: '110px', 
          overflowY: 'auto', 
          scrollBehavior: 'smooth', 
          display: 'flex',
          flexDirection: 'column',
          gap: '5px',
          flexShrink: 0
        }}
      >
        <div style={{ fontSize: '11px', color: '#888', marginBottom: '5px', borderBottom: '1px solid #333', paddingBottom: '3px', position: 'sticky', top: 0, backgroundColor: '#0a0a0a' }}>
          &gt;_ SİSTEM LOGLARI &amp; TERMİNAL
        </div>
        {system?.logs?.map((log, index) => {
          let displayText = log;
          
          if (missionPoints.length === 0 && log.includes('GÖREV BAŞARIYLA YÜKLENDİ!')) {
            displayText = log.replace('GÖREV BAŞARIYLA YÜKLENDİ!', 'Hafızayı silme görevini başarıyla tamamladı!');
          }

          let logColor = '#f1c40f'; 
          if (displayText.includes('SİLİNDİ') || displayText.includes('silme')) logColor = '#e74c3c'; 
          else if (displayText.includes('BAŞARILI') || displayText.includes('başarıyla')) logColor = '#2ecc71'; 
          else if (displayText.includes('GÖREV')) logColor = '#3498db'; 

          return (
            <div key={index} style={{ 
              color: logColor,
              fontFamily: 'monospace', 
              fontSize: '11px' 
            }}>
              {displayText}
            </div>
          );
        })}
      </div>

      <div style={{
        display: 'flex',
        justifyContent: 'space-between',
        alignItems: 'center',
        backgroundColor: '#0a0a0a',
        border: `1px solid ${gpsColor}40`, 
        borderRadius: '4px',
        padding: '8px 12px',
        marginTop: '15px',
        boxShadow: `0 0 10px ${gpsColor}20`,
        flexShrink: 0
      }}>
        <div style={{ display: 'flex', alignItems: 'center', gap: '8px' }}>
          <div style={{
            width: '10px', height: '10px', borderRadius: '50%',
            backgroundColor: gpsColor,
            boxShadow: `0 0 8px ${gpsColor}`
          }}></div>
          <span style={{ fontSize: '11px', fontWeight: 'bold', color: '#fff', letterSpacing: '0.5px' }}>
            GPS DURUM: <span style={{ color: gpsColor }}>{gpsText}</span>
          </span>
        </div>
        <div style={{ fontSize: '11px', fontWeight: 'bold', color: '#ccc' }}>
          UYDU: <span style={{ color: sats >= 8 ? '#2ecc71' : (sats > 0 ? '#f1c40f' : '#e74c3c'), fontSize: '13px' }}>{sats}</span>
        </div>
      </div>

      {/* --- ŞARTNAME UYUMLU: GERÇEK VE SETPOINT GÖSTERGELERİ --- */}
      <div style={{ display: 'flex', gap: '10px', marginTop: '15px', flexShrink: 0 }}>
        
        {/* Şartname 1: Gerçek hız, hız isteği (setpoint) */}
        <div style={{ flex: 1, backgroundColor: '#0a0a0a', border: '1px solid #333', borderRadius: '4px', padding: '8px 12px', display: 'flex', flexDirection: 'column', justifyContent: 'center' }}>
          <div style={{ fontSize: '9px', color: '#888', fontWeight: 'bold', letterSpacing: '0.5px', marginBottom: '6px', textAlign: 'center', borderBottom: '1px solid #222', paddingBottom: '4px' }}>
            GERÇEK HIZ & HIZ İSTEĞİ (m/s)
          </div>
          <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center' }}>
            <div style={{ textAlign: 'center' }}>
              <div style={{ fontSize: '8px', color: '#3498db' }}>HIZ İSTEĞİ (SETPOINT)</div>
              <div style={{ fontSize: '13px', fontWeight: 'bold', color: '#3498db' }}>{ida?.target_speed || '0.0'}</div>
            </div>
            <div style={{ fontSize: '14px', color: '#444' }}>|</div>
            <div style={{ textAlign: 'center' }}>
              <div style={{ fontSize: '8px', color: '#e74c3c' }}>GERÇEK HIZ</div>
              <div style={{ fontSize: '13px', fontWeight: 'bold', color: '#e74c3c' }}>{ida?.speed || '0.0'}</div>
            </div>
          </div>
        </div>

        {/* Şartname 2: Gerçek heading/yaw açısı, heading/yaw açısı isteği (setpoint) */}
        <div style={{ flex: 1, backgroundColor: '#0a0a0a', border: '1px solid #333', borderRadius: '4px', padding: '8px 12px', display: 'flex', flexDirection: 'column', justifyContent: 'center' }}>
          <div style={{ fontSize: '9px', color: '#888', fontWeight: 'bold', letterSpacing: '0.5px', marginBottom: '6px', textAlign: 'center', borderBottom: '1px solid #222', paddingBottom: '4px' }}>
            GERÇEK & İSTENEN HEADING (°)
          </div>
          <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center' }}>
            <div style={{ textAlign: 'center' }}>
              <div style={{ fontSize: '8px', color: '#2ecc71' }}>HEADING İSTEĞİ (SETPOINT)</div>
              <div style={{ fontSize: '13px', fontWeight: 'bold', color: '#2ecc71' }}>{ida?.target_heading || '0'}</div>
            </div>
            <div style={{ fontSize: '14px', color: '#444' }}>|</div>
            <div style={{ textAlign: 'center' }}>
              <div style={{ fontSize: '8px', color: '#f1c40f' }}>GERÇEK HEADING</div>
              <div style={{ fontSize: '13px', fontWeight: 'bold', color: '#f1c40f' }}>{ida?.heading || '0'}</div>
            </div>
          </div>
        </div>

      </div>

      {/* Şartname 3: Thrusterlardan kuvvet isteği */}
      <div style={{ marginTop: '15px', flexShrink: 0, backgroundColor: '#0a0a0a', border: '1px solid #333', borderRadius: '4px', padding: '10px' }}>
        <div style={{ fontSize: '9px', color: '#888', fontWeight: 'bold', letterSpacing: '0.5px', marginBottom: '10px', textAlign: 'center', borderBottom: '1px solid #222', paddingBottom: '4px' }}>
          THRUSTERLARDAN KUVVET İSTEĞİ
        </div>
        <div style={{ display: 'flex', justifyContent: 'center', gap: '30px' }}>
          <RpmGauge title="SOL THRUSTER" rpm={ida?.rpm_left || 0} pct={ida?.motor_left_pct || 0} />
          <RpmGauge title="SAĞ THRUSTER" rpm={ida?.rpm_right || 0} pct={ida?.motor_right_pct || 0} />
        </div>
      </div>

    </div>
  );
};

export default MissionPlanner;
