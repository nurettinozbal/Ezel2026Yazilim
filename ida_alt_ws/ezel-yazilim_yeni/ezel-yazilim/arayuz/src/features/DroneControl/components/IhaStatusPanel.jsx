import { useVehicle } from '@/context/useVehicle';
import InfoCard from '@/features/TelemetryPanel/components/InfoCard';
import { Activity, Battery, Navigation, Satellite, Signal, Anchor, Home } from 'lucide-react';
import styles from '../styles/DroneControl.module.css';

const formatCoord = (value) => (Number.isFinite(value) && value !== 0 ? value.toFixed(6) : '—');

const heartbeatAgeSeconds = (lastHeartbeat) => {
  if (!lastHeartbeat) return null;
  return Math.max(0, Math.round(Date.now() / 1000 - lastHeartbeat));
};

const IhaStatusPanel = () => {
  const { telemetry } = useVehicle();
  const { iha, system } = telemetry;

  const isConnected = system.iha_link === 'CONNECTED';
  const age = heartbeatAgeSeconds(iha.last_heartbeat);

  const gpsColor = iha.gps_sats >= 8 ? '#2ecc71' : (iha.gps_sats > 0 ? '#f1c40f' : '#e74c3c');
  const batteryColor = iha.battery >= 50 ? '#2ecc71' : (iha.battery >= 25 ? '#f1c40f' : '#e74c3c');

  return (
    <div className={styles.panel}>
      <div className={styles.panelTitle}>İHA DURUMU</div>

      <div className={styles.linkRow}>
        <span
          className={styles.linkDot}
          style={{ backgroundColor: isConnected ? '#2ecc71' : '#e74c3c' }}
        />
        <span className={styles.linkText}>
          BAĞLANTI: <strong style={{ color: isConnected ? '#2ecc71' : '#e74c3c' }}>
            {isConnected ? 'AKTİF' : 'YOK'}
          </strong>
        </span>
        <span className={styles.linkAge}>
          {age === null ? 'heartbeat yok' : `${age} sn önce`}
        </span>
      </div>

      <InfoCard
        title="Uçuş Modu"
        value={iha.mode || 'BİLİNMİYOR'}
        unit={iha.armed ? '(ARMED)' : '(DISARMED)'}
        icon={Anchor}
        color={iha.armed ? '#e74c3c' : '#f1c40f'}
      />
      <InfoCard title="İrtifa (AGL)" value={iha.alt} unit="m" icon={Activity} color="#e67e22" />
      <InfoCard title="Batarya" value={iha.battery} unit="%" icon={Battery} color={batteryColor} />
      <InfoCard title="Voltaj" value={iha.voltage} unit="V" icon={Battery} color="#f1c40f" />
      <InfoCard
        title="GPS"
        value={iha.gps_sats}
        unit={`uydu (HDOP ${iha.hdop})`}
        icon={Satellite}
        color={gpsColor}
      />
      <InfoCard title="Telemetri Sinyali" value={iha.rssi} unit="dBm" icon={Signal} color="#3498db" />

      <div className={styles.coordBox}>
        <div className={styles.coordRow}>
          <Navigation size={12} />
          <span>Konum: {formatCoord(iha.lat)}, {formatCoord(iha.lon)}</span>
        </div>
        <div className={styles.coordRow}>
          <Home size={12} />
          <span>
            Kalkış: {iha.home_locked
              ? `${formatCoord(iha.home_lat)}, ${formatCoord(iha.home_lon)}`
              : 'kilitlenmedi'}
          </span>
        </div>
        {iha.return_home_status && iha.return_home_status !== 'idle' && (
          <div className={styles.coordRow} style={{ color: '#f1c40f' }}>
            <span>Dönüş durumu: {iha.return_home_status}</span>
          </div>
        )}
      </div>
    </div>
  );
};

export default IhaStatusPanel;
