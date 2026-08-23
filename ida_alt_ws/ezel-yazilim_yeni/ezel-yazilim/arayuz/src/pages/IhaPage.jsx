import { useState } from 'react';
import MapView from '@/features/MapSystem/components/MapView';
import IhaStatusPanel from '@/features/DroneControl/components/IhaStatusPanel';
import TargetDetectionPanel from '@/features/DroneControl/components/TargetDetectionPanel';
import IhaCommandPanel from '@/features/DroneControl/components/IhaCommandPanel';
import GeofenceEditor from '@/features/DroneControl/components/GeofenceEditor';
import ErrorBoundary from '@/shared/ui/ErrorBoundary';
import styles from './Pages.module.css';

/**
 * İHA OPERASYON SAYFASI
 *
 * Şartname §4.1 gereği bu ekranda kamera görüntüsü YOKTUR: İHA'dan yer tarafına
 * görüntü aktarımı ve YKİ'de görüntü işleme yasaktır. Gösterilen renk bilgisi,
 * İHA'nın kendi üzerinde ürettiği tespit sonucunun telemetri olarak iletilmiş halidir.
 */
const IhaPage = () => {
  const [drawingGeofence, setDrawingGeofence] = useState(false);

  const handleFlyToIha = () => window.dispatchEvent(new Event('FOCUS_IHA'));

  return (
    <div className={styles.ihaGrid}>
      <div className={styles.ihaSidebar}>
        <IhaStatusPanel />
        <TargetDetectionPanel />
        <IhaCommandPanel />
        <GeofenceEditor
          drawing={drawingGeofence}
          onToggleDrawing={() => setDrawingGeofence((prev) => !prev)}
        />
      </div>

      <div className={styles.ihaMapArea} style={{ position: 'relative' }}>
        <ErrorBoundary
          fallbackTitle="HARİTA HATASI"
          fallbackMessage="Harita modülü yüklenemedi. İHA telemetrisi ve komutları aktif kalmaya devam ediyor."
        >
          <MapView geofenceMode={drawingGeofence} />
        </ErrorBoundary>

        <div className={styles.ihaMapOverlay}>
          <button type="button" onClick={handleFlyToIha} className={styles.focusButton}>
            🎯 İHA BUL
          </button>
          <div
            className={styles.missionModeLabel}
            style={{
              margin: 0,
              position: 'relative',
              top: 'auto',
              right: 'auto',
              background: drawingGeofence ? '#2ecc71' : '#e67e22',
            }}
          >
            {drawingGeofence ? 'SINIR ÇİZİM MODU' : 'İHA OPERASYON'}
          </div>
        </div>
      </div>
    </div>
  );
};

export default IhaPage;
