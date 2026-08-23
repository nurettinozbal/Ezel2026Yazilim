import { useVehicle } from '@/context/useVehicle';
import { MIN_GEOFENCE_POINTS } from '../utils/geofence';
import { MapPin, Undo2, Trash2, ShieldCheck, ShieldAlert, ShieldQuestion } from 'lucide-react';
import styles from '../styles/DroneControl.module.css';

const GeofenceEditor = ({ drawing, onToggleDrawing }) => {
  const { geofencePoints, undoGeofencePoint, clearGeofence, isIhaInsideGeofence } = useVehicle();

  const pointCount = geofencePoints.length;
  const isComplete = pointCount >= MIN_GEOFENCE_POINTS;

  // Üç ayrı durum: sınır yok / içeride / dışarıda. null "bilinmiyor" demektir.
  let banner;
  if (!isComplete) {
    banner = {
      text: 'SINIR TANIMSIZ',
      detail: `En az ${MIN_GEOFENCE_POINTS} nokta gerekli (${pointCount} girildi)`,
      color: '#71717a',
      background: '#18181b',
      Icon: ShieldQuestion,
    };
  } else if (isIhaInsideGeofence === false) {
    banner = {
      text: 'BÖLGE DIŞI — PARKUR-3 RİSKİ',
      detail: 'Şartname §5.5.3.1: bölge dışı uçuşta Parkur-3 başarısız sayılır',
      color: '#ffffff',
      background: '#dc2626',
      Icon: ShieldAlert,
    };
  } else if (isIhaInsideGeofence === true) {
    banner = {
      text: 'BÖLGE İÇİNDE',
      detail: 'İHA tanımlı uçuş bölgesinde',
      color: '#2ecc71',
      background: '#052e16',
      Icon: ShieldCheck,
    };
  } else {
    banner = {
      text: 'İHA KONUMU YOK',
      detail: 'Geçerli GPS konumu gelmeden bölge kontrolü yapılamaz',
      color: '#f1c40f',
      background: '#1c1917',
      Icon: ShieldQuestion,
    };
  }

  const { Icon } = banner;

  return (
    <div className={styles.panel}>
      <div className={styles.panelTitle}>
        <MapPin size={14} /> UÇUŞ BÖLGESİ (GEOFENCE)
      </div>

      <div
        className={styles.geofenceBanner}
        style={{ backgroundColor: banner.background, color: banner.color, borderColor: banner.color }}
      >
        <Icon size={18} />
        <div>
          <div className={styles.geofenceBannerText}>{banner.text}</div>
          <div className={styles.geofenceBannerDetail}>{banner.detail}</div>
        </div>
      </div>

      <button
        type="button"
        onClick={onToggleDrawing}
        className={`${styles.commandButton} ${drawing ? styles.drawingActive : styles.drawingInactive}`}
      >
        <MapPin size={14} />
        {drawing ? 'ÇİZİMİ BİTİR' : 'HARİTAYA TIKLAYARAK SINIR EKLE'}
      </button>

      {drawing && (
        <div className={styles.detailNote}>
          Harita üzerinde sınır noktalarına sırayla tıklayın. Görev planlama tıklaması
          bu modda devre dışıdır.
        </div>
      )}

      <div className={styles.detailRow}>
        <span>Sınır noktası</span>
        <strong>{pointCount}</strong>
      </div>

      <div className={styles.geofenceActions}>
        <button
          type="button"
          onClick={undoGeofencePoint}
          disabled={pointCount === 0}
          className={styles.smallButton}
        >
          <Undo2 size={13} /> SON NOKTAYI SİL
        </button>
        <button
          type="button"
          onClick={clearGeofence}
          disabled={pointCount === 0}
          className={`${styles.smallButton} ${styles.dangerButton}`}
        >
          <Trash2 size={13} /> TEMİZLE
        </button>
      </div>
    </div>
  );
};

export default GeofenceEditor;
