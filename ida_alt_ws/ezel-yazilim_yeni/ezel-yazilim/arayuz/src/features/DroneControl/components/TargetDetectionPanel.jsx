import { useVehicle } from '@/context/useVehicle';
import { Target, Lock, Radio, TriangleAlert } from 'lucide-react';
import styles from '../styles/DroneControl.module.css';

// Backend VALID_COLORS ile birebir; şartname hedef renkleri DSB olduğu için
// resmi renkler açıklandığında backend target_manager.py ile birlikte güncellenir.
const COLOR_HEX_MAP = {
  KIRMIZI: '#ef4444',
  YEŞİL: '#2ecc71',
  SİYAH: '#18181b',
};

const DELIVERY_LABELS = {
  not_sent: 'GÖNDERİLMEDİ',
  pending_send: 'GÖNDERİM BEKLİYOR',
  acked: "PIXHAWK'A YAZILDI · JETSON BEKLENİYOR",
  state_verified: 'JETSON / OTONOMİ DOĞRULADI',
  timeout: 'GERİ OKUMA ZAMAN AŞIMI',
  sent_unconfirmed: 'DOĞRULANMADI',
  rejected: 'REDDEDİLDİ',
  failed: 'BAŞARISIZ',
};

const isWarningDelivery = (status) => ['sent_unconfirmed', 'pending_send', 'acked'].includes(status);
const isErrorDelivery = (status) => ['rejected', 'failed', 'timeout'].includes(status);

const TargetDetectionPanel = () => {
  const { telemetry } = useVehicle();
  const { iha, system } = telemetry;

  const color = iha.detected_color;
  const hasDetection = Boolean(color) && color !== 'BEKLENİYOR';
  const isAutonomous = system.target_source === 'IHA';

  const deliveryStatus = system.target_delivery_status;
  const deliveryLabel = DELIVERY_LABELS[deliveryStatus] || String(deliveryStatus || '—').toUpperCase();
  const deliveryColor = isErrorDelivery(deliveryStatus)
    ? '#e74c3c'
    : (isWarningDelivery(deliveryStatus) ? '#f1c40f' : '#71717a');

  const lockedAt = system.target_locked_at
    ? new Date(system.target_locked_at * 1000).toLocaleTimeString('tr-TR')
    : null;

  return (
    <div className={styles.panel}>
      <div className={styles.panelTitle}>
        <Target size={14} /> HEDEF PLAKA TESPİTİ
      </div>

      <div
        className={styles.colorBlock}
        style={{
          backgroundColor: hasDetection ? (COLOR_HEX_MAP[color] || '#3f3f46') : '#18181b',
          borderColor: hasDetection ? (COLOR_HEX_MAP[color] || '#52525b') : '#3f3f46',
        }}
      >
        <span className={styles.colorName}>{hasDetection ? color : 'BEKLENİYOR'}</span>
      </div>

      <div className={styles.sourceBadgeRow}>
        <span
          className={styles.sourceBadge}
          style={{
            backgroundColor: isAutonomous ? '#065f46' : '#3f3f46',
            borderColor: isAutonomous ? '#2ecc71' : '#71717a',
            color: isAutonomous ? '#2ecc71' : '#d4d4d8',
          }}
        >
          <Radio size={11} /> {isAutonomous ? 'İHA (OTONOM)' : 'MANUEL'}
        </span>
        {system.target_locked && (
          <span className={styles.lockBadge}>
            <Lock size={11} /> KİLİTLİ{lockedAt ? ` · ${lockedAt}` : ''}
          </span>
        )}
      </div>

      <div className={styles.detailRow}>
        <span>İDA'ya gönderim</span>
        <strong style={{ color: deliveryColor }}>{deliveryLabel}</strong>
      </div>

      {['sent_unconfirmed', 'acked'].includes(deliveryStatus) && (
        <div className={styles.warningNote}>
          <TriangleAlert size={12} />
          <span>
            Pixhawk yazımı tek başına teslim kanıtı değildir. Jetson SCR_USER4'ü
            okuyup TGT_ACK döndürene kadar görev başlatılmamalıdır.
          </span>
        </div>
      )}

      {system.target_delivery_message && (
        <div className={styles.detailNote}>{system.target_delivery_message}</div>
      )}

      {system.mission_started && (
        <div className={styles.warningNote}>
          <Lock size={12} />
          <span>
            İDA göreve başladı — şartname §5.5.3.1 gereği hedef bilgisi artık
            değiştirilemez ve aktarılamaz.
          </span>
        </div>
      )}
    </div>
  );
};

export default TargetDetectionPanel;
