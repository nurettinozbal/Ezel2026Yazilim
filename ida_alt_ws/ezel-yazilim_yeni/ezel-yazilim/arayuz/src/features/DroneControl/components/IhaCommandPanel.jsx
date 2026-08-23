import { useState } from 'react';
import { useVehicle } from '@/context/useVehicle';
import ConfirmModal from '@/shared/ui/ConfirmModal/ConfirmModal';
import { Power, Home, Plane } from 'lucide-react';
import styles from '../styles/DroneControl.module.css';

// Backend command_gate.IHA_MODES ile birebir.
const IHA_MODES = ['LOITER', 'GUIDED', 'AUTO', 'RTL'];
// Emergency aktifken backend yalnız bunlara izin verir (command_gate._allow).
const EMERGENCY_ALLOWED_MODES = new Set(['LOITER', 'RTL']);

const IhaCommandPanel = () => {
  const { telemetry, isEmergencyActive, toggleArmIha, setIhaMode, triggerIhaReturnHome } = useVehicle();
  const { iha, system } = telemetry;
  const [pendingAction, setPendingAction] = useState(null);

  const isConnected = system.iha_link === 'CONNECTED';
  const isArmed = Boolean(iha.armed);

  const isModeDisabled = (mode) => (
    !isConnected || (isEmergencyActive && !EMERGENCY_ALLOWED_MODES.has(mode))
  );

  const handleConfirm = () => {
    const action = pendingAction;
    setPendingAction(null);
    if (!action) return;
    action.run();
  };

  return (
    <div className={styles.panel}>
      <div className={styles.panelTitle}>
        <Plane size={14} /> İHA KOMUTA
      </div>

      {!isConnected && (
        <div className={styles.disabledNote}>
          İHA bağlantısı yok — komutlar backend tarafından reddedilir.
        </div>
      )}

      <button
        type="button"
        disabled={!isConnected || isEmergencyActive}
        onClick={() => setPendingAction({
          title: isArmed ? 'İHA DISARM' : 'İHA ARM',
          message: isArmed
            ? 'İHA disarm edilecek. Havadayken disarm aracın düşmesine yol açar. Onaylıyor musunuz?'
            : 'İHA arm edilecek. Pervanelerin çevresinin güvenli olduğundan emin olun. Onaylıyor musunuz?',
          confirmLabel: isArmed ? 'DISARM ET' : 'ARM ET',
          run: toggleArmIha,
        })}
        className={`${styles.commandButton} ${isArmed ? styles.armActive : styles.armInactive}`}
      >
        <Power size={14} /> {isArmed ? 'DISARM (KAPAT)' : 'ARM (AÇ)'}
      </button>

      <div className={styles.modeLabel}>UÇUŞ MODU</div>
      <div className={styles.modeGrid}>
        {IHA_MODES.map((mode) => {
          const disabled = isModeDisabled(mode);
          const isCurrent = String(iha.mode || '').toUpperCase() === mode;
          return (
            <button
              key={mode}
              type="button"
              disabled={disabled}
              title={disabled && isEmergencyActive
                ? 'Emergency aktifken yalnız LOITER/RTL seçilebilir'
                : undefined}
              onClick={() => setPendingAction({
                title: `İHA MOD: ${mode}`,
                message: `İHA uçuş modu ${mode} olarak değiştirilecek. Onaylıyor musunuz?`,
                confirmLabel: 'MODU DEĞİŞTİR',
                run: () => setIhaMode(mode),
              })}
              className={`${styles.modeButton} ${isCurrent ? styles.modeButtonActive : ''}`}
            >
              {mode}
            </button>
          );
        })}
      </div>

      <button
        type="button"
        disabled={!isConnected}
        onClick={() => setPendingAction({
          title: 'İHA EVE DÖNÜŞ (RTL)',
          message: 'İHA kalkış noktasına otomatik dönecek. Onaylıyor musunuz?',
          confirmLabel: 'RTL GÖNDER',
          run: triggerIhaReturnHome,
        })}
        className={styles.rtlButton}
      >
        <Home size={14} /> ACİL: EVE DÖNÜŞ (RTL)
      </button>

      <ConfirmModal
        isOpen={Boolean(pendingAction)}
        title={pendingAction?.title || ''}
        message={pendingAction?.message || ''}
        confirmLabel={pendingAction?.confirmLabel || 'ONAYLA'}
        onConfirm={handleConfirm}
        onCancel={() => setPendingAction(null)}
      />
    </div>
  );
};

export default IhaCommandPanel;
