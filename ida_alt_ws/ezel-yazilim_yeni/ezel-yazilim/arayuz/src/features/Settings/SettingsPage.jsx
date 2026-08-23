import { useState } from 'react';
import { useVehicle } from '@/context/useVehicle';
import { Save, Radio } from 'lucide-react';
import LogTerminal from '@/features/TelemetryPanel/components/LogTerminal';
import ConfirmModal from '@/shared/ui/ConfirmModal/ConfirmModal';
import styles from './styles/SettingsPage.module.css'; // CSS Modülü

const SettingsPage = () => {
  const { addLog, sendCommand, telemetry } = useVehicle();
  const [freq, setFreq] = useState("915000");
  const [netId, setNetId] = useState("25");
  const [vehicle, setVehicle] = useState("ida");
  const [isConfirmOpen, setIsConfirmOpen] = useState(false);

  const vehicleState = vehicle === 'ida' ? telemetry.ida : telemetry.iha;
  const isArmed = Boolean(vehicleState?.armed);
  const isConnected = Boolean(vehicleState?.connected);
  const freqValue = Number(freq);
  const isFreqValid = Number.isFinite(freqValue) && freqValue > 0;
  const canWrite = isFreqValid && isConnected && !isArmed;

  const handleSaveClick = () => {
    if (!canWrite) return;
    setIsConfirmOpen(true);
  };

  const handleConfirm = () => {
    setIsConfirmOpen(false);
    addLog(
      `${vehicle.toUpperCase()} modemine frekans yazma başlatıldı: ${freqValue} kHz`,
      "WARNING",
    );
    sendCommand('SET_RADIO_FREQUENCY', { vehicle, freq_khz: freqValue });
  };

  return (
    <div className={styles.container}>

      <h2 className={styles.pageTitle}>
        SİSTEM KONFİGÜRASYONU & LOGLAR
      </h2>

      {/* AYARLAR KUTUSU */}
      <div className={styles.settingsBox}>
        <h3 className={styles.boxHeader}>
          <Radio size={20} /> Haberleşme Modülü Ayarları
        </h3>

        <div className={styles.vehicleSelector}>
          <button
            type="button"
            onClick={() => setVehicle('ida')}
            className={`${styles.vehicleButton} ${vehicle === 'ida' ? styles.vehicleButtonActive : ''}`}
          >
            İDA MODEMİ
          </button>
          <button
            type="button"
            onClick={() => setVehicle('iha')}
            className={`${styles.vehicleButton} ${vehicle === 'iha' ? styles.vehicleButtonActive : ''}`}
          >
            İHA MODEMİ
          </button>
        </div>

        <div className={styles.inputGrid}>
          <div>
            <label className={styles.label}>Frekans (kHz)</label>
            <input
              type="text"
              value={freq}
              onChange={e => setFreq(e.target.value)}
              className={styles.input}
            />
            <div className={styles.fieldNote}>
              Modeme min={freqValue || '—'} / max={freqValue ? freqValue + 1000 : '—'} kHz olarak yazılır.
            </div>
          </div>
          <div>
            <label className={styles.label}>Net ID (Takım ID)</label>
            <input
              type="text"
              value={netId}
              onChange={e => setNetId(e.target.value)}
              className={styles.input}
            />
            <div className={styles.fieldNote}>
              Net ID bu akışta modeme yazılmaz; yalnız kayıt amaçlı gösterilir.
            </div>
          </div>
        </div>

        <button
          onClick={handleSaveClick}
          disabled={!canWrite}
          title={
            !isConnected
              ? `${vehicle.toUpperCase()} bağlantısı yok`
              : isArmed
                ? 'Frekans yalnız araç DISARMED iken değiştirilebilir'
                : !isFreqValid
                  ? 'Geçerli bir frekans giriniz'
                  : 'Modeme frekansı yaz'
          }
          className={styles.saveButton}
        >
          <Save size={18} /> FREKANSI MODEME YAZ
        </button>

        <div className={styles.warningNote}>
          ⚠ Bu işlem Pixhawk’ı yeniden başlatır ve telemetri bağlantısını geçici olarak keser.
          Yalnız araç karada ve DISARMED iken kullanılmalıdır.
        </div>
      </div>

      {/* TERMİNAL ALANI */}
      <div className={styles.terminalSection}>
        <h3 className={styles.terminalTitle}>CANLI SİSTEM ÇIKTISI</h3>

        {/* Terminal yüksekliği sabit */}
        <LogTerminal height="500px" />
      </div>

      <ConfirmModal
        isOpen={isConfirmOpen}
        title="FREKANS YAZMA ONAYI"
        message={
          `${vehicle.toUpperCase()} modemine ${freqValue} kHz yazılacak.\n\n` +
          `Bu işlem Pixhawk'ı yeniden başlatır ve telemetri bağlantısı ~10 saniye kesilir. ` +
          `Aracın karada ve DISARMED olduğundan emin olun.`
        }
        confirmLabel="YAZ"
        onConfirm={handleConfirm}
        onCancel={() => setIsConfirmOpen(false)}
      />

    </div>
  );
};

export default SettingsPage;
