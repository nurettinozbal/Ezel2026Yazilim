import { useVehicle } from '@/context/useVehicle';
import { RotateCcw, ZapOff, TriangleAlert } from 'lucide-react';
import styles from '../styles/KillSwitch.module.css'; // CSS Modülünü çektik

const KillSwitch = () => {
  const { triggerEmergency, isEmergencyActive, resetEmergency } = useVehicle();

  return (
    <div className={styles.container}>
      
      {/* ÜST UYARI YAZISI */}
      <div className={styles.header}>
        <TriangleAlert size={12} />
        ACİL DURUM KONTROLÜ
      </div>

      {/* ANA BUTON */}
      <button 
        onClick={triggerEmergency}
        className={`${styles.button} ${isEmergencyActive ? styles.active : ''}`}
      >
        {/* ŞİMŞEK İKONU */}
        <ZapOff size={24} color="white" strokeWidth={2} />
        
        {/* BUTON YAZISI */}
        <span className={styles.buttonText}>
          {isEmergencyActive ? 'SİSTEM KİLİTLENDİ' : 'ACİL DURDUR'}
        </span>
      </button>

      {isEmergencyActive && (
        <button
          onClick={resetEmergency}
          className={styles.resetButton}
          type="button"
        >
          <RotateCcw size={14} />
          KONTROLLÜ RESET
        </button>
      )}

    </div>
  );
};

export default KillSwitch;
