import { useState } from 'react';
import { useVehicle } from '@/context/useVehicle';
import InfoCard from './InfoCard';
import KillSwitch from '../../EmergencySystem/components/KillSwitch';
import ColorInputModal from '@/shared/ui/ColorInputModal/ColorInputModal';
import styles from '../styles/Telemetry.module.css'; // CSS Modülü
import { Activity, Navigation, Anchor, Zap, Gauge, Target, Route, Edit3, Lock } from 'lucide-react';

const DashboardView = () => {
  const { telemetry, updateDroneColorManuel } = useVehicle();
  const { ida, iha, system } = telemetry;
  const modeColor = ida.mode === 'AUTO' ? '#2ecc71' : '#f1c40f';
  const [isColorModalOpen, setIsColorModalOpen] = useState(false);
  const isTargetLocked = Boolean(system?.target_locked || system?.mission_started);

  // Backend'in kabul ettiği hedef renkleri (bkz. backend/services/target_manager.py
  // VALID_COLORS) ile birebir eşleşir. Şartnamede hedef renkleri DSB (Daha Sonra
  // Belirlenecek) olduğu için resmi renkler açıklandığında bu harita güncellenmelidir.
  const COLOR_HEX_MAP = {
    KIRMIZI: '#ef4444',
    YEŞİL: '#2ecc71',
    SİYAH: '#18181b',
  };

  const getDetectedColorHex = (colorName) => COLOR_HEX_MAP[colorName] || '#71717a';

  const handleColorClick = () => {
    // Şartname §5.5.3.1: hedef bilgisi İDA harekete başladıktan sonra aktarılamaz.
    if (isTargetLocked) return;
    setIsColorModalOpen(true);
  };

  const handleColorConfirm = (input) => {
    setIsColorModalOpen(false);
    if (input.trim() !== "") {
      updateDroneColorManuel(input);
    }
  };

  const handleColorCancel = () => {
    setIsColorModalOpen(false);
  };

  return (
    <div className={styles.dashboardContainer}>
      <div className={styles.sectionTitle}>İDA (GEMİ) DURUMU</div>
      
      <InfoCard title="Kontrol Modu" value={ida.mode} unit="" icon={Anchor} color={modeColor} />
      <InfoCard title="Yer Hızı" value={ida.speed} unit="m/s" icon={Gauge} color="#3498db" />

      {/* --- DONANIMSAL GÜÇ HATTI AYRILDIĞI İÇİN GİZLENDİ --- */}
      {/* <div className={styles.gridContainer}>
         <InfoCard title="Voltaj" value={ida.voltage} unit="V" icon={Zap} color={ida.voltage < 14.8 ? '#e74c3c' : '#f1c40f'} alert={ida.voltage < 14.0} />
         <InfoCard title="Akım" value={ida.current} unit="A" icon={Activity} color="#e67e22" />
      </div> 
      */}

      {ida.mode === 'AUTO' && (
        <div className={styles.nextWpBox}>
          <InfoCard title="Sıradaki Hedef" value={`WP #${ida.current_wp}`} unit={`(${ida.dist_to_wp}m)`} icon={Route} color="#3498db" />
        </div>
      )}

      <InfoCard title="Yönelim" value={ida.heading} unit="°" icon={Navigation} color="#9b59b6" />

      <div className={styles.sectionTitle}>İHA (DRONE) DURUMU</div>
      <InfoCard title="İrtifa (AGL)" value={iha.alt} unit="m" icon={Activity} color="#e67e22" />
      
      {/* YAZILABİLİR ALAN — hedef kilitlendikten/görev başladıktan sonra salt okunur */}
      <div
        className={styles.manualInputWrapper}
        onClick={handleColorClick}
        style={{ cursor: isTargetLocked ? 'not-allowed' : 'pointer' }}
        title={
          isTargetLocked
            ? 'Hedef kilitli; görev başladıktan sonra değiştirilemez (şartname §5.5.3.1)'
            : 'Rengi EL İLE girmek için tıklayın'
        }
      >
        <InfoCard
          title="Tespit Edilen Renk"
          value={iha.detected_color}
          unit=""
          icon={Target}
          color={getDetectedColorHex(iha.detected_color)}
        />
        <div className={styles.editIcon}>
          {isTargetLocked ? <Lock size={14} color="white" /> : <Edit3 size={14} color="white" />}
        </div>
      </div>
      
      <div className={styles.bottomSection}>
        <KillSwitch />
      </div>

      {/* RENK GİRİŞ MODALI */}
      {isColorModalOpen && (
        <ColorInputModal
          isOpen={isColorModalOpen}
          defaultValue={iha.detected_color}
          onConfirm={handleColorConfirm}
          onCancel={handleColorCancel}
        />
      )}
    </div>
  );
};

export default DashboardView;
