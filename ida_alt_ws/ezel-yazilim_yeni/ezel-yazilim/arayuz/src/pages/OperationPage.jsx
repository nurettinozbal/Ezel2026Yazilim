import MapView from '@/features/MapSystem/components/MapView';
import DashboardView from '@/features/TelemetryPanel/components/DashboardView';
import ErrorBoundary from '@/shared/ui/ErrorBoundary';
import styles from './Pages.module.css';

// OPERASYON SAYFASI (Telemetri ve Tam Ekran Harita)
const OperationPage = () => (
  <div className={styles.operationGrid}>
    
    {/* SOL SÜTUN: TELEMETRİ PANELİ */}
    <div className={styles.telemetrySidebar}>
      <DashboardView />
    </div>

    {/* SAĞ SÜTUN: İÇERİK ALANI (Sadece Harita) */}
    <div className={styles.contentArea}>
      
      {/* HARİTA (Tüm Alanı Kaplar) */}
      <div className={styles.mapSection}>
        <ErrorBoundary fallbackTitle="HARİTA HATASI" fallbackMessage="Harita modülü yüklenemedi. Telemetri verileri aktif kalmaya devam ediyor.">
          <MapView />
        </ErrorBoundary>
        <div className={styles.mapOverlayLabel}>
           SATELLITE VIEW (ESRI)
        </div>
      </div>
      
    </div>
  </div>
);

export default OperationPage;