import MapView from '@/features/MapSystem/components/MapView';
import MissionPlanner from '@/features/MissionControl/components/MissionPlanner';
import ErrorBoundary from '@/shared/ui/ErrorBoundary';
import styles from './Pages.module.css';

// GÖREV PLANLAMA SAYFASI
const MissionPage = () => {

  // Telsiz Göndericileri (Harita dosyasına sinyal gönderir)
  const handleFlyToIda = () => window.dispatchEvent(new Event('FOCUS_IDA'));
  const handleFlyToIha = () => window.dispatchEvent(new Event('FOCUS_IHA'));
  const handleFlyToYki = () => window.dispatchEvent(new Event('FOCUS_YKI')); // --- YENİ EKLENDİ ---

  return (
    <div className={styles.missionGrid}>
       <div className={styles.missionSidebar}>
          <MissionPlanner />
       </div>
       
       <div className={styles.missionMapArea} style={{ position: 'relative' }}>
          
          <ErrorBoundary fallbackTitle="HARİTA HATASI" fallbackMessage="Harita modülü yüklenemedi. Görev planlama paneli aktif kalmaya devam ediyor.">
            <MapView planningEnabled />
          </ErrorBoundary>

          {/* --- ÜST PANEL: BUTONLAR VE ETİKET --- */}
          <div style={{
            position: 'absolute',
            top: '20px',
            right: '20px',
            zIndex: 1000,
            display: 'flex',
            alignItems: 'center', 
            gap: '15px' 
          }}>
            
            {/* YKİ BUL BUTONU (MOR) - YENİ */}
            <button
              onClick={handleFlyToYki}
              style={{
                backgroundColor: '#9b59b6', color: '#fff', border: '1px solid #8e44ad', padding: '8px 16px',
                borderRadius: '4px', fontWeight: 'bold', cursor: 'pointer', fontSize: '12px',
                boxShadow: '0 4px 6px rgba(0,0,0,0.5)', transition: 'transform 0.1s'
              }}
              onMouseDown={(e) => e.target.style.transform = 'scale(0.95)'}
              onMouseUp={(e) => e.target.style.transform = 'scale(1)'}
            >
              🎯 YKİ BUL
            </button>

            {/* İDA BUL BUTONU (KIRMIZI) */}
            <button
              onClick={handleFlyToIda}
              style={{
                backgroundColor: '#ef4444', color: '#fff', border: '1px solid #c0392b', padding: '8px 16px',
                borderRadius: '4px', fontWeight: 'bold', cursor: 'pointer', fontSize: '12px',
                boxShadow: '0 4px 6px rgba(0,0,0,0.5)', transition: 'transform 0.1s'
              }}
              onMouseDown={(e) => e.target.style.transform = 'scale(0.95)'}
              onMouseUp={(e) => e.target.style.transform = 'scale(1)'}
            >
              🎯 İDA BUL
            </button>

            {/* İHA BUL BUTONU (TURUNCU) */}
            <button
              onClick={handleFlyToIha}
              style={{
                backgroundColor: '#e67e22', color: '#fff', border: '1px solid #d35400', padding: '8px 16px',
                borderRadius: '4px', fontWeight: 'bold', cursor: 'pointer', fontSize: '12px',
                boxShadow: '0 4px 6px rgba(0,0,0,0.5)', transition: 'transform 0.1s'
              }}
              onMouseDown={(e) => e.target.style.transform = 'scale(0.95)'}
              onMouseUp={(e) => e.target.style.transform = 'scale(1)'}
            >
              🎯 İHA BUL
            </button>

            {/* MEVCUT SARI ETİKET */}
            <div className={styles.missionModeLabel} style={{ margin: 0, position: 'relative', top: 'auto', right: 'auto' }}>
               GÖREV PLANLAMA MODU
            </div>
            
          </div>
       </div>
    </div>
  );
};

export default MissionPage;
