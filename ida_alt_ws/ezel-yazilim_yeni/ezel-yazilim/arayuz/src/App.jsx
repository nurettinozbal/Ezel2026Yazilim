import { BrowserRouter, Routes, Route } from 'react-router-dom';
import { VehicleProvider } from '@/context/VehicleContext';
import ErrorBoundary from '@/shared/ui/ErrorBoundary';
import MainLayout from '@/shared/layout/MainLayout';
import NavigationBar from '@/shared/ui/Navigation/NavigationBar';

// --- SAYFA BİLEŞENLERİ ---
import OperationPage from '@/pages/OperationPage';
import MissionPage from '@/pages/MissionPage';
import IhaPage from '@/pages/IhaPage';
import EngineeringPage from '@/features/Engineering/EngineeringPage';
import SettingsPage from '@/features/Settings/SettingsPage';
import BottomControlBar from '@/features/MissionControl/components/BottomControlBar';

function App() {
  return (
    <ErrorBoundary fallbackTitle="SİSTEM HATASI" fallbackMessage="Yer Kontrol İstasyonu beklenmeyen bir hatayla karşılaştı. Lütfen sayfayı yenileyin.">
      <VehicleProvider>
        <BrowserRouter>
          <MainLayout
            headerContent={<NavigationBar />} 
            content={
              <Routes>
                <Route path="/" element={<OperationPage />} />
                <Route path="/mission" element={<MissionPage />} />
                <Route path="/iha" element={<IhaPage />} />
                <Route path="/engineering" element={<EngineeringPage />} />
                <Route path="/settings" element={<SettingsPage />} />
              </Routes>
            }
            footer={<BottomControlBar />}
          />
        </BrowserRouter>
      </VehicleProvider>
    </ErrorBoundary>
  );
}

export default App;