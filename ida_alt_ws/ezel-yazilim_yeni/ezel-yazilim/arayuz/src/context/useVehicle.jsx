import { createContext, useContext } from 'react';

/**
 * Paylaşılan Vehicle context nesnesi.
 *
 * Gerçek Provider implementasyonu `VehicleContext.jsx` içindedir (backend WebSocket
 * auth, CommandGate farkındalığı, emergency latch, mission-upload zorunluluğu vb.
 * güvenlik mimarisini barındırır). Bu dosya kasıtlı olarak SADECE context nesnesini
 * ve `useVehicle` hook'unu tanımlar; kendi Provider'ını tanımlamaz. Aksi halde iki
 * farklı Provider aynı isimle var olur ve App.jsx'teki import yanlışlıkla
 * değiştirilirse tüm güvenlik kontrolleri (auth, emergency latch, mission-upload
 * şartı) sessizce devre dışı kalabilir.
 */
export const VehicleContext = createContext(null);

export const useVehicle = () => {
  const context = useContext(VehicleContext);
  if (!context) {
    throw new Error('useVehicle hooku bir VehicleProvider içerisinde kullanılmalıdır!');
  }
  return context;
};