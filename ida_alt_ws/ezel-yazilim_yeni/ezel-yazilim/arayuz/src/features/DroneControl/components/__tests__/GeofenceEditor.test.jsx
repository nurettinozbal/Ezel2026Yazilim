import { describe, it, expect, vi } from 'vitest';
import { render, screen, fireEvent } from '@testing-library/react';
import { VehicleContext } from '@/context/useVehicle';
import GeofenceEditor from '@/features/DroneControl/components/GeofenceEditor';

const buildContext = (overrides = {}) => ({
  geofencePoints: [],
  isIhaInsideGeofence: null,
  undoGeofencePoint: vi.fn(),
  clearGeofence: vi.fn(),
  ...overrides,
});

const renderEditor = (overrides, props = {}) => {
  const value = buildContext(overrides);
  render(
    <VehicleContext.Provider value={value}>
      <GeofenceEditor drawing={false} onToggleDrawing={vi.fn()} {...props} />
    </VehicleContext.Provider>,
  );
  return value;
};

const THREE_POINTS = [
  { lat: 40, lon: 28 },
  { lat: 41, lon: 28 },
  { lat: 41, lon: 29 },
];

describe('GeofenceEditor durum bannerı', () => {
  it('3ten az nokta varken SINIR TANIMSIZ gösterir', () => {
    renderEditor({ geofencePoints: [{ lat: 40, lon: 28 }] });

    expect(screen.getByText('SINIR TANIMSIZ')).toBeInTheDocument();
    expect(screen.getByText(/En az 3 nokta gerekli \(1 girildi\)/)).toBeInTheDocument();
  });

  it('İHA bölge içindeyken BÖLGE İÇİNDE gösterir', () => {
    renderEditor({ geofencePoints: THREE_POINTS, isIhaInsideGeofence: true });

    expect(screen.getByText('BÖLGE İÇİNDE')).toBeInTheDocument();
  });

  it('İHA bölge dışındayken Parkur-3 riski uyarısı gösterir', () => {
    renderEditor({ geofencePoints: THREE_POINTS, isIhaInsideGeofence: false });

    expect(screen.getByText('BÖLGE DIŞI — PARKUR-3 RİSKİ')).toBeInTheDocument();
    expect(screen.getByText(/§5.5.3.1/)).toBeInTheDocument();
  });

  it('sınır tanımlı ama konum yokken ihlal olarak göstermez', () => {
    renderEditor({ geofencePoints: THREE_POINTS, isIhaInsideGeofence: null });

    expect(screen.getByText('İHA KONUMU YOK')).toBeInTheDocument();
    expect(screen.queryByText('BÖLGE DIŞI — PARKUR-3 RİSKİ')).not.toBeInTheDocument();
  });
});

describe('GeofenceEditor aksiyonları', () => {
  it('nokta yokken sil/temizle butonları devre dışıdır', () => {
    renderEditor({ geofencePoints: [] });

    expect(screen.getByRole('button', { name: /SON NOKTAYI SİL/ })).toBeDisabled();
    expect(screen.getByRole('button', { name: /TEMİZLE/ })).toBeDisabled();
  });

  it('nokta varken son noktayı silme çağrılır', () => {
    const value = renderEditor({ geofencePoints: THREE_POINTS });

    fireEvent.click(screen.getByRole('button', { name: /SON NOKTAYI SİL/ }));

    expect(value.undoGeofencePoint).toHaveBeenCalledTimes(1);
  });

  it('çizim modu açıkken buton etiketi değişir', () => {
    renderEditor({}, { drawing: true });

    expect(screen.getByRole('button', { name: /ÇİZİMİ BİTİR/ })).toBeInTheDocument();
  });
});
