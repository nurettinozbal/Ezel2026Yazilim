import { describe, it, expect, vi } from 'vitest';
import { render, screen, fireEvent } from '@testing-library/react';
import { VehicleContext } from '@/context/useVehicle';
import IhaCommandPanel from '@/features/DroneControl/components/IhaCommandPanel';

const buildContext = ({ ihaOverrides = {}, systemOverrides = {}, isEmergencyActive = false } = {}) => ({
  telemetry: {
    iha: { armed: false, mode: 'LOITER', ...ihaOverrides },
    system: { iha_link: 'CONNECTED', ...systemOverrides },
  },
  isEmergencyActive,
  toggleArmIha: vi.fn(),
  setIhaMode: vi.fn(),
  triggerIhaReturnHome: vi.fn(),
});

const renderPanel = (options) => {
  const value = buildContext(options);
  render(
    <VehicleContext.Provider value={value}>
      <IhaCommandPanel />
    </VehicleContext.Provider>,
  );
  return value;
};

describe('IhaCommandPanel emergency kısıtları', () => {
  it('emergency aktifken yalnız LOITER/RTL modlarına izin verir', () => {
    renderPanel({ isEmergencyActive: true });

    expect(screen.getByRole('button', { name: 'LOITER' })).toBeEnabled();
    expect(screen.getByRole('button', { name: 'RTL' })).toBeEnabled();
    expect(screen.getByRole('button', { name: 'AUTO' })).toBeDisabled();
    expect(screen.getByRole('button', { name: 'GUIDED' })).toBeDisabled();
  });

  it('emergency aktifken ARM butonu devre dışıdır', () => {
    renderPanel({ isEmergencyActive: true });

    expect(screen.getByRole('button', { name: /ARM \(AÇ\)/ })).toBeDisabled();
  });

  it('emergency yokken tüm modlar seçilebilir', () => {
    renderPanel();

    ['LOITER', 'GUIDED', 'AUTO', 'RTL'].forEach((mode) => {
      expect(screen.getByRole('button', { name: mode })).toBeEnabled();
    });
  });
});

describe('IhaCommandPanel bağlantı durumu', () => {
  it('İHA bağlı değilken komutları kilitler ve uyarı gösterir', () => {
    renderPanel({ systemOverrides: { iha_link: 'DISCONNECTED' } });

    expect(screen.getByText(/İHA bağlantısı yok/)).toBeInTheDocument();
    expect(screen.getByRole('button', { name: /ARM \(AÇ\)/ })).toBeDisabled();
    expect(screen.getByRole('button', { name: 'LOITER' })).toBeDisabled();
  });
});

describe('IhaCommandPanel onay akışı', () => {
  it('mod değişimi ancak onaydan sonra gönderilir', () => {
    const value = renderPanel();

    fireEvent.click(screen.getByRole('button', { name: 'AUTO' }));
    expect(value.setIhaMode).not.toHaveBeenCalled();

    fireEvent.click(screen.getByRole('button', { name: 'MODU DEĞİŞTİR' }));
    expect(value.setIhaMode).toHaveBeenCalledWith('AUTO');
  });

  it('onay iptal edilirse komut gönderilmez', () => {
    const value = renderPanel();

    fireEvent.click(screen.getByRole('button', { name: /ARM \(AÇ\)/ }));
    fireEvent.click(screen.getByRole('button', { name: 'İPTAL' }));

    expect(value.toggleArmIha).not.toHaveBeenCalled();
  });

  it('RTL onaylandığında eve dönüş komutu gönderilir', () => {
    const value = renderPanel();

    fireEvent.click(screen.getByRole('button', { name: /ACİL: EVE DÖNÜŞ/ }));
    fireEvent.click(screen.getByRole('button', { name: 'RTL GÖNDER' }));

    expect(value.triggerIhaReturnHome).toHaveBeenCalledTimes(1);
  });

  it('armed durumdayken DISARM etiketi gösterilir', () => {
    renderPanel({ ihaOverrides: { armed: true } });

    expect(screen.getByRole('button', { name: /DISARM \(KAPAT\)/ })).toBeInTheDocument();
  });
});
