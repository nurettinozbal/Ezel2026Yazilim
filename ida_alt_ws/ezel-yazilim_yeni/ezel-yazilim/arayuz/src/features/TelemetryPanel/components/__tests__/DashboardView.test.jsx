import { describe, it, expect, vi } from 'vitest';
import { render, screen, fireEvent } from '@testing-library/react';
import { VehicleContext } from '@/context/useVehicle';
import DashboardView from '@/features/TelemetryPanel/components/DashboardView';

const buildContext = (systemOverrides = {}) => ({
  telemetry: {
    ida: { mode: 'DISARMED', speed: 0, heading: 0, current_wp: 0, dist_to_wp: 0 },
    iha: { alt: 0, detected_color: 'KIRMIZI' },
    system: { target_locked: false, mission_started: false, ...systemOverrides },
  },
  updateDroneColorManuel: vi.fn(),
  triggerEmergency: vi.fn(),
  resetEmergency: vi.fn(),
  isEmergencyActive: false,
});

const renderDashboard = (systemOverrides) => {
  const value = buildContext(systemOverrides);
  render(
    <VehicleContext.Provider value={value}>
      <DashboardView />
    </VehicleContext.Provider>,
  );
  return value;
};

describe('DashboardView hedef renk kilidi', () => {
  it('hedef kilitli değilken renk giriş modalını açar', () => {
    renderDashboard();

    fireEvent.click(screen.getByTitle('Rengi EL İLE girmek için tıklayın'));

    expect(screen.getByText('HEDEF RENK GİRİŞİ')).toBeInTheDocument();
  });

  it('hedef kilitliyken modalı açmaz', () => {
    renderDashboard({ target_locked: true });

    const card = screen.getByTitle(
      'Hedef kilitli; görev başladıktan sonra değiştirilemez (şartname §5.5.3.1)',
    );
    fireEvent.click(card);

    expect(screen.queryByText('HEDEF RENK GİRİŞİ')).not.toBeInTheDocument();
  });

  it('görev başladıysa hedef değişimini engeller', () => {
    renderDashboard({ mission_started: true });

    const card = screen.getByTitle(
      'Hedef kilitli; görev başladıktan sonra değiştirilemez (şartname §5.5.3.1)',
    );
    fireEvent.click(card);

    expect(screen.queryByText('HEDEF RENK GİRİŞİ')).not.toBeInTheDocument();
  });
});
