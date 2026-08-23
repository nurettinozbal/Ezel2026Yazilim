import { describe, it, expect } from 'vitest';
import { render, screen } from '@testing-library/react';
import InfoCard from '@/features/TelemetryPanel/components/InfoCard';
import { Activity } from 'lucide-react';

describe('InfoCard', () => {
  it('renders title and value', () => {
    render(<InfoCard title="Voltaj" value={16.5} unit="V" icon={Activity} />);
    
    expect(screen.getByText('Voltaj')).toBeInTheDocument();
    expect(screen.getByText('16.5')).toBeInTheDocument();
    expect(screen.getByText('V')).toBeInTheDocument();
  });

  it('renders without icon gracefully', () => {
    render(<InfoCard title="Test" value="OK" unit="" />);
    expect(screen.getByText('Test')).toBeInTheDocument();
    expect(screen.getByText('OK')).toBeInTheDocument();
  });

  it('applies alert class when alert prop is true', () => {
    const { container } = render(
      <InfoCard title="Voltaj" value={13.5} unit="V" icon={Activity} alert={true} />
    );
    // alertMode class should be present on the card
    const card = container.firstChild;
    expect(card.className).toContain('alertMode');
  });

  it('does not apply alert class when alert is false', () => {
    const { container } = render(
      <InfoCard title="Voltaj" value={16.5} unit="V" icon={Activity} alert={false} />
    );
    const card = container.firstChild;
    expect(card.className).not.toContain('alertMode');
  });
});
