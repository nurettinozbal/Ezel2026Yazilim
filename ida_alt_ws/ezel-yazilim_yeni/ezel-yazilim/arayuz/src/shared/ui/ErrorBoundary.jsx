import { Component } from 'react';

/**
 * ErrorBoundary — React component ağacında oluşan render hatalarını yakalar.
 * Tüm uygulama beyaz ekrana düşmek yerine, kullanıcıya bilgilendirici bir hata mesajı gösterir.
 * Bu bileşen davranışı değiştirmez; yalnız crash durumunda graceful degradation sağlar.
 */
class ErrorBoundary extends Component {
  constructor(props) {
    super(props);
    this.state = { hasError: false, error: null };
  }

  static getDerivedStateFromError(error) {
    return { hasError: true, error };
  }

  componentDidCatch(error, errorInfo) {
    console.error('[ErrorBoundary] Yakalanan hata:', error, errorInfo);
  }

  handleRetry = () => {
    this.setState({ hasError: false, error: null });
  };

  render() {
    if (this.state.hasError) {
      const { fallbackTitle = 'Modül Hatası', fallbackMessage } = this.props;
      return (
        <div style={{
          display: 'flex',
          flexDirection: 'column',
          alignItems: 'center',
          justifyContent: 'center',
          height: '100%',
          width: '100%',
          background: '#0a0a0a',
          color: '#ef4444',
          fontFamily: 'monospace',
          padding: '20px',
          textAlign: 'center',
          gap: '12px',
        }}>
          <div style={{ fontSize: '14px', fontWeight: 'bold', letterSpacing: '1px' }}>
            ⚠ {fallbackTitle}
          </div>
          <div style={{ fontSize: '11px', color: '#71717a', maxWidth: '400px' }}>
            {fallbackMessage || `Bir bileşen beklenmeyen bir hatayla karşılaştı. Sistem çalışmaya devam ediyor.`}
          </div>
          <div style={{ fontSize: '10px', color: '#52525b', maxWidth: '500px', wordBreak: 'break-word' }}>
            {this.state.error?.message}
          </div>
          <button
            onClick={this.handleRetry}
            style={{
              marginTop: '8px',
              padding: '6px 16px',
              background: '#27272a',
              color: '#a1a1aa',
              border: '1px solid #3f3f46',
              borderRadius: '4px',
              fontSize: '11px',
              cursor: 'pointer',
              fontFamily: 'monospace',
            }}
          >
            YENİDEN DENE
          </button>
        </div>
      );
    }

    return this.props.children;
  }
}

export default ErrorBoundary;
