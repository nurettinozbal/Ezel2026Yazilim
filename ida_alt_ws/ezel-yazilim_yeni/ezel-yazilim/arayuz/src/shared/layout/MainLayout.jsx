import SystemClock from '../ui/Navigation/SystemClock';
import styles from './MainLayout.module.css';

const MainLayout = ({ headerContent, content, footer }) => {
  return (
    <div className={styles.container}>
      
      {/* 1. ÜST BAŞLIK */}
      <div className={styles.header}>
        
        {/* SOL: LOGO + YAZI */}
        <div className={styles.brand}>
          {/* Logo */}
          <img 
            src="/logo.png" 
            alt="TEKNOFEST İDA" 
            className={styles.logoImage} 
          />

          {/* YENİ: Yanına Yazı Ekledik */}
          <div className={styles.brandText}>
            <span className={styles.title}>EZEL</span>
            <span className={styles.subtitle}>YER KONTROL İSTASYONU</span>
          </div>
        </div>

        {/* SAĞ: SAAT */}
        <SystemClock />
      </div>

      {/* 2. MENÜ ALANI */}
      <div className={styles.navigationBar}>
        {headerContent}
      </div>

      {/* 3. İÇERİK ALANI */}
      <div className={styles.mainContent}>
        {content}
      </div>

      {/* 4. ALT BİLGİ (Footer) */}
      <div className={styles.footer}>
        {footer}
      </div>
    </div>
  );
};

export default MainLayout;  