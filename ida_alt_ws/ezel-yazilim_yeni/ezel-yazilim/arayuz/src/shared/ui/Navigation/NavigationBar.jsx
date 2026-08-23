import { AppWindow, Map, Plane, Database, Settings } from 'lucide-react';
import NavButton from './NavButton';
import styles from './styles/NavigationBar.module.css';

const NavigationBar = () => {
  return (
    <nav className={styles.nav}>
      <NavButton to="/" icon={AppWindow} label="OPERASYON" />
      <NavButton to="/mission" icon={Map} label="GÖREV PLANLAMA" />
      <NavButton to="/iha" icon={Plane} label="İHA OPERASYON" />
      <NavButton to="/engineering" icon={Database} label="MÜHENDİSLİK & TEST" />
      <NavButton to="/settings" icon={Settings} label="SİSTEM AYARLARI" />
    </nav>
  );
};

export default NavigationBar;