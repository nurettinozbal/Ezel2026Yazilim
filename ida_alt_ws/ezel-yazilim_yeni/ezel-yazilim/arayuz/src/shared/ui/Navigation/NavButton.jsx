import { NavLink } from 'react-router-dom';
import styles from './styles/NavButton.module.css'; // CSS importu

const NavButton = ({ to, icon, label }) => {
  const IconComponent = icon;

  return (
    <NavLink 
      to={to} 
      className={({ isActive }) => 
        // Aktif ise hem .link hem .active sınıfını ekle, değilse sadece .link
        `${styles.link} ${isActive ? styles.active : ''}`
      }
    >
      <IconComponent size={16} />
      <span>{label}</span>
    </NavLink>
  );
};

export default NavButton;
