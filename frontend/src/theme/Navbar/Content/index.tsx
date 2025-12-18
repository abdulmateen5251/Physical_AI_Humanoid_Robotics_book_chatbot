import React from 'react';
import OriginalNavbarContent from '@theme-original/Navbar/Content';
import AuthButton from '@site/src/components/AuthButton';
import LanguageToggle from '@site/src/components/LanguageToggle';

export default function NavbarContent(props) {
  return (
    <div style={{ display: 'flex', alignItems: 'center', width: '100%' }}>
      <OriginalNavbarContent {...props} />
      <div style={{ marginLeft: 'auto', paddingRight: '16px', display: 'flex', alignItems: 'center', gap: '12px' }}>
        <LanguageToggle />
        <AuthButton />
      </div>
    </div>
  );
}
